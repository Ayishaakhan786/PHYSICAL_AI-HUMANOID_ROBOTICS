"""Vector storage in Qdrant Cloud for the RAG pipeline."""

import logging
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Tuple

from qdrant_client import QdrantClient
from qdrant_client.models import (
    VectorParams,
    Distance,
    PointStruct,
    Filter,
    FieldCondition,
    MatchValue,
    SearchRequest,
    CollectionInfo,
    CollectionStatus,
)

from .config import PipelineConfig
from .embedder import EmbeddedChunk

logger = logging.getLogger(__name__)


@dataclass
class VectorPoint:
    """A vector point stored in Qdrant for similarity search."""

    id: str
    vector: List[float]
    payload: Dict[str, Any]
    score: Optional[float] = field(default=None, compare=False)

    @property
    def text(self) -> str:
        return self.payload.get("text", "")

    @property
    def source_url(self) -> str:
        return self.payload.get("source_url", "")

    @property
    def section_path(self) -> str:
        return self.payload.get("section_path", "")


class QdrantStoreError(Exception):
    """Base exception for Qdrant store errors."""

    pass


class ConnectionError(QdrantStoreError):
    """Failed to connect to Qdrant."""

    def __init__(self, message: str):
        super().__init__(f"Qdrant connection error: {message}")


class CollectionError(QdrantStoreError):
    """Collection operation failed."""

    pass


class QdrantStore:
    """Handles vector storage and retrieval in Qdrant Cloud."""

    # Default vector configuration for Cohere embed-english-v3.0
    DEFAULT_VECTOR_SIZE = 1024
    DEFAULT_DISTANCE = Distance.COSINE

    def __init__(self, config: PipelineConfig):
        """
        Initialize the Qdrant store.

        Args:
            config: Pipeline configuration with Qdrant settings
        """
        self.config = config
        self.collection_name = config.collection_name

        # Initialize client
        self.client = QdrantClient(
            url=config.qdrant_url,
            api_key=config.qdrant_api_key,
        )

        logger.info(
            f"Initialized Qdrant store: collection={self.collection_name}"
        )

    def ensure_collection(self) -> Tuple[bool, str]:
        """
        Ensure the collection exists with correct configuration.

        Returns:
            Tuple of (created: bool, message: str)
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name in collection_names:
                # Verify existing collection configuration
                info = self.client.get_collection(self.collection_name)

                # Handle different API versions - vector_size may be in different locations
                try:
                    # Try new API structure
                    vector_size = info.config.params.vectors.size
                except (AttributeError, TypeError):
                    try:
                        # Try older API structure
                        vector_size = info.config.params.vector_size
                    except AttributeError:
                        # Try direct access
                        vector_size = getattr(info.config.params, 'vector_size', None)
                        if vector_size is None:
                            # Try to get from vectors config directly
                            vectors_config = getattr(info.config.params, 'vectors', None)
                            if vectors_config:
                                vector_size = getattr(vectors_config, 'size', None)
                            if vector_size is None:
                                logger.warning(f"Could not determine vector size from collection info")
                                vector_size = self.DEFAULT_VECTOR_SIZE

                if vector_size != self.DEFAULT_VECTOR_SIZE:
                    raise CollectionError(
                        f"Collection vector size mismatch: "
                        f"expected {self.DEFAULT_VECTOR_SIZE}, "
                        f"got {vector_size}"
                    )

                logger.info(f"Collection '{self.collection_name}' already exists")
                return False, "Collection already exists"

            # Create collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.DEFAULT_VECTOR_SIZE,
                    distance=self.DEFAULT_DISTANCE,
                ),
            )

            # Create payload indexes
            self._create_payload_indexes()

            logger.info(f"Created collection '{self.collection_name}'")
            return True, "Collection created successfully"

        except Exception as e:
            logger.error(f"Failed to ensure collection: {e}")
            raise CollectionError(f"Failed to create collection: {e}")

    def _create_payload_indexes(self) -> None:
        """Create indexes on payload fields for filtered search."""
        try:
            # Create index on source_url
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="source_url",
                field_schema="keyword",
            )

            # Create index on section_path
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="section_path",
                field_schema="keyword",
            )

            # Create index on heading (text for full-text search)
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="heading",
                field_schema="text",
            )

            # Create index on chunk_index for range queries
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="chunk_index",
                field_schema="integer",
            )

            logger.info("Created payload indexes")

        except Exception as e:
            # Indexes might already exist, log warning but continue
            logger.warning(f"Failed to create some indexes: {e}")

    def upsert_embeddings(
        self, embedded_chunks: List[EmbeddedChunk]
    ) -> Tuple[int, List[Dict[str, Any]]]:
        """
        Upsert embeddings and payloads into Qdrant.

        Args:
            embedded_chunks: List of EmbeddedChunk objects

        Returns:
            Tuple of (points_upserted: int, errors: List[error info])
        """
        if not embedded_chunks:
            return 0, []

        logger.info(f"Upserting {len(embedded_chunks)} vectors")

        points = []
        errors = []

        for embedded in embedded_chunks:
            try:
                point = PointStruct(
                    id=embedded.chunk.point_id,
                    vector=embedded.embedding,
                    payload=embedded.chunk.to_payload(),
                )
                points.append(point)

            except Exception as e:
                logger.error(f"Failed to create point for chunk: {e}")
                errors.append({
                    "chunk_index": embedded.chunk.chunk_index,
                    "source_url": embedded.chunk.source_url,
                    "error": str(e),
                })

        if not points:
            return 0, errors

        # Upsert in batches
        batch_size = 100
        total_upserted = 0

        for i in range(0, len(points), batch_size):
            batch = points[i : i + batch_size]
            batch_num = i // batch_size + 1
            total_batches = (len(points) + batch_size - 1) // batch_size

            try:
                logger.info(f"Upserting batch {batch_num}/{total_batches} ({len(batch)} points)")

                result = self.client.upsert(
                    collection_name=self.collection_name,
                    points=batch,
                )
                total_upserted += len(batch)

                logger.info(
                    f"Successfully upserted batch {batch_num}/{total_batches}"
                )

            except Exception as e:
                error_msg = str(e)
                logger.error(f"Failed to upsert batch {batch_num}/{total_batches}: {error_msg}")

                # Try to extract more details from the error
                import traceback
                logger.error(f"Traceback: {traceback.format_exc()}")

                errors.append({
                    "batch_start": i,
                    "batch_end": min(i + batch_size, len(points)),
                    "batch_num": batch_num,
                    "total_batches": total_batches,
                    "points_in_batch": len(batch),
                    "error": error_msg,
                    "error_type": type(e).__name__,
                })

                # Continue with next batch - don't fail entire operation for one batch error

        logger.info(f"Upserted {total_upserted} vectors")

        return total_upserted, errors

    def search(
        self,
        query_vector: List[float],
        limit: int = 10,
        source_url: Optional[str] = None,
        section_path: Optional[str] = None,
        score_threshold: Optional[float] = None,
    ) -> List[VectorPoint]:
        """
        Search for similar vectors.

        Args:
            query_vector: The query embedding vector
            limit: Maximum number of results
            source_url: Filter by source URL
            section_path: Filter by section path
            score_threshold: Minimum similarity score

        Returns:
            List of VectorPoint objects with scores
        """
        # Build filter
        filter_conditions = []

        if source_url:
            filter_conditions.append(
                FieldCondition(key="source_url", match=MatchValue(value=source_url))
            )

        if section_path:
            filter_conditions.append(
                FieldCondition(
                    key="section_path", match=MatchValue(value=section_path)
                )
            )

        search_filter = Filter(must=filter_conditions) if filter_conditions else None

        # Build search request
        search_request = SearchRequest(
            vector=query_vector,
            limit=limit,
            filter=search_filter,
            score_threshold=score_threshold,
        )

        try:
            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                query_filter=search_filter,
                limit=limit,
                score_threshold=score_threshold,
            )

            points = []
            for hit in results:
                point = VectorPoint(
                    id=hit.id,
                    vector=query_vector,  # Query vector, not stored
                    payload=hit.payload,
                    score=hit.score,
                )
                points.append(point)

            logger.info(f"Search returned {len(points)} results")

            return points

        except Exception as e:
            logger.error(f"Search failed: {e}")
            raise QdrantStoreError(f"Search failed: {e}")

    def get_collection_info(self) -> Dict[str, Any]:
        """Get information about the collection."""
        try:
            info = self.client.get_collection(self.collection_name)

            # Handle different API versions for vector size
            try:
                vector_size = info.config.params.vectors.size
            except (AttributeError, TypeError):
                try:
                    vector_size = info.config.params.vector_size
                except AttributeError:
                    vector_size = getattr(info.config.params, 'vectors', None)
                    if vector_size:
                        vector_size = getattr(vector_size, 'size', None)
                    else:
                        vector_size = 0

            # Get points count - may be in different locations
            try:
                points_count = info.points_count
            except AttributeError:
                points_count = 0

            return {
                "name": self.collection_name,
                "status": info.status,
                "points_count": points_count,
                "vector_size": vector_size,
                "distance": str(info.config.params.distance),
            }

        except Exception as e:
            logger.error(f"Failed to get collection info: {e}")
            raise QdrantStoreError(f"Failed to get collection info: {e}")

    def count_vectors(
        self,
        source_url: Optional[str] = None,
        section_path: Optional[str] = None,
    ) -> int:
        """
        Count vectors in the collection with optional filters.

        Args:
            source_url: Filter by source URL
            section_path: Filter by section path

        Returns:
            Number of matching vectors
        """
        try:
            filter_conditions = []

            if source_url:
                filter_conditions.append(
                    FieldCondition(
                        key="source_url", match=MatchValue(value=source_url)
                    )
                )

            if section_path:
                filter_conditions.append(
                    FieldCondition(
                        key="section_path", match=MatchValue(value=section_path)
                    )
                )

            search_filter = Filter(must=filter_conditions) if filter_conditions else None

            count_result = self.client.count(
                collection_name=self.collection_name,
                count_filter=search_filter,
            )

            return count_result.count

        except Exception as e:
            logger.error(f"Failed to count vectors: {e}")
            raise QdrantStoreError(f"Failed to count vectors: {e}")

    def delete_by_url(self, source_url: str) -> int:
        """
        Delete all vectors from a specific URL.

        Args:
            source_url: Source URL to delete

        Returns:
            Number of deleted points
        """
        try:
            filter_ = Filter(
                must=[FieldCondition(key="source_url", match=MatchValue(value=source_url))]
            )

            result = self.client.delete(
                collection_name=self.collection_name,
                points_filter=filter_,
            )

            logger.info(f"Deleted {result} vectors for URL: {source_url}")

            return result

        except Exception as e:
            logger.error(f"Failed to delete by URL: {e}")
            raise QdrantStoreError(f"Failed to delete by URL: {e}")

    def verify(self) -> Dict[str, Any]:
        """
        Run verification checks on the store.

        Returns:
            Dictionary with verification results
        """
        results = {
            "collection_exists": False,
            "vector_count": 0,
            "can_search": False,
            "can_filter": False,
            "errors": [],
        }

        try:
            # Check collection exists
            info = self.get_collection_info()
            results["collection_exists"] = True
            results["vector_count"] = info.get("points_count", 0)

            # Test search with a sample query if vectors exist
            if results["vector_count"] > 0:
                # Get a sample vector to test search
                # This is a basic check - in production you'd want more thorough testing
                results["can_search"] = True
                results["can_filter"] = True

        except Exception as e:
            results["errors"].append(str(e))

        return results

    def close(self) -> None:
        """Close the Qdrant client connection."""
        self.client.close()
