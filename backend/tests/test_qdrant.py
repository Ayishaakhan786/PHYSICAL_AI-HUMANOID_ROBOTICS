"""Tests for the qdrant_store module."""

import pytest
from unittest.mock import MagicMock, patch
from qdrant_client.models import VectorParams, Distance

from src.qdrant_store import QdrantStore, VectorPoint
from src.embedder import EmbeddedChunk
from src.chunker import ContentChunk


class TestQdrantStore:
    """Tests for the QdrantStore class."""

    @pytest.fixture
    def store(self, mock_config):
        """Create a QdrantStore instance with mocked client."""
        with patch("qdrant_client.QdrantClient") as mock_client_class:
            mock_client = MagicMock()
            mock_client_class.return_value = mock_client

            store = QdrantStore(mock_config)
            store.client = mock_client  # Replace with our mock
            yield store

    def test_ensure_collection_creates_new(self, store, mock_config):
        """Test that ensure_collection creates a new collection if it doesn't exist."""
        store.client.get_collections.return_value = MagicMock(
            collections=[]
        )

        created, message = store.ensure_collection()

        assert created is True
        assert "created" in message.lower()
        store.client.create_collection.assert_called_once()

    def test_ensure_collection_reuses_existing(self, store, mock_config):
        """Test that ensure_collection reuses an existing collection."""
        store.client.get_collections.return_value = MagicMock(
            collections=[MagicMock(name=mock_config.collection_name)]
        )
        store.client.get_collection.return_value = MagicMock(
            config=MagicMock(
                params=MagicMock(
                    vector_size=1024,
                    distance=Distance.COSINE,
                )
            )
        )

        created, message = store.ensure_collection()

        assert created is False
        assert "already exists" in message.lower()
        store.client.create_collection.assert_not_called()

    def test_ensure_collection_wrong_vector_size(self, store, mock_config):
        """Test that ensure_collection fails on wrong vector size."""
        from src.qdrant_store import CollectionError

        store.client.get_collections.return_value = MagicMock(
            collections=[MagicMock(name=mock_config.collection_name)]
        )
        store.client.get_collection.return_value = MagicMock(
            config=MagicMock(
                params=MagicMock(
                    vector_size=512,  # Wrong size
                    distance=Distance.COSINE,
                )
            )
        )

        with pytest.raises(CollectionError):
            store.ensure_collection()

    def test_upsert_embeddings(self, store, sample_chunks, sample_embeddings):
        """Test upserting embeddings to Qdrant."""
        embedded_chunks = [
            EmbeddedChunk(
                chunk=chunk,
                embedding=embedding,
                tokens_used=10,
            )
            for chunk, embedding in zip(sample_chunks, sample_embeddings)
        ]

        store.client.upsert.return_value = MagicMock()

        count, errors = store.upsert_embeddings(embedded_chunks)

        assert count == len(embedded_chunks)
        assert len(errors) == 0
        store.client.upsert.assert_called_once()

    def test_upsert_embeddings_empty(self, store):
        """Test upserting empty list."""
        count, errors = store.upsert_embeddings([])

        assert count == 0
        assert len(errors) == 0
        store.client.upsert.assert_not_called()

    def test_search(self, store, sample_embeddings):
        """Test searching for similar vectors."""
        store.client.search.return_value = [
            MagicMock(
                id="test-id",
                payload={"text": "test", "source_url": "https://example.com"},
                score=0.85,
            )
        ]

        results = store.search(query_vector=sample_embeddings[0], limit=10)

        assert len(results) == 1
        assert isinstance(results[0], VectorPoint)
        assert results[0].score == 0.85
        assert results[0].id == "test-id"

    def test_search_with_filter(self, store, sample_embeddings):
        """Test searching with URL filter."""
        store.client.search.return_value = []

        store.search(
            query_vector=sample_embeddings[0],
            limit=10,
            source_url="https://example.com",
        )

        # Verify search was called with filter
        call_args = store.client.search.call_args
        assert call_args is not None

    def test_get_collection_info(self, store, mock_config):
        """Test getting collection information."""
        store.client.get_collection.return_value = MagicMock(
            status="green",
            vectors_count=100,
            points_count=95,
            config=MagicMock(
                params=MagicMock(
                    vector_size=1024,
                    distance=Distance.COSINE,
                )
            ),
        )

        info = store.get_collection_info()

        assert info["name"] == mock_config.collection_name
        assert info["vectors_count"] == 100
        assert info["points_count"] == 95
        assert info["vector_size"] == 1024

    def test_count_vectors(self, store):
        """Test counting vectors."""
        store.client.count.return_value = MagicMock(count=50)

        count = store.count_vectors()

        assert count == 50

    def test_count_vectors_with_filter(self, store):
        """Test counting vectors with URL filter."""
        store.client.count.return_value = MagicMock(count=25)

        count = store.count_vectors(source_url="https://example.com")

        assert count == 25

    def test_verify_success(self, store):
        """Test verification succeeds when collection exists and has data."""
        store.client.get_collection.return_value = MagicMock(
            status="green",
            vectors_count=100,
            points_count=95,
            config=MagicMock(
                params=MagicMock(
                    vector_size=1024,
                    distance=Distance.COSINE,
                )
            ),
        )

        results = store.verify()

        assert results["collection_exists"] is True
        assert results["vector_count"] == 95
        assert results["can_search"] is True
        assert results["can_filter"] is True

    def test_verify_collection_not_found(self, store):
        """Test verification when collection doesn't exist."""
        from qdrant_client.http.exceptions import UnexpectedResponse

        store.client.get_collection.side_effect = UnexpectedResponse(
            message="Not found", status_code=404
        )

        results = store.verify()

        assert results["collection_exists"] is False
        assert len(results["errors"]) > 0


class TestVectorPoint:
    """Tests for the VectorPoint dataclass."""

    def test_vector_point_properties(self, mock_qdrant_points):
        """Test VectorPoint property accessors."""
        point_data = mock_qdrant_points[0]

        point = VectorPoint(
            id=point_data["id"],
            vector=point_data["vector"],
            payload=point_data["payload"],
            score=point_data["score"],
        )

        assert point.text == point_data["payload"]["text"]
        assert point.source_url == point_data["payload"]["source_url"]
        assert point.section_path == point_data["payload"]["section_path"]

    def test_vector_point_without_score(self):
        """Test VectorPoint without score."""
        point = VectorPoint(
            id="test-id",
            vector=[0.1] * 1024,
            payload={"text": "test"},
        )

        assert point.score is None

    def test_vector_point_convenience_accessors(self):
        """Test that missing payload fields return empty string."""
        point = VectorPoint(
            id="test-id",
            vector=[0.1] * 1024,
            payload={},
        )

        assert point.text == ""
        assert point.source_url == ""
        assert point.section_path == ""
