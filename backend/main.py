#!/usr/bin/env python3
"""
RAG Pipeline - URL Ingestion, Embedding Generation, and Vector Storage

A pipeline that ingests content from Docusaurus book URLs, generates embeddings
using Cohere's embedding API, and stores vectors in Qdrant Cloud for downstream
retrieval.
"""

import argparse
import logging
import sys
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List, Any, Optional

from src.config import PipelineConfig
from src.crawler import Crawler, ExtractedPage
from src.chunker import Chunker, ContentChunk
from src.embedder import Embedder, EmbeddedChunk
from src.qdrant_store import QdrantStore, VectorPoint

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)


@dataclass
class PipelineState:
    """Tracks execution state for the pipeline."""

    start_time: datetime = field(default_factory=datetime.utcnow)
    end_time: Optional[datetime] = None
    urls_processed: int = 0
    urls_failed: int = 0
    chunks_created: int = 0
    vectors_stored: int = 0
    errors: List[Dict[str, Any]] = field(default_factory=list)

    @property
    def duration_seconds(self) -> float:
        """Calculate total execution time."""
        end = self.end_time or datetime.utcnow()
        return (end - self.start_time).total_seconds()

    def to_dict(self) -> Dict[str, Any]:
        """Convert state to dictionary for logging."""
        return {
            "duration_seconds": self.duration_seconds,
            "urls_processed": self.urls_processed,
            "urls_failed": self.urls_failed,
            "chunks_created": self.chunks_created,
            "vectors_stored": self.vectors_stored,
            "error_count": len(self.errors),
            "errors": self.errors[:10] if self.errors else [],  # Limit errors logged
        }


class RAGPipeline:
    """Main pipeline orchestrator."""

    def __init__(self, config: PipelineConfig):
        """Initialize the pipeline with configuration."""
        self.config = config
        self.state = PipelineState()

        # Initialize components
        self.crawler = Crawler(config)
        self.chunker = Chunker(
            max_tokens=config.chunk_max_tokens,
            overlap_tokens=config.chunk_overlap,
        )
        self.embedder = Embedder(config)
        self.qdrant_store = QdrantStore(config)

        logger.info("RAG Pipeline initialized")

    def run(self, urls: Optional[List[str]] = None) -> PipelineState:
        """
        Execute the full pipeline.

        Args:
            urls: Optional list of URLs to process (overrides config)

        Returns:
            PipelineState with execution results
        """
        urls_to_process = urls

        # If no URLs provided, check for sitemap configuration
        if not urls_to_process:
            if self.config.sitemap_url:
                logger.info(f"Fetching URLs from sitemap: {self.config.sitemap_url}")
                all_urls, sitemap_errors = self.crawler.fetch_sitemap(self.config.sitemap_url)

                if sitemap_errors:
                    self.state.errors.extend(sitemap_errors)

                # Filter to doc URLs only
                base_url = self.config.source_urls_list[0] if self.config.source_urls_list else ""
                urls_to_process = self.crawler.filter_doc_urls(all_urls, base_url)
                logger.info(f"Found {len(urls_to_process)} documentation URLs from sitemap")
            else:
                urls_to_process = self.config.source_urls_list

        logger.info(f"Starting RAG pipeline for {len(urls_to_process)} URLs")

        try:
            # Step 1: Ensure Qdrant collection exists
            created, message = self.qdrant_store.ensure_collection()
            logger.info(f"Qdrant: {message}")

            # Step 2: Fetch URLs
            pages, fetch_errors = self.crawler.fetch_all(urls_to_process)
            self.state.urls_processed = len(pages)
            self.state.urls_failed = len(fetch_errors)
            self.state.errors.extend(fetch_errors)

            if not pages:
                logger.warning("No pages fetched, stopping pipeline")
                self._finalize()
                return self.state

            logger.info(f"Fetched {len(pages)} pages successfully")

            # Step 3: Chunk content
            all_chunks = []
            for page in pages:
                chunks = self.chunker.chunk_page(page)
                all_chunks.extend(chunks)

            self.state.chunks_created = len(all_chunks)
            logger.info(f"Created {len(all_chunks)} chunks")

            if not all_chunks:
                logger.warning("No chunks created, stopping pipeline")
                self._finalize()
                return self.state

            # Step 4: Generate embeddings
            logger.info("Step 4: Starting embedding generation...")
            embedded_chunks = self.embedder.embed_chunks(all_chunks)
            logger.info(f"Step 4 COMPLETE: Generated {len(embedded_chunks)} embeddings")

            if not embedded_chunks:
                logger.warning("No embeddings generated, stopping pipeline")
                self._finalize()
                return self.state

            # Step 5: Store in Qdrant
            logger.info(f"Step 5: Starting Qdrant upsert for {len(embedded_chunks)} embeddings...")
            vectors_stored, store_errors = self.qdrant_store.upsert_embeddings(
                embedded_chunks
            )
            self.state.vectors_stored = vectors_stored
            self.state.errors.extend(store_errors)

            logger.info(f"Step 5 COMPLETE: Stored {vectors_stored} vectors in Qdrant")

        except Exception as e:
            logger.error(f"Pipeline failed: {e}")
            self.state.errors.append({
                "type": "pipeline_error",
                "error": str(e),
                "timestamp": datetime.utcnow().isoformat(),
            })

        self._finalize()
        return self.state

    def verify(self) -> Dict[str, Any]:
        """
        Verify the pipeline and stored data.

        Returns:
            Dictionary with verification results
        """
        logger.info("Running verification...")

        results = {
            "timestamp": datetime.utcnow().isoformat(),
            "config_valid": False,
            "qdrant": {},
            "sample_search": None,
            "errors": [],
        }

        # Validate configuration
        config_errors = self.config.validate()
        results["config_valid"] = len(config_errors) == 0
        if config_errors:
            results["errors"].extend(config_errors)

        # Check Qdrant
        try:
            qdrant_results = self.qdrant_store.verify()
            results["qdrant"] = qdrant_results

            if qdrant_results.get("can_search") and qdrant_results["vector_count"] > 0:
                # Run sample search
                sample_query = "test query"
                sample_embedding = self.embedder.embed_texts([sample_query])[0]

                search_results = self.qdrant_store.search(
                    query_vector=sample_embedding,
                    limit=3,
                )

                results["sample_search"] = {
                    "query": sample_query,
                    "results_count": len(search_results),
                    "top_score": search_results[0].score if search_results else None,
                }

        except Exception as e:
            results["errors"].append(f"Qdrant verification failed: {str(e)}")

        # Summary
        results["passed"] = (
            results["config_valid"]
            and results["qdrant"].get("collection_exists")
            and results["qdrant"].get("can_search", False)
        )

        return results

    def _finalize(self) -> None:
        """Finalize the pipeline run."""
        self.state.end_time = datetime.utcnow()
        duration = self.state.duration_seconds

        logger.info(
            f"Pipeline complete in {duration:.1f}s: "
            f"{self.state.urls_processed} URLs, "
            f"{self.state.chunks_created} chunks, "
            f"{self.state.vectors_stored} vectors, "
            f"{len(self.state.errors)} errors"
        )

    def close(self) -> None:
        """Clean up resources."""
        self.chunker.close()
        self.qdrant_store.close()


def main() -> int:
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="RAG Pipeline - URL ingestion, embedding generation, and vector storage"
    )
    parser.add_argument(
        "--urls",
        nargs="+",
        help="URLs to process (overrides SOURCE_URLS env var)",
    )
    parser.add_argument(
        "--verify",
        action="store_true",
        help="Run verification instead of full pipeline",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level",
    )

    args = parser.parse_args()

    # Set log level
    logging.getLogger().setLevel(getattr(logging, args.log_level))

    try:
        # Load configuration
        config = PipelineConfig.from_env()

        # Validate configuration
        errors = config.validate()
        if errors:
            for error in errors:
                logger.error(f"Configuration error: {error}")
            return 1

        # Create pipeline
        pipeline = RAGPipeline(config)

        try:
            if args.verify:
                # Run verification
                results = pipeline.verify()

                print("\n=== Verification Results ===")
                print(f"Config valid: {results['config_valid']}")
                print(f"Qdrant collection: {results['qdrant'].get('collection_exists', False)}")
                print(f"Vector count: {results['qdrant'].get('vector_count', 0)}")
                print(f"Can search: {results['qdrant'].get('can_search', False)}")

                if results["sample_search"]:
                    print(f"\nSample search: '{results['sample_search']['query']}'")
                    print(f"Results: {results['sample_search']['results_count']}")
                    print(f"Top score: {results['sample_search']['top_score']:.4f}")

                if results["errors"]:
                    print(f"\nErrors: {len(results['errors'])}")
                    for error in results["errors"][:5]:
                        print(f"  - {error}")

                print(f"\n{'PASS' if results['passed'] else 'FAIL'}")

                return 0 if results["passed"] else 1

            else:
                # Run pipeline
                state = pipeline.run(args.urls)

                print("\n=== Pipeline Results ===")
                print(f"Duration: {state.duration_seconds:.1f}s")
                print(f"URLs processed: {state.urls_processed}/{state.urls_processed + state.urls_failed}")
                print(f"Chunks created: {state.chunks_created}")
                print(f"Vectors stored: {state.vectors_stored}")

                if state.errors:
                    print(f"Errors: {len(state.errors)}")
                    for error in state.errors[:5]:
                        print(f"  - {error.get('url', 'unknown')}: {error.get('error', error.get('type', 'unknown'))}")

                return 0 if state.vectors_stored > 0 else 1

        finally:
            pipeline.close()

    except KeyboardInterrupt:
        logger.info("Pipeline interrupted by user")
        return 130

    except Exception as e:
        logger.error(f"Pipeline failed: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
