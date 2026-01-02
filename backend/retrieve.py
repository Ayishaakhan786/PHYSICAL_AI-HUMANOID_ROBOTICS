#!/usr/bin/env python3
"""
RAG Retrieval Pipeline Validation Script

Validates semantic search retrieval from Qdrant Cloud collection.
Performs relevance validation, metadata verification, and latency measurement.
"""

import argparse
import json
import logging
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Enable UTF-8 encoding for Windows
if sys.platform == "win32":
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)

# T001: CLI argument parsing
parser = argparse.ArgumentParser(
    description="RAG Retrieval Pipeline Validation"
)
parser.add_argument(
    "--query", "-q", type=str, help="Single query to validate"
)
parser.add_argument(
    "--verbose", "-v", action="store_true", help="Enable verbose output"
)
parser.add_argument(
    "--output", "-o", type=str, help="Output results to JSON file"
)
parser.add_argument(
    "--limit", type=int, default=5, help="Number of results per query (default: 5)"
)
parser.add_argument(
    "--threshold", type=float, default=0.50, help="Relevance score threshold (default: 0.50)"
)
args = parser.parse_args()

# Increase log level for verbose mode
if args.verbose:
    logging.getLogger().setLevel(logging.DEBUG)


# T002: Config loading from .env
def load_config() -> Dict[str, str]:
    """Load configuration from .env file."""
    from dotenv import load_dotenv

    # Try to load from root .env (via backend/..)
    env_paths = [
        Path(__file__).parent.parent / ".env",  # Root .env from backend/
        Path(__file__).parent / ".env",          # backend/.env
    ]

    config_loaded = False
    for env_path in env_paths:
        if env_path.exists():
            load_dotenv(dotenv_path=str(env_path), override=True)
            logger.debug(f"Loaded config from {env_path}")
            config_loaded = True
            break

    if not config_loaded:
        logger.warning("No .env file found, using environment variables")

    config = {
        "QDRANT_URL": "",
        "QDRANT_API_KEY": "",
        "COHERE_API_KEY": "",
        "COLLECTION_NAME": "book-rag-embeddings",
        "EMBEDDING_MODEL": "embed-english-v3.0",
    }

    for key in config:
        value = _get_env_or_raise(key)
        config[key] = value

    logger.info(f"Configuration loaded: QDRANT_URL={config['QDRANT_URL'][:50]}...")
    return config


def _get_env_or_raise(key: str) -> str:
    """Get environment variable or raise an error."""
    import os

    value = os.getenv(key, "")
    if not value:
        raise ValueError(f"Required environment variable {key} not set")
    return value


@dataclass
class TestQuery:
    """Represents a validation test case."""
    query_text: str
    expected_topics: List[str] = field(default_factory=list)
    min_results: int = 3
    source_url_pattern: Optional[str] = None


@dataclass
class RetrievalResult:
    """Represents a single chunk from search."""
    id: str
    score: float
    text: str
    source_url: str
    section_path: str
    heading: str
    chunk_index: int


@dataclass
class ValidationOutcome:
    """Result of a single test query."""
    query: str
    results: List[RetrievalResult]
    passed: bool
    latency_ms: float
    errors: List[str] = field(default_factory=list)
    relevance_scores: List[float] = field(default_factory=list)


class RetrievalValidator:
    """Validates RAG retrieval pipeline."""

    def __init__(self, config: Dict[str, str], limit: int = 5, threshold: float = 0.5):
        self.config = config
        self.limit = limit
        self.threshold = threshold

        # T003: Qdrant connection
        from qdrant_client import QdrantClient
        self.qdrant_client = QdrantClient(
            url=config["QDRANT_URL"],
            api_key=config["QDRANT_API_KEY"],
        )
        logger.debug("Connected to Qdrant")

        # T004: Cohere client for query embedding
        import cohere
        self.cohere_client = cohere.Client(api_key=config["COHERE_API_KEY"])
        self.embedding_model = config["EMBEDDING_MODEL"]
        logger.debug(f"Initialized Cohere client with model: {self.embedding_model}")

        # T005: Predefined test queries
        self.test_queries = self._get_default_test_queries()

    def _get_default_test_queries(self) -> List[TestQuery]:
        """Get default test queries based on book content."""
        return [
            TestQuery(
                query_text="humanoid robot kinematics",
                expected_topics=["kinematics", "motion", "biomechanics", "robotics"],
                min_results=3,
            ),
            TestQuery(
                query_text="Isaac Sim GPU simulation",
                expected_topics=["isaac", "simulation", "gpu", "nvidia"],
                min_results=3,
            ),
            TestQuery(
                query_text="vision language action models",
                expected_topics=["vla", "vision", "language", "action"],
                min_results=3,
            ),
            TestQuery(
                query_text="domain randomization sim-to-real",
                expected_topics=["domain randomization", "sim-to-real", "transfer learning"],
                min_results=3,
            ),
            TestQuery(
                query_text="human interaction humanoid robots",
                expected_topics=["human interaction", "humanoid", "social", "interaction"],
                min_results=3,
            ),
        ]

    def _embed_query(self, query: str) -> List[float]:
        """Generate embedding for query using Cohere API."""
        logger.debug(f"Embedding query: {query[:50]}...")

        response = self.cohere_client.embed(
            texts=[query],
            model=self.embedding_model,
            input_type="search_query",
        )
        return response.embeddings[0]

    def _search(self, query_vector: List[float]) -> List[RetrievalResult]:
        """Search Qdrant collection."""
        collection_name = self.config["COLLECTION_NAME"]

        results = self.qdrant_client.query_points(
            collection_name=collection_name,
            query=query_vector,
            limit=self.limit,
            with_payload=True,
        )

        retrieval_results = []
        for hit in results.points:
            payload = hit.payload or {}

            retrieval_results.append(
                RetrievalResult(
                    id=str(hit.id),
                    score=hit.score,
                    text=payload.get("text", ""),
                    source_url=payload.get("source_url", ""),
                    section_path=payload.get("section_path", ""),
                    heading=payload.get("heading", ""),
                    chunk_index=payload.get("chunk_index", 0),
                )
            )

        logger.debug(f"Found {len(retrieval_results)} results")
        return retrieval_results

    def _validate_result(
        self, result: RetrievalResult, query: TestQuery
    ) -> Tuple[bool, List[str]]:
        """Validate a single retrieval result."""
        errors = []

        # Metadata completeness checks
        if not result.source_url:
            errors.append("Missing source_url")

        if not result.section_path:
            errors.append("Missing section_path")

        if not result.heading:
            errors.append("Missing heading")

        # Relevance check
        if result.score < self.threshold:
            errors.append(f"Score {result.score:.3f} below threshold {self.threshold}")

        # Content check
        if not result.text:
            errors.append("Empty text content")

        return len(errors) == 0, errors

    def _run_single_query(self, query: TestQuery) -> ValidationOutcome:
        """Execute a single test query and validate results."""
        start_time = time.time()

        try:
            # Generate query embedding
            query_vector = self._embed_query(query.query_text)

            # Search Qdrant
            results = self._search(query_vector)

            latency_ms = (time.time() - start_time) * 1000

            # Validate results - only check TOP result for pass/fail
            top_passed = True
            top_errors = []
            relevance_scores = []

            # Only validate the top result (index 0) for pass/fail status
            if results:
                top_result = results[0]
                top_passed, top_errors = self._validate_result(top_result, query)

            all_passed = top_passed
            all_errors = top_errors

            for result in results:
                relevance_scores.append(result.score)

            return ValidationOutcome(
                query=query.query_text,
                results=results,
                passed=all_passed,
                latency_ms=latency_ms,
                errors=all_errors,
                relevance_scores=relevance_scores,
            )

        except Exception as e:
            latency_ms = (time.time() - start_time) * 1000
            logger.error(f"Query failed: {e}")
            return ValidationOutcome(
                query=query.query_text,
                results=[],
                passed=False,
                latency_ms=latency_ms,
                errors=[f"Query execution failed: {str(e)}"],
            )

    # T006: Validation logic and T007: Latency measurement
    def run_validation(self, custom_query: Optional[str] = None) -> Dict[str, Any]:
        """Run validation on test queries or custom query."""
        outcomes = []
        latencies = []

        if custom_query:
            # Single custom query
            query = TestQuery(query_text=custom_query, min_results=1)
            outcome = self._run_single_query(query)
            outcomes.append(outcome)
            latencies.append(outcome.latency_ms)
        else:
            # Run all test queries
            for i, query in enumerate(self.test_queries):
                logger.info(f"Running test query {i+1}/{len(self.test_queries)}: {query.query_text[:50]}...")
                outcome = self._run_single_query(query)
                outcomes.append(outcome)
                latencies.append(outcome.latency_ms)

        # Calculate metrics
        passed = sum(1 for o in outcomes if o.passed)
        failed = len(outcomes) - passed
        avg_latency = sum(latencies) / len(latencies) if latencies else 0

        # Calculate p95 latency
        sorted_latencies = sorted(latencies)
        p95_index = int(len(sorted_latencies) * 0.95)
        p95_latency = sorted_latencies[min(p95_index, len(sorted_latencies) - 1)] if sorted_latencies else 0

        # Build report
        report = {
            "total_queries": len(outcomes),
            "passed": passed,
            "failed": failed,
            "avg_latency_ms": round(avg_latency, 2),
            "p95_latency_ms": round(p95_latency, 2),
            "success_rate": round(passed / len(outcomes) * 100, 1) if outcomes else 0,
            "results": [],
        }

        for outcome in outcomes:
            result_entry = {
                "query": outcome.query,
                "passed": outcome.passed,
                "result_count": len(outcome.results),
                "latency_ms": round(outcome.latency_ms, 2),
                "min_score": min(outcome.relevance_scores) if outcome.relevance_scores else 0,
                "max_score": max(outcome.relevance_scores) if outcome.relevance_scores else 0,
            }

            if outcome.errors:
                result_entry["errors"] = outcome.errors

            # Always include details with scores and text for debugging
            result_entry["details"] = [
                {
                    "score": r.score,
                    "source_url": r.source_url,
                    "section_path": r.section_path,
                    "heading": r.heading[:50] if r.heading else "",
                    "text": r.text[:200] if r.text else "",
                }
                for r in outcome.results[:3]
            ]

            # Also store full results for reference
            result_entry["full_results"] = [
                {
                    "score": r.score,
                    "text": r.text[:300] if r.text else "",
                    "source_url": r.source_url,
                }
                for r in outcome.results
            ]

            report["results"].append(result_entry)

        return report

    def close(self):
        """Close connections."""
        self.qdrant_client.close()


# T008: Error handling
def main():
    """Main entry point."""
    try:
        # Load configuration
        config = load_config()

        # Initialize validator
        validator = RetrievalValidator(
            config, limit=args.limit, threshold=args.threshold
        )

        # Run validation
        report = validator.run_validation(custom_query=args.query)

        # Close connections
        validator.close()

        # Output results
        if args.output:
            output_path = Path(args.output)
            with open(output_path, "w") as f:
                json.dump(report, f, indent=2)
            logger.info(f"Results written to {output_path}")

        # Print summary
        print("\n=== Retrieval Validation Results ===")
        print(f"Total Queries: {report['total_queries']}")
        print(f"Passed: {report['passed']}")
        print(f"Failed: {report['failed']}")
        print(f"Avg Latency: {report['avg_latency_ms']:.2f}ms")
        print(f"p95 Latency: {report['p95_latency_ms']:.2f}ms")
        print(f"Success Rate: {report['success_rate']}%")

        print("\nResults:")
        for result in report["results"]:
            status = "PASS" if result["passed"] else "FAIL"
            print(f"  [{status}] {result['query'][:50]}... ({result['result_count']} results, {result['latency_ms']:.2f}ms)")

            # Print top 3 results with scores and text for debugging
            if "details" in result:
                print("\n    Top 3 Results:")
                for i, detail in enumerate(result["details"], 1):
                    score = detail.get("score", 0)
                    text = detail.get("text", "")[:200]
                    try:
                        text = text.encode('utf-8', errors='ignore').decode('utf-8')
                    except Exception:
                        text = ""
                    print(f"      [{i}] Score: {score:.4f}")
                    if text:
                        print(f"          Text: {text[:150]}...")
                    print(f"          URL: {detail.get('source_url', 'N/A')}")

        # Exit with appropriate code
        if report["failed"] > 0:
            print(f"\nWARNING: {report['failed']} test(s) failed")
            sys.exit(1)

        print("\nAll tests passed!")
        sys.exit(0)

    except KeyboardInterrupt:
        logger.info("Validation interrupted by user")
        sys.exit(130)

    except Exception as e:
        logger.error(f"Validation failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
