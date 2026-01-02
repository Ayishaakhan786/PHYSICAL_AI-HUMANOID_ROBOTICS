"""Pytest configuration and fixtures for the RAG pipeline tests."""

import pytest
from unittest.mock import MagicMock, patch
from typing import List, Dict, Any

from src.config import PipelineConfig
from src.crawler import ExtractedPage
from src.chunker import ContentChunk
from src.embedder import EmbeddedChunk


@pytest.fixture
def mock_config():
    """Create a mock configuration for testing."""
    config = MagicMock(spec=PipelineConfig)
    config.qdrant_url = "https://test.qdrant.cloud:6333"
    config.qdrant_api_key = "test-api-key"
    config.cohere_api_key = "test-cohere-key"
    config.source_urls = ["https://example.com/docs"]
    config.collection_name = "test-rag-embeddings"
    config.embedding_model = "embed-english-v3.0"
    config.chunk_max_tokens = 512
    config.chunk_overlap = 50
    config.request_timeout = 30.0
    config.batch_size = 96
    config.log_level = "DEBUG"
    return config


@pytest.fixture
def sample_page():
    """Create a sample extracted page for testing."""
    return ExtractedPage(
        url="https://example.com/docs/intro",
        title="Introduction",
        text="""
# Introduction

Welcome to the documentation. This is the introduction section.

## Getting Started

To get started, follow these steps:
1. Install the package
2. Configure your environment
3. Run the pipeline

## Architecture

The system consists of multiple components working together.

### Components

- Crawler: Fetches web pages
- Chunker: Splits content into chunks
- Embedder: Generates embeddings
- Store: Stores vectors in Qdrant

## Conclusion

This concludes the introduction.
""".strip(),
        html="<html><body><article>...</article></body></html>",
        section_path="docs/intro",
        headings=["Introduction", "Getting Started", "Architecture", "Components", "Conclusion"],
        links=["https://example.com/docs/getting-started", "https://example.com/docs/architecture"],
    )


@pytest.fixture
def sample_chunks():
    """Create sample content chunks for testing."""
    return [
        ContentChunk(
            text="Welcome to the documentation. This is the introduction section.",
            source_url="https://example.com/docs/intro",
            section_path="docs/intro",
            heading="Introduction",
            chunk_index=0,
        ),
        ContentChunk(
            text="To get started, follow these steps: 1. Install the package 2. Configure your environment 3. Run the pipeline",
            source_url="https://example.com/docs/intro",
            section_path="docs/intro",
            heading="Getting Started",
            chunk_index=1,
        ),
        ContentChunk(
            text="The system consists of multiple components working together. Crawler: Fetches web pages, Chunker: Splits content into chunks, Embedder: Generates embeddings, Store: Stores vectors in Qdrant",
            source_url="https://example.com/docs/intro",
            section_path="docs/intro",
            heading="Components",
            chunk_index=2,
        ),
    ]


@pytest.fixture
def sample_embeddings():
    """Create sample embedding vectors for testing."""
    import random
    random.seed(42)  # For reproducibility

    # Create 3 embeddings of dimension 1024
    return [[random.uniform(-1, 1) for _ in range(1024)] for _ in range(3)]


@pytest.fixture
def mock_cohere_response():
    """Create a mock Cohere embed response."""
    import random
    random.seed(42)

    return {
        "embeddings": [[random.uniform(-1, 1) for _ in range(1024)] for _ in range(3)],
        "meta": {
            "api_version": "2021-12-01",
            "billed_units": {"input_tokens": 150},
        },
    }


@pytest.fixture
def mock_qdrant_points():
    """Create mock Qdrant points for testing."""
    return [
        {
            "id": "abc123",
            "vector": [0.1] * 1024,
            "payload": {
                "text": "Sample text",
                "source_url": "https://example.com/docs/intro",
                "section_path": "docs/intro",
                "heading": "Introduction",
                "chunk_index": 0,
                "char_count": 100,
                "url_hash": "abc123",
                "content_hash": "def456",
            },
            "score": 0.85,
        },
    ]
