# Data Model: RAG Pipeline

This document defines the data entities, validation rules, and relationships for the RAG embedding pipeline.

## Entities

### ContentChunk

Represents a segment of extracted text with associated metadata for vector storage.

```python
class ContentChunk:
    """A chunk of content extracted from a web page."""

    text: str                           # The textual content of the chunk
    source_url: str                     # The original URL the content was extracted from
    section_path: str                   # Hierarchical path of the section (e.g., "docs/modules/module-1")
    heading: str                        # The nearest heading above the chunk content
    chunk_index: int                    # Sequence number for ordering chunks from the same source
    char_count: int                     # Length of the chunk text for debugging
    url_hash: str                       # SHA-256 hash of source_url for deterministic ID
    content_hash: str                   # SHA-256 hash of text for idempotency

    @property
    def point_id(self) -> str:
        """Generate deterministic point ID for Qdrant upsert."""
        return hashlib.sha256(
            f"{self.url_hash}:{self.content_hash}:{self.chunk_index}".encode()
        ).hexdigest()[:32]
```

**Validation Rules**:
- `text`: Must be non-empty string, max 10,000 characters before chunking
- `source_url`: Must be valid HTTP/HTTPS URL, max 2048 characters
- `section_path`: Optional, max 512 characters, forward-slash separated
- `heading`: Optional, max 256 characters
- `chunk_index`: Must be non-negative integer
- `char_count`: Auto-calculated from `len(text)`

### VectorPoint

Represents an embedding vector stored in Qdrant with associated payload.

```python
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
import numpy as np

@dataclass
class VectorPoint:
    """A vector point stored in Qdrant for similarity search."""

    id: str                             # Unique identifier (content-based hash)
    vector: List[float]                 # The embedding vector from Cohere
    payload: Dict[str, Any]             # Metadata including all ContentChunk fields
    score: Optional[float] = None       # Similarity score (populated during search)

    # Derived from payload for convenience
    @property
    def text(self) -> str:
        return self.payload.get("text", "")

    @property
    def source_url(self) -> str:
        return self.payload.get("source_url", "")
```

**Validation Rules**:
- `id`: Must be unique string, max 64 characters
- `vector`: Must be list of floats, length matching Cohere model (1024 for embed-english-v3.0)
- `payload`: Must contain required keys: `text`, `source_url`, `section_path`, `heading`, `chunk_index`

### PipelineConfig

Configuration for the ingestion pipeline loaded from environment variables.

```python
from pydantic import BaseSettings, Field

class PipelineConfig(BaseSettings):
    """Configuration for the RAG pipeline."""

    # Required environment variables
    qdrant_url: str = Field(
        ...,
        description="Qdrant Cloud endpoint URL",
        env="QDRANT_URL"
    )
    qdrant_api_key: str = Field(
        ...,
        description="Authentication key for Qdrant",
        env="QDRANT_API_KEY"
    )
    cohere_api_key: str = Field(
        ...,
        description="API key for Cohere embedding service",
        env="COHERE_API_KEY"
    )
    source_urls: List[str] = Field(
        ...,
        description="List of URLs to ingest",
        env="SOURCE_URLS",
        env_delimiter=","
    )

    # Optional configuration with defaults
    collection_name: str = Field(
        default="book-rag-embeddings",
        description="Name of the Qdrant collection"
    )
    embedding_model: str = Field(
        default="embed-english-v3.0",
        description="Cohere model name to use"
    )
    chunk_max_tokens: int = Field(
        default=512,
        description="Maximum tokens per chunk"
    )
    chunk_overlap: int = Field(
        default=50,
        description="Overlap between chunks for context preservation"
    )
    request_timeout: float = Field(
        default=30.0,
        description="Timeout for HTTP requests in seconds"
    )
    batch_size: int = Field(
        default=96,
        description="Number of texts to embed per batch"
    )
    log_level: str = Field(
        default="INFO",
        description="Logging level (DEBUG, INFO, WARNING, ERROR)"
    )

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
```

### PipelineState

Tracks the execution state of the pipeline for logging and debugging.

```python
from dataclasses import dataclass, field
from datetime import datetime
from typing import Dict, List

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
            "errors": self.errors
        }
```

## Relationships

```
PipelineConfig
    │
    ▼
Crawler ──produces──▶ ExtractedPage ──chunks──▶ ContentChunk
                                            │
                                            ▼
                                        Embedder ──produces──▶ VectorPoint
                                                            │
                                                            ▼
                                                    QdrantStore ──upserts──▶ Qdrant Cloud
```

## Qdrant Collection Schema

```json
{
  "collection_name": "book-rag-embeddings",
  "vectors": {
    "size": 1024,
    "distance": "Cosine"
  },
  "indexes": [
    {
      "field_name": "source_url",
      "field_schema": "keyword"
    },
    {
      "field_name": "section_path",
      "field_schema": "keyword"
    },
    {
      "field_name": "heading",
      "field_schema": "text"
    }
  ]
}
```

## State Transitions

```
NOT_STARTED
    │
    ▼
FETCHING_URLS ──success──▶ EXTRACTING_TEXT ──success──▶ CHUNKING
    │                       │                       │
    │                       │                       │
    │                       ▼                       ▼
    │                    FAILED                 GENERATING_EMBEDDINGS ──success──▶ STORING
    │                       │                       │                       │
    │                       │                       │                       │
    │                       └───────────────────────┘                       ▼
    │                                       FAILED                    VERIFYING
    │                                                               │
    │                                                               ▼
    │                                                            COMPLETED
    │
    ▼
FAILED
```
