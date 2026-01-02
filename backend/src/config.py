"""Configuration for the RAG pipeline loaded from environment variables."""

import os
from pathlib import Path
from typing import List
from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict

# Forcefully reload .env file to ensure latest values are used
from dotenv import load_dotenv
env_path = Path(__file__).parent.parent / ".env"
if env_path.exists():
    load_dotenv(dotenv_path=env_path, override=True)
    # Debug: print first 4 chars of keys being loaded
    cohere_key = os.getenv("COHERE_API_KEY", "")
    qdrant_key = os.getenv("QDRANT_API_KEY", "")
    print(f"[DEBUG] COHERE_API_KEY first 4 chars: '{cohere_key[:4]}'")
    print(f"[DEBUG] QDRANT_API_KEY first 4 chars: '{qdrant_key[:4]}'")


class PipelineConfig(BaseSettings):
    """Configuration for the RAG pipeline."""

    # Required environment variables
    qdrant_url: str = Field(
        ...,
        description="Qdrant Cloud endpoint URL",
        env="QDRANT_URL",
    )
    qdrant_api_key: str = Field(
        ...,
        description="Authentication key for Qdrant",
        env="QDRANT_API_KEY",
    )
    cohere_api_key: str = Field(
        ...,
        description="API key for Cohere embedding service",
        env="COHERE_API_KEY",
    )
    source_urls: str = Field(
        default="",
        description="Comma-separated list of URLs to ingest (or use SITEMAP_URL)",
        env="SOURCE_URLS",
    )
    sitemap_url: str = Field(
        default="",
        description="URL to sitemap.xml for bulk URL ingestion",
        env="SITEMAP_URL",
    )

    # Optional configuration with defaults
    collection_name: str = Field(
        default="book-rag-embeddings",
        description="Name of the Qdrant collection",
    )
    embedding_model: str = Field(
        default="embed-english-v3.0",
        description="Cohere model name to use",
    )
    chunk_max_tokens: int = Field(
        default=512,
        description="Maximum tokens per chunk",
    )
    chunk_overlap: int = Field(
        default=50,
        description="Overlap between chunks for context preservation",
    )
    request_timeout: float = Field(
        default=30.0,
        description="Timeout for HTTP requests in seconds",
    )
    batch_size: int = Field(
        default=96,
        description="Number of texts to embed per batch",
    )
    log_level: str = Field(
        default="INFO",
        description="Logging level (DEBUG, INFO, WARNING, ERROR)",
    )

    model_config = SettingsConfigDict(
        extra="ignore",
    )

    @property
    def source_urls_list(self) -> List[str]:
        """Parse comma-separated URLs into a list."""
        if not self.source_urls:
            return []
        return [url.strip() for url in self.source_urls.split(",") if url.strip()]

    @property
    def sitemap_urls_list(self) -> List[str]:
        """Get doc URLs from sitemap if configured."""
        if not self.sitemap_url:
            return []
        # This will be populated at runtime by the crawler
        return []

    @classmethod
    def from_env(cls) -> "PipelineConfig":
        """Create configuration from environment variables."""
        return cls()

    def validate(self) -> List[str]:
        """Validate configuration and return list of errors (empty if valid)."""
        errors = []

        if not self.qdrant_url.startswith("http"):
            errors.append("QDRANT_URL must start with http:// or https://")

        if not self.qdrant_api_key:
            errors.append("QDRANT_API_KEY must not be empty")

        if not self.cohere_api_key:
            errors.append("COHERE_API_KEY must not be empty")

        if not self.source_urls:
            errors.append("SOURCE_URLS must contain at least one URL")

        for url in self.source_urls_list:
            if not url.startswith("http"):
                errors.append(f"Invalid URL in SOURCE_URLS: {url}")

        if self.chunk_max_tokens < 100:
            errors.append("CHUNK_MAX_TOKENS must be at least 100")

        if self.chunk_overlap >= self.chunk_max_tokens:
            errors.append("CHUNK_OVERLAP must be less than CHUNK_MAX_TOKENS")

        if self.batch_size < 1 or self.batch_size > 96:
            errors.append("BATCH_SIZE must be between 1 and 96")

        return errors
