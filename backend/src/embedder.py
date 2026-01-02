"""Embedding generation using Cohere API for the RAG pipeline."""

import logging
import time
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional

import cohere
from cohere.core.api_error import ApiError

from .config import PipelineConfig
from .chunker import ContentChunk

logger = logging.getLogger(__name__)


@dataclass
class EmbeddedChunk:
    """A chunk with its generated embedding vector."""

    chunk: ContentChunk
    embedding: List[float]
    tokens_used: int = 0


class EmbeddingError(Exception):
    """Base exception for embedding errors."""

    pass


class APIError(EmbeddingError):
    """Cohere API request failed."""

    def __init__(self, message: str, status_code: Optional[int] = None):
        self.message = message
        self.status_code = status_code
        super().__init__(message)


class RateLimitError(EmbeddingError):
    """API rate limit exceeded."""

    def __init__(self, message: str = "Rate limit exceeded"):
        super().__init__(message)


class Embedder:
    """Generates embeddings using Cohere's embedding API."""

    def __init__(self, config: PipelineConfig):
        """
        Initialize the embedder with configuration.

        Args:
            config: Pipeline configuration with Cohere API settings
        """
        self.config = config
        self.client = cohere.Client(api_key=config.cohere_api_key)
        self.model = config.embedding_model
        self.batch_size = config.batch_size

        logger.info(
            f"Initialized embedder with model={self.model}, "
            f"batch_size={self.batch_size}"
        )

    def embed_chunks(self, chunks: List[ContentChunk]) -> List[EmbeddedChunk]:
        """
        Generate embeddings for a list of content chunks.

        Args:
            chunks: List of ContentChunk objects to embed

        Returns:
            List of EmbeddedChunk objects with embeddings
        """
        if not chunks:
            return []

        logger.info(f"Embedding {len(chunks)} chunks in batches of {self.batch_size}")

        embedded_chunks = []
        texts = [chunk.text for chunk in chunks]

        # Process in batches
        for i in range(0, len(texts), self.batch_size):
            batch_texts = texts[i : i + self.batch_size]
            batch_chunks = chunks[i : i + self.batch_size]

            batch_embeddings = self._embed_batch(batch_texts)

            for chunk, embedding in zip(batch_chunks, batch_embeddings):
                embedded = EmbeddedChunk(
                    chunk=chunk,
                    embedding=embedding,
                    tokens_used=len(self._encode(chunk.text)),
                )
                embedded_chunks.append(embedded)

            logger.info(
                f"Embedded batch {i // self.batch_size + 1}/"
                f"{(len(texts) + self.batch_size - 1) // self.batch_size}"
            )

            # Sleep 12 seconds between batches to respect Free Tier rate limit (5 calls/minute)
            if i + self.batch_size < len(texts):
                logger.info("Sleeping 12 seconds between batches to respect API rate limit")
                time.sleep(12)

        logger.info(f"Generated {len(embedded_chunks)} embeddings")

        return embedded_chunks

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of text strings.

        Args:
            texts: List of text strings to embed

        Returns:
            List of embedding vectors
        """
        if not texts:
            return []

        logger.info(f"Embedding {len(texts)} texts")

        all_embeddings = []

        for i in range(0, len(texts), self.batch_size):
            batch = texts[i : i + self.batch_size]
            embeddings = self._embed_batch(batch)
            all_embeddings.extend(embeddings)

        return all_embeddings

    def _embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Embed a batch of texts with retry logic.

        Args:
            texts: List of text strings to embed

        Returns:
            List of embedding vectors

        Raises:
            APIError: If the API request fails after retries
            RateLimitError: If rate limit is hit and cannot be recovered
        """
        max_retries = 5
        base_delay = 1.0  # seconds

        for attempt in range(max_retries):
            try:
                # Debug: print first 4 chars of API key being used
                print(f"[DEBUG] Cohere API key first 4 chars: '{self.config.cohere_api_key[:4]}'")
                print(f"[DEBUG] Making API call to Cohere embed with {len(texts)} texts...")

                response = self.client.embed(
                    texts=texts,
                    model=self.model,
                    input_type="search_document",  # Required for embed-english-v3.0
                    truncate="END",
                )

                # Check for API errors in response
                if hasattr(response, "meta") and hasattr(response.meta, "warnings") and response.meta.warnings:
                    for warning in response.meta.warnings:
                        logger.warning(f"Cohere API warning: {warning}")

                return response.embeddings

            except ApiError as e:
                status_code = getattr(e, "status_code", None)

                # Check for rate limit (429)
                if status_code == 429:
                    if attempt < max_retries - 1:
                        # Exponential backoff
                        delay = base_delay * (2 ** attempt)
                        # Add jitter
                        delay *= 0.5 + hash(str(time.time())) % 100 / 100.0
                        logger.warning(
                            f"Rate limited, retrying in {delay:.1f}s "
                            f"(attempt {attempt + 1}/{max_retries})"
                        )
                        time.sleep(delay)
                        continue
                    else:
                        raise RateLimitError(
                            "Rate limit exceeded after maximum retries"
                        )

                # Check for other client errors (4xx)
                if status_code and status_code < 500:
                    # Extract message from BadRequestError
                    error_msg = str(e)
                    raise APIError(
                        f"Cohere API error: {error_msg}",
                        status_code=status_code,
                    )

                # Server errors (5xx) - retry
                if attempt < max_retries - 1:
                    delay = base_delay * (2 ** attempt)
                    logger.warning(
                        f"Server error {status_code}, retrying in {delay:.1f}s"
                    )
                    time.sleep(delay)
                    continue

                raise APIError(f"Cohere API error: {e}", status_code=status_code)

            except Exception as e:
                if attempt < max_retries - 1:
                    delay = base_delay * (2 ** attempt)
                    logger.warning(
                        f"Embedding failed: {e}, retrying in {delay:.1f}s"
                    )
                    time.sleep(delay)
                    continue
                raise EmbeddingError(f"Embedding failed after retries: {e}")

        # Should not reach here
        raise APIError("Unexpected error in embed batch")

    def _encode(self, text: str) -> List[int]:
        """Encode text to tokens using the model's tokenizer."""
        # Use tiktoken for token counting (same as chunker)
        import tiktoken
        encoding = tiktoken.get_encoding("cl100k_base")
        try:
            return encoding.encode(text)
        finally:
            pass  # tiktoken encodings don't need to be closed

    def get_embedding_dimension(self) -> int:
        """
        Get the embedding dimension for the configured model.

        Returns:
            Dimension of embedding vectors
        """
        # Common Cohere embed models dimensions:
        # embed-english-v3.0: 1024
        # embed-multilingual-v3.0: 1024
        # embed-english-light-v3.0: 384
        # embed-multilingual-light-v3.0: 384
        # embed-english-v2.0: 4096
        # embed-multilingual-v2.0: 768

        model_dims = {
            "embed-english-v3.0": 1024,
            "embed-multilingual-v3.0": 1024,
            "embed-english-light-v3.0": 384,
            "embed-multilingual-light-v3.0": 384,
            "embed-english-v2.0": 4096,
            "embed-multilingual-v2.0": 768,
        }

        return model_dims.get(self.model, 1024)

    def validate_embedding(self, embedding: List[float]) -> bool:
        """
        Validate an embedding vector.

        Args:
            embedding: The embedding vector to validate

        Returns:
            True if valid, False otherwise
        """
        expected_dim = self.get_embedding_dimension()

        if not embedding:
            return False

        if len(embedding) != expected_dim:
            logger.warning(
                f"Embedding dimension mismatch: expected {expected_dim}, "
                f"got {len(embedding)}"
            )
            return False

        # Check for NaN or Inf values
        for i, val in enumerate(embedding):
            if not (isinstance(val, (int, float)) and float("-inf") < val < float("inf")):
                logger.warning(f"Invalid value in embedding at index {i}")
                return False

        return True

    def compute_similarity(
        self, embedding1: List[float], embedding2: List[float]
    ) -> float:
        """
        Compute cosine similarity between two embeddings.

        Args:
            embedding1: First embedding vector
            embedding2: Second embedding vector

        Returns:
            Cosine similarity score between -1 and 1
        """
        import math

        if not embedding1 or not embedding2:
            return 0.0

        dot_product = sum(a * b for a, b in zip(embedding1, embedding2))
        norm1 = math.sqrt(sum(a * a for a in embedding1))
        norm2 = math.sqrt(sum(b * b for b in embedding2))

        if norm1 == 0 or norm2 == 0:
            return 0.0

        return dot_product / (norm1 * norm2)
