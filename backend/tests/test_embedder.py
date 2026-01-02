"""Tests for the embedder module."""

import pytest
from unittest.mock import MagicMock, patch
import cohere

from src.embedder import Embedder, EmbeddedChunk
from src.chunker import ContentChunk


class TestEmbedder:
    """Tests for the Embedder class."""

    @pytest.fixture
    def embedder(self, mock_config):
        """Create an embedder instance with mocked config."""
        with patch.object(cohere, "Client") as mock_client:
            mock_client.return_value = MagicMock()
            embedder = Embedder(mock_config)
            yield embedder
            # Clean up

    def test_embedder_initialization(self, mock_config):
        """Test embedder initializes with correct parameters."""
        with patch.object(cohere, "Client") as mock_client:
            mock_client.return_value = MagicMock()
            embedder = Embedder(mock_config)

            assert embedder.model == mock_config.embedding_model
            assert embedder.batch_size == mock_config.batch_size

    def test_get_embedding_dimension(self, mock_config):
        """Test getting embedding dimension for different models."""
        with patch.object(cohere, "Client") as mock_client:
            mock_client.return_value = MagicMock()

            # Test English v3.0
            mock_config.embedding_model = "embed-english-v3.0"
            embedder = Embedder(mock_config)
            assert embedder.get_embedding_dimension() == 1024

            # Test multilingual v3.0
            mock_config.embedding_model = "embed-multilingual-v3.0"
            embedder = Embedder(mock_config)
            assert embedder.get_embedding_dimension() == 1024

            # Test light version
            mock_config.embedding_model = "embed-english-light-v3.0"
            embedder = Embedder(mock_config)
            assert embedder.get_embedding_dimension() == 384

    def test_validate_embedding_valid(self, embedder):
        """Test validation of a valid embedding."""
        embedding = [0.1] * 1024
        assert embedder.validate_embedding(embedding) is True

    def test_validate_embedding_wrong_dimension(self, embedder):
        """Test validation fails for wrong dimension."""
        embedding = [0.1] * 512  # Wrong size
        assert embedder.validate_embedding(embedding) is False

    def test_validate_embedding_empty(self, embedder):
        """Test validation fails for empty embedding."""
        assert embedder.validate_embedding([]) is False

    def test_validate_embedding_with_nan(self, embedder):
        """Test validation fails for embedding with NaN."""
        import math
        embedding = [0.1] * 1024
        embedding[0] = float("nan")
        assert embedder.validate_embedding(embedding) is False

    def test_compute_similarity(self, embedder):
        """Test cosine similarity computation."""
        # Identical vectors
        v1 = [0.5] * 1024
        v2 = [0.5] * 1024
        similarity = embedder.compute_similarity(v1, v2)
        assert similarity == pytest.approx(1.0, abs=0.01)

        # Opposite vectors
        v1 = [1.0] * 1024
        v2 = [-1.0] * 1024
        similarity = embedder.compute_similarity(v1, v2)
        assert similarity == pytest.approx(-1.0, abs=0.01)

        # Orthogonal vectors
        import random
        random.seed(42)
        v1 = [random.uniform(-1, 1) for _ in range(1024)]
        v2 = [random.uniform(-1, 1) for _ in range(1024)]
        similarity = embedder.compute_similarity(v1, v2)
        assert -1.0 <= similarity <= 1.0

    def test_compute_similarity_empty(self, embedder):
        """Test similarity with empty vectors."""
        assert embedder.compute_similarity([], [1.0] * 1024) == 0.0
        assert embedder.compute_similarity([1.0] * 1024, []) == 0.0
        assert embedder.compute_similarity([], []) == 0.0

    def test_embed_chunks_empty(self, embedder):
        """Test embedding empty list."""
        result = embedder.embed_chunks([])
        assert result == []

    @patch.object(cohere.Client, "embed")
    def test_embed_chunks_single(self, mock_embed, embedder, sample_chunks, mock_cohere_response):
        """Test embedding a single chunk."""
        mock_embed.return_value = MagicMock(
            embeddings=[mock_cohere_response["embeddings"][0]],
            meta=mock_cohere_response["meta"],
        )

        result = embedder.embed_chunks(sample_chunks[:1])

        assert len(result) == 1
        assert isinstance(result[0], EmbeddedChunk)
        assert result[0].chunk == sample_chunks[0]
        assert len(result[0].embedding) == 1024

    @patch.object(cohere.Client, "embed")
    def test_embed_chunks_batch(self, mock_embed, embedder, sample_chunks, mock_cohere_response):
        """Test embedding multiple chunks in batch."""
        mock_embed.return_value = MagicMock(
            embeddings=mock_cohere_response["embeddings"],
            meta=mock_cohere_response["meta"],
        )

        result = embedder.embed_chunks(sample_chunks)

        assert len(result) == len(sample_chunks)

    @patch.object(cohere.Client, "embed")
    def test_embed_chunks_retries_on_error(self, mock_embed, embedder, sample_chunks):
        """Test that embedding retries on transient errors."""
        from cohere.core.api_error import ApiError

        # First two calls fail, third succeeds
        mock_embed.side_effect = [
            ApiError(message="Server error", status_code=500),
            ApiError(message="Server error", status_code=500),
            MagicMock(
                embeddings=[mock_chunks["embedding"] for mock_chunks in [sample_chunks[0].text] * 3],
                meta={"billed_units": {"input_tokens": 100}},
            ),
        ]

        # Should succeed after retries
        with patch("time.sleep"):  # Skip actual sleep
            result = embedder.embed_chunks(sample_chunks[:1])

        assert len(result) == 1
        assert mock_embed.call_count == 3


class TestEmbeddedChunk:
    """Tests for the EmbeddedChunk dataclass."""

    def test_embedded_chunk_creation(self, sample_chunks, sample_embeddings):
        """Test creating an embedded chunk."""
        chunk = EmbeddedChunk(
            chunk=sample_chunks[0],
            embedding=sample_embeddings[0],
            tokens_used=10,
        )

        assert chunk.chunk == sample_chunks[0]
        assert chunk.embedding == sample_embeddings[0]
        assert chunk.tokens_used == 10
