"""Tests for the chunker module."""

import pytest
from src.chunker import Chunker, ContentChunk


class TestChunker:
    """Tests for the Chunker class."""

    def test_chunker_initialization(self):
        """Test chunker initializes with correct parameters."""
        chunker = Chunker(max_tokens=256, overlap_tokens=25)
        assert chunker.max_tokens == 256
        assert chunker.overlap_tokens == 25

    def test_count_tokens(self):
        """Test token counting."""
        chunker = Chunker()
        text = "Hello, world! This is a test."
        count = chunker.count_tokens(text)
        assert count > 0
        assert count == len(chunker.encoding.encode(text))

    def test_chunk_single_paragraph(self, sample_page):
        """Test chunking a simple page with one paragraph."""
        chunker = Chunker(max_tokens=512, overlap_tokens=50)

        chunks = chunker.chunk_page(sample_page)

        assert len(chunks) >= 1
        assert all(isinstance(c, ContentChunk) for c in chunks)

        # Verify chunk properties
        for chunk in chunks:
            assert chunk.source_url == sample_page.url
            assert chunk.section_path == sample_page.section_path
            assert chunk.char_count == len(chunk.text)
            assert chunk.chunk_index >= 0

    def test_chunk_metadata_complete(self, sample_page):
        """Test that chunks have complete metadata."""
        chunker = Chunker(max_tokens=512, overlap_tokens=50)

        chunks = chunker.chunk_page(sample_page)

        if chunks:
            chunk = chunks[0]
            assert chunk.text is not None
            assert chunk.text != ""
            assert chunk.source_url is not None
            assert chunk.section_path is not None
            # heading may be empty if not found
            assert isinstance(chunk.chunk_index, int)
            assert chunk.chunk_index >= 0

    def test_chunk_idempotent(self, sample_page):
        """Test that chunking produces deterministic results."""
        chunker = Chunker(max_tokens=512, overlap_tokens=50)

        chunks1 = chunker.chunk_page(sample_page)
        chunks2 = chunker.chunk_page(sample_page)

        assert len(chunks1) == len(chunks2)

        for c1, c2 in zip(chunks1, chunks2):
            assert c1.text == c2.text
            assert c1.point_id == c2.point_id

    def test_chunk_text_method(self):
        """Test the chunk_text method with raw text."""
        chunker = Chunker(max_tokens=256, overlap_tokens=25)

        text = "This is a test. " * 50  # Create enough text for multiple chunks
        chunks = chunker.chunk_text(
            text=text,
            source_url="https://example.com/test",
            section_path="test",
            heading="Test Heading",
        )

        assert len(chunks) > 1

        # Verify all chunks have required metadata
        for chunk in chunks:
            assert chunk.source_url == "https://example.com/test"
            assert chunk.section_path == "test"
            assert chunk.heading == "Test Heading"
            assert chunk.chunk_index >= 0

    def test_chunk_token_limit(self):
        """Test that chunks respect token limit."""
        chunker = Chunker(max_tokens=100, overlap_tokens=10)

        # Create text with many tokens
        text = "word " * 200  # ~200 tokens
        chunks = chunker.chunk_text(text, "https://example.com", "test")

        # Each chunk should be within token limit
        for chunk in chunks:
            token_count = chunker.count_tokens(chunk.text)
            assert token_count <= 100

    def test_chunk_to_payload(self, sample_chunks):
        """Test converting chunk to payload dictionary."""
        chunk = sample_chunks[0]
        payload = chunk.to_payload()

        assert payload["text"] == chunk.text
        assert payload["source_url"] == chunk.source_url
        assert payload["section_path"] == chunk.section_path
        assert payload["heading"] == chunk.heading
        assert payload["chunk_index"] == chunk.chunk_index
        assert payload["char_count"] == chunk.char_count

    def test_point_id_deterministic(self):
        """Test that point IDs are deterministic."""
        chunk = ContentChunk(
            text="Test text",
            source_url="https://example.com",
            section_path="test",
            heading="Test",
            chunk_index=0,
        )

        id1 = chunk.point_id
        id2 = chunk.point_id

        assert id1 == id2

    def test_different_chunks_different_ids(self, sample_chunks):
        """Test that different chunks have different point IDs."""
        ids = [chunk.point_id for chunk in sample_chunks]
        unique_ids = set(ids)

        # All IDs should be unique
        assert len(unique_ids) == len(ids)

    def test_close(self):
        """Test closing the chunker."""
        chunker = Chunker()
        chunker.close()  # Should not raise


class TestContentChunk:
    """Tests for the ContentChunk dataclass."""

    def test_chunk_auto_hashes(self):
        """Test that hashes are auto-generated."""
        chunk = ContentChunk(
            text="Test content",
            source_url="https://example.com",
            section_path="docs",
        )

        assert chunk.url_hash is not None
        assert chunk.url_hash != ""
        assert chunk.content_hash is not None
        assert chunk.content_hash != ""

    def test_chunk_char_count(self):
        """Test that char_count is auto-calculated."""
        text = "Hello, world!"
        chunk = ContentChunk(
            text=text,
            source_url="https://example.com",
            section_path="test",
        )

        assert chunk.char_count == len(text)
