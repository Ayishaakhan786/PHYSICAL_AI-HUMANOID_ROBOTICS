"""Text chunking with metadata for the RAG pipeline."""

import hashlib
import logging
from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional

import tiktoken

from .crawler import ExtractedPage

logger = logging.getLogger(__name__)


@dataclass
class ContentChunk:
    """A chunk of content extracted from a web page."""

    text: str
    source_url: str
    section_path: str
    heading: str = ""
    chunk_index: int = 0
    char_count: int = field(default=0, compare=False)
    url_hash: str = field(default="", compare=False)
    content_hash: str = field(default="", compare=False)

    def __post_init__(self):
        self.char_count = len(self.text)
        if not self.url_hash:
            self.url_hash = hashlib.sha256(self.source_url.encode()).hexdigest()[:16]
        if not self.content_hash:
            self.content_hash = hashlib.sha256(self.text.encode()).hexdigest()[:16]

    @property
    def point_id(self) -> str:
        """Generate deterministic point ID for Qdrant upsert."""
        return hashlib.sha256(
            f"{self.url_hash}:{self.content_hash}:{self.chunk_index}".encode()
        ).hexdigest()[:32]

    def to_payload(self) -> Dict[str, Any]:
        """Convert chunk to payload dictionary for Qdrant."""
        return {
            "text": self.text,
            "source_url": self.source_url,
            "section_path": self.section_path,
            "heading": self.heading,
            "chunk_index": self.chunk_index,
            "char_count": self.char_count,
            "url_hash": self.url_hash,
            "content_hash": self.content_hash,
        }


class Chunker:
    """Splits content into semantic chunks with token-based sizing."""

    def __init__(
        self,
        max_tokens: int = 512,
        overlap_tokens: int = 50,
        model_name: str = "cl100k_base",
    ):
        """
        Initialize the chunker.

        Args:
            max_tokens: Maximum tokens per chunk
            overlap_tokens: Number of tokens to overlap between chunks
            model_name: Tiktoken model name for token counting
        """
        self.max_tokens = max_tokens
        self.overlap_tokens = overlap_tokens
        self.encoding = tiktoken.get_encoding(model_name)

        logger.info(
            f"Initialized chunker: max_tokens={max_tokens}, "
            f"overlap_tokens={overlap_tokens}"
        )

    def chunk_page(self, page: ExtractedPage) -> List[ContentChunk]:
        """
        Chunk a single page into content chunks.

        Args:
            page: The extracted page to chunk

        Returns:
            List of ContentChunk objects
        """
        chunks = []
        url_hash = hashlib.sha256(page.url.encode()).hexdigest()[:16]

        # Split by paragraphs first to preserve semantic structure
        paragraphs = self._split_paragraphs(page.text)

        current_chunk_text = ""
        current_heading = ""
        chunk_index = 0

        for paragraph in paragraphs:
            # Get heading for this paragraph
            paragraph_heading = self._get_heading_for_paragraph(
                paragraph, page.headings
            )

            # Estimate tokens for this paragraph
            para_tokens = len(self.encoding.encode(paragraph))

            # Check if adding this paragraph would exceed the limit
            current_tokens = len(self.encoding.encode(current_chunk_text))

            if current_tokens + para_tokens > self.max_tokens and current_chunk_text:
                # Save current chunk
                chunk = ContentChunk(
                    text=current_chunk_text.strip(),
                    source_url=page.url,
                    section_path=page.section_path,
                    heading=current_heading,
                    chunk_index=chunk_index,
                    url_hash=url_hash,
                )
                chunks.append(chunk)
                chunk_index += 1

                # Start new chunk with overlap
                if self.overlap_tokens > 0:
                    current_chunk_text = self._get_overlap_text(
                        current_chunk_text, chunks[-1] if chunks else None
                    )
                else:
                    current_chunk_text = ""

            # Update heading if this paragraph has a different one
            if paragraph_heading:
                current_heading = paragraph_heading

            # Add paragraph to current chunk
            if current_chunk_text:
                current_chunk_text += "\n\n"
            current_chunk_text += paragraph

        # Don't forget the last chunk
        if current_chunk_text.strip():
            chunk = ContentChunk(
                text=current_chunk_text.strip(),
                source_url=page.url,
                section_path=page.section_path,
                heading=current_heading,
                chunk_index=chunk_index,
                url_hash=url_hash,
            )
            chunks.append(chunk)

        logger.info(
            f"Created {len(chunks)} chunks from {page.url} "
            f"({page.char_count} chars)"
        )

        return chunks

    def chunk_text(
        self,
        text: str,
        source_url: str,
        section_path: str,
        heading: str = "",
    ) -> List[ContentChunk]:
        """
        Chunk raw text into content chunks.

        Args:
            text: The text to chunk
            source_url: Source URL for metadata
            section_path: Section path for metadata
            heading: Heading context for metadata

        Returns:
            List of ContentChunk objects
        """
        chunks = []
        url_hash = hashlib.sha256(source_url.encode()).hexdigest()[:16]

        paragraphs = self._split_paragraphs(text)
        current_chunk_text = ""
        chunk_index = 0

        for paragraph in paragraphs:
            para_tokens = len(self.encoding.encode(paragraph))
            current_tokens = len(self.encoding.encode(current_chunk_text))

            if current_tokens + para_tokens > self.max_tokens and current_chunk_text:
                # Save current chunk
                chunk = ContentChunk(
                    text=current_chunk_text.strip(),
                    source_url=source_url,
                    section_path=section_path,
                    heading=heading,
                    chunk_index=chunk_index,
                    url_hash=url_hash,
                )
                chunks.append(chunk)
                chunk_index += 1

                # Start new chunk with overlap
                if self.overlap_tokens > 0:
                    current_chunk_text = self._get_overlap_text(
                        current_chunk_text, chunks[-1] if chunks else None
                    )
                else:
                    current_chunk_text = ""

            if current_chunk_text:
                current_chunk_text += "\n\n"
            current_chunk_text += paragraph

        # Last chunk
        if current_chunk_text.strip():
            chunk = ContentChunk(
                text=current_chunk_text.strip(),
                source_url=source_url,
                section_path=section_path,
                heading=heading,
                chunk_index=chunk_index,
                url_hash=url_hash,
            )
            chunks.append(chunk)

        return chunks

    def _split_paragraphs(self, text: str) -> List[str]:
        """Split text into paragraphs while preserving structure."""
        # Split by double newlines (paragraphs)
        paragraphs = text.split("\n\n")

        # Filter out empty paragraphs and clean up
        result = []
        for para in paragraphs:
            para = para.strip()
            if para:
                # Remove excessive internal newlines
                para = " ".join(para.split())
                if para:
                    result.append(para)

        return result

    def _get_heading_for_paragraph(
        self, paragraph: str, headings: List[str]
    ) -> Optional[str]:
        """
        Find the heading that precedes this paragraph.

        This is a simple implementation that could be improved with
        positional information from the original HTML.
        """
        # For now, return the first heading if available
        # A more sophisticated implementation would track positions
        return headings[0] if headings else ""

    def _get_overlap_text(
        self, text: str, previous_chunk: Optional[ContentChunk]
    ) -> str:
        """
        Get overlapping text from the end of the previous chunk.

        Args:
            text: The text to extract overlap from
            previous_chunk: The previous chunk for reference

        Returns:
            Overlapping text to include in next chunk
        """
        if not previous_chunk:
            # If no previous chunk, just take from the start of text
            tokens = self.encoding.encode(text)
            overlap_tokens = tokens[-self.overlap_tokens:]
            return self.encoding.decode(overlap_tokens)

        # Take the last N tokens from the text
        tokens = self.encoding.encode(text)
        if len(tokens) <= self.overlap_tokens:
            return text

        overlap_tokens = tokens[-self.overlap_tokens:]
        return self.encoding.decode(overlap_tokens)

    def count_tokens(self, text: str) -> int:
        """Count the number of tokens in a text string."""
        return len(self.encoding.encode(text))

    def close(self) -> None:
        """Close the tiktoken encoding."""
        self.encoding.close()
