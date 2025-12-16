import re
import logging
from typing import List
from src.models.chunk import TextChunk


class TextProcessor:
    """Service for processing and validating text content."""

    def __init__(self):
        self.min_text_length = 10
        self.max_text_length = 100000  # 100k characters max

    def validate_text(self, text: str) -> tuple[bool, str]:
        """
        Validate text content.

        Args:
            text: Text to validate

        Returns:
            Tuple of (is_valid, error_message)
        """
        if not text or len(text.strip()) < self.min_text_length:
            return False, f"Text must be at least {self.min_text_length} characters long"

        if len(text) > self.max_text_length:
            return False, f"Text exceeds maximum length of {self.max_text_length} characters"

        # Check for meaningful content
        if not re.search(r'[a-zA-Z0-9]', text):
            return False, "Text must contain alphanumeric characters"

        return True, ""

    def chunk_text(self, text: str, chunk_size: int = 500, overlap: int = 50) -> List[TextChunk]:
        """
        Split text into chunks.

        Args:
            text: Text to chunk
            chunk_size: Target size of each chunk in characters
            overlap: Number of characters to overlap between chunks

        Returns:
            List of TextChunk objects
        """
        chunks = []
        text_length = len(text)
        chunk_index = 0

        if text_length <= chunk_size:
            # If text is smaller than chunk size, return as single chunk
            chunk = TextChunk(
                chunk_id=f"chunk_{chunk_index}",
                session_id="temp_session",
                content=text.strip(),
                position=chunk_index,
                token_count=len(text.split())
            )
            chunks.append(chunk)
            return chunks

        start = 0

        while start < text_length:
            end = start + chunk_size

            # If we're near the end, include the remainder
            if end > text_length:
                end = text_length

            # Extract the chunk
            chunk_text = text[start:end]

            # Create chunk object
            chunk = TextChunk(
                chunk_id=f"chunk_{chunk_index}",
                session_id="temp_session",
                content=chunk_text.strip(),
                position=chunk_index,
                token_count=len(chunk_text.split())
            )

            chunks.append(chunk)

            # Move to next position with overlap
            start = end - overlap if overlap < end else end
            chunk_index += 1

            # Safety check to prevent infinite loops
            if start >= text_length:
                break

        logging.info(f"Text chunked into {len(chunks)} chunks")
        return chunks

    def clean_text(self, text: str) -> str:
        """
        Clean text by removing extra whitespace and normalizing.

        Args:
            text: Text to clean

        Returns:
            Cleaned text
        """
        # Remove extra whitespace
        cleaned = re.sub(r'\s+', ' ', text)

        # Remove control characters
        cleaned = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]', '', cleaned)

        return cleaned.strip()

    def preprocess_text(self, text: str) -> str:
        """
        Preprocess text before chunking.

        Args:
            text: Text to preprocess

        Returns:
            Preprocessed text
        """
        return self.clean_text(text)