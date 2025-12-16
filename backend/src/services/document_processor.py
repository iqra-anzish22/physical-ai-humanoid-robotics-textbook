import re
from typing import List, Optional, Dict, Any
from dataclasses import dataclass
from langdetect import detect
from urllib.parse import urlparse

from src.models.chunk import Chunk
from src.models.document import Document, ProcessingStatus
from src.services.embedding_service import EmbeddingService
from src.utils.logger import logger


@dataclass
class ProcessingConfig:
    """Configuration for document processing."""
    chunk_size: int = 500
    chunk_overlap: int = 50
    min_chunk_size: int = 100
    max_chunk_size: int = 2000
    split_by_sentence: bool = True
    remove_extra_whitespace: bool = True
    preserve_line_breaks: bool = False


class DocumentProcessor:
    """Service for processing and chunking documents."""

    def __init__(self, embedding_service: EmbeddingService, config: Optional[ProcessingConfig] = None):
        self.embedding_service = embedding_service
        self.config = config or ProcessingConfig()

    def process_document(self, document: Document) -> List[Chunk]:
        """
        Process a document and create chunks suitable for embedding.

        Args:
            document: Document to process

        Returns:
            List of chunks created from the document
        """
        try:
            # Mark document as processing
            document.mark_processing()

            # Preprocess content
            processed_content = self._preprocess_content(document.content)

            # Split content into chunks
            chunks = self._split_content(processed_content, document)

            # Process each chunk
            processed_chunks = []
            for i, chunk in enumerate(chunks):
                processed_chunk = self._process_chunk(chunk, document, i)
                processed_chunks.append(processed_chunk)

            # Mark document as processed
            document.mark_processed()

            logger.info(f"Successfully processed document {document.id} into {len(processed_chunks)} chunks")
            return processed_chunks

        except Exception as e:
            logger.error(f"Error processing document {document.id}: {str(e)}")
            document.mark_failed(str(e))
            raise

    def _preprocess_content(self, content: str) -> str:
        """Preprocess document content before splitting."""
        processed = content

        # Remove extra whitespace if configured
        if self.config.remove_extra_whitespace:
            processed = re.sub(r'\s+', ' ', processed)

        # Preserve line breaks if configured
        if not self.config.preserve_line_breaks:
            processed = processed.replace('\n', ' ')

        # Normalize text
        processed = processed.strip()

        return processed

    def _split_content(self, content: str, document: Document) -> List[str]:
        """Split content into chunks based on configuration."""
        if self.config.split_by_sentence:
            return self._split_by_sentence(content)
        else:
            return self._split_by_length(content)

    def _split_by_sentence(self, content: str) -> List[str]:
        """Split content by sentences while respecting chunk size limits."""
        # Split by sentence endings
        sentences = re.split(r'[.!?]+', content)

        chunks = []
        current_chunk = ""

        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue

            # Check if adding this sentence would exceed the max chunk size
            if len(current_chunk) + len(sentence) > self.config.max_chunk_size:
                if len(current_chunk) >= self.config.min_chunk_size:
                    chunks.append(current_chunk.strip())
                    current_chunk = sentence + ". "
                else:
                    # If the current chunk is too small, just add the sentence
                    current_chunk += sentence + ". "
            elif len(current_chunk) + len(sentence) <= self.config.chunk_size:
                # If it fits within the target chunk size, add it
                current_chunk += sentence + ". "
            else:
                # If adding this sentence would exceed target size but not max size,
                # check if we should start a new chunk
                if len(current_chunk) >= self.config.chunk_size - self.config.chunk_overlap:
                    chunks.append(current_chunk.strip())
                    # Start new chunk with overlap
                    current_chunk = self._get_overlap(current_chunk, self.config.chunk_overlap) + sentence + ". "
                else:
                    current_chunk += sentence + ". "

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        # Further split any chunks that are still too large
        final_chunks = []
        for chunk in chunks:
            if len(chunk) > self.config.max_chunk_size:
                final_chunks.extend(self._split_large_chunk(chunk))
            else:
                final_chunks.append(chunk)

        return final_chunks

    def _split_by_length(self, content: str) -> List[str]:
        """Split content by character length."""
        chunks = []
        start = 0
        content_len = len(content)

        while start < content_len:
            end = start + self.config.chunk_size
            if end > content_len:
                end = content_len

            # Add overlap if not at the end
            if end < content_len and self.config.chunk_overlap > 0:
                overlap_start = end - self.config.chunk_overlap
                if overlap_start > start:
                    end = min(end + self.config.chunk_overlap, content_len)

            chunk = content[start:end].strip()
            if chunk:
                chunks.append(chunk)

            start = end

        return chunks

    def _get_overlap(self, text: str, overlap_size: int) -> str:
        """Get the last overlap_size characters from text, preferably ending at a word boundary."""
        if len(text) <= overlap_size:
            return text

        # Find the last space within the overlap range
        substr = text[-overlap_size:]
        last_space = substr.rfind(' ')

        if last_space != -1:
            return substr[last_space+1:]
        else:
            return substr[:overlap_size]

    def _split_large_chunk(self, chunk: str) -> List[str]:
        """Split a chunk that is too large into smaller pieces."""
        if len(chunk) <= self.config.max_chunk_size:
            return [chunk]

        # Split the large chunk by length
        subchunks = []
        start = 0
        chunk_len = len(chunk)

        while start < chunk_len:
            end = start + self.config.max_chunk_size
            if end > chunk_len:
                end = chunk_len

            subchunk = chunk[start:end].strip()
            if subchunk:
                subchunks.append(subchunk)

            start = end

        return subchunks

    def _process_chunk(self, content: str, document: Document, chunk_index: int) -> Chunk:
        """Process a single chunk and create a Chunk object."""
        # Create chunk ID using document ID and chunk index
        chunk_id = f"{document.id}_{chunk_index}"

        # Detect language if possible
        language = "unknown"
        try:
            if len(content) > 10:  # Need enough text for reliable detection
                language = detect(content)
        except Exception:
            # If detection fails, keep as unknown
            pass

        # Create metadata for the chunk
        chunk_metadata = {
            "document_id": document.id,
            "filename": document.original_filename,
            "chunk_index": chunk_index,
            "language": language,
            "source": "knowledge_pipeline",
            **document.metadata  # Include document-level metadata
        }

        # Create the chunk object
        chunk = Chunk(
            id=chunk_id,
            text=content,
            document_id=document.id,
            metadata=chunk_metadata
        )

        return chunk

    def validate_chunk_quality(self, chunk: Chunk) -> bool:
        """Validate if a chunk meets quality requirements."""
        # Check minimum length
        if len(chunk.text.strip()) < self.config.min_chunk_size:
            return False

        # Check for meaningful content (not just special characters)
        if not re.search(r'[a-zA-Z0-9]', chunk.text):
            return False

        return True

    def post_process_chunks(self, chunks: List[Chunk]) -> List[Chunk]:
        """Apply post-processing to chunks."""
        processed_chunks = []

        for chunk in chunks:
            # Validate chunk quality
            if self.validate_chunk_quality(chunk):
                # Apply any additional transformations
                processed_chunk = self._transform_chunk(chunk)
                processed_chunks.append(processed_chunk)

        return processed_chunks

    def _transform_chunk(self, chunk: Chunk) -> Chunk:
        """Apply transformations to a chunk."""
        # Clean up the text further if needed
        cleaned_text = self._clean_chunk_text(chunk.text)
        chunk.text = cleaned_text

        return chunk

    def _clean_chunk_text(self, text: str) -> str:
        """Clean up chunk text by removing problematic characters."""
        # Remove extra whitespace
        cleaned = re.sub(r'\s+', ' ', text)

        # Remove control characters except common ones
        cleaned = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]', '', cleaned)

        return cleaned.strip()