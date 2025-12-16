import re
from typing import List, Tuple
import tiktoken
from ..config import settings


class TextProcessor:
    def __init__(self):
        # Use the cl100k_base tokenizer which is suitable for most models
        self.tokenizer = tiktoken.get_encoding("cl100k_base")

    def count_tokens(self, text: str) -> int:
        """
        Count the number of tokens in a text string.

        Args:
            text: The text to count tokens for

        Returns:
            Number of tokens in the text
        """
        return len(self.tokenizer.encode(text))

    def chunk_text(self, text: str, chunk_size: int = None, overlap: int = None) -> List[dict]:
        """
        Split text into chunks of specified token size with overlap.

        Args:
            text: The text to chunk
            chunk_size: Size of each chunk in tokens (default from settings)
            overlap: Overlap between chunks in tokens (default from settings)

        Returns:
            List of dictionaries containing chunk_id, content, token_count, and position
        """
        if chunk_size is None:
            chunk_size = settings.default_chunk_size
        if overlap is None:
            overlap = settings.default_overlap

        # Split text into sentences to avoid breaking sentences across chunks
        sentences = re.split(r'[.!?]+\s+', text)
        chunks = []
        current_chunk = ""
        current_token_count = 0
        chunk_position = 0

        for sentence in sentences:
            # Estimate token count for the sentence
            sentence_token_count = self.count_tokens(sentence)

            # If adding this sentence would exceed chunk size
            if current_token_count + sentence_token_count > chunk_size and current_chunk:
                # Save the current chunk
                chunks.append({
                    "content": current_chunk.strip(),
                    "token_count": current_token_count,
                    "position": chunk_position
                })
                chunk_position += 1

                # Start a new chunk with overlap
                if overlap > 0:
                    # Get the last part of the current chunk for overlap
                    overlap_text = self._get_overlap_text(current_chunk, overlap)
                    current_chunk = overlap_text + " " + sentence
                else:
                    current_chunk = sentence

                current_token_count = self.count_tokens(current_chunk)
            else:
                # Add sentence to current chunk
                if current_chunk:
                    current_chunk += " " + sentence
                else:
                    current_chunk = sentence
                current_token_count += sentence_token_count

        # Add the final chunk if it has content
        if current_chunk.strip():
            chunks.append({
                "content": current_chunk.strip(),
                "token_count": current_token_count,
                "position": chunk_position
            })

        # Create chunk IDs and return
        result = []
        for i, chunk in enumerate(chunks):
            result.append({
                "chunk_id": f"chunk_{i:04d}",
                "content": chunk["content"],
                "token_count": chunk["token_count"],
                "position": chunk["position"]
            })

        return result

    def _get_overlap_text(self, text: str, overlap_tokens: int) -> str:
        """
        Get the last part of text with approximately the specified number of tokens.

        Args:
            text: The text to extract overlap from
            overlap_tokens: Number of tokens for overlap

        Returns:
            Overlapping text
        """
        tokens = self.tokenizer.encode(text)
        if len(tokens) <= overlap_tokens:
            return self.tokenizer.decode(tokens)

        # Get the last 'overlap_tokens' tokens
        overlap_tokens_list = tokens[-overlap_tokens:]
        return self.tokenizer.decode(overlap_tokens_list)

    def validate_text(self, text: str) -> Tuple[bool, str]:
        """
        Validate the provided text.

        Args:
            text: The text to validate

        Returns:
            Tuple of (is_valid, error_message)
        """
        if not text or not text.strip():
            return False, "Text cannot be empty"

        if len(text) > settings.max_text_length:
            return False, f"Text exceeds maximum length of {settings.max_text_length} characters"

        return True, ""