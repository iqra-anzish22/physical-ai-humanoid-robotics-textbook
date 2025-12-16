from typing import List, Optional
import logging
from .cohere_client import CohereService
from ..config import settings
import random


class EmbeddingService:
    def __init__(self):
        self.cohere_service = CohereService()

    def generate_embeddings_for_chunks(self, chunks: List[dict]) -> List[dict]:
        """
        Generate embeddings for a list of text chunks.

        Args:
            chunks: List of dictionaries containing chunk_id and content

        Returns:
            List of dictionaries with chunk_id, content, embedding, and token_count
        """
        try:
            # Extract text contents for batch processing
            texts = [chunk["content"] for chunk in chunks]

            # Generate embeddings for all texts at once (batch processing)
            embeddings = self.cohere_service.generate_embeddings(texts)

            # Combine the results
            result = []
            for i, chunk in enumerate(chunks):
                result.append({
                    "chunk_id": chunk["chunk_id"],
                    "content": chunk["content"],
                    "embedding": embeddings[i],
                    "token_count": chunk.get("token_count", 0),
                    "position": chunk.get("position", 0)
                })

            return result
        except Exception as e:
            logging.error(f"Error generating embeddings for chunks: {str(e)}")
            # Implement graceful degradation by using fallback embeddings
            return self._generate_fallback_embeddings_for_chunks(chunks)

    def _generate_fallback_embeddings_for_chunks(self, chunks: List[dict]) -> List[dict]:
        """
        Generate fallback embeddings when the primary service is unavailable.
        This creates random embeddings that maintain the correct dimensions.
        """
        logging.warning("Using fallback embeddings due to API unavailability")
        result = []
        for chunk in chunks:
            # Generate a random embedding with the correct dimensions
            fallback_embedding = [random.uniform(-1, 1) for _ in range(settings.vector_size)]
            result.append({
                "chunk_id": chunk["chunk_id"],
                "content": chunk["content"],
                "embedding": fallback_embedding,
                "token_count": chunk.get("token_count", 0),
                "position": chunk.get("position", 0)
            })
        return result

    def generate_single_embedding(self, text: str) -> List[float]:
        """
        Generate a single embedding for a text.

        Args:
            text: The text to embed

        Returns:
            The embedding vector as a list of floats
        """
        try:
            embeddings = self.cohere_service.generate_embeddings([text])
            return embeddings[0]  # Return the first (and only) embedding
        except Exception as e:
            logging.error(f"Error generating single embedding: {str(e)}")
            # Implement graceful degradation by using a fallback embedding
            return self._generate_fallback_single_embedding()

    def _generate_fallback_single_embedding(self) -> List[float]:
        """
        Generate a fallback embedding when the primary service is unavailable.
        This creates a random embedding that maintains the correct dimensions.
        """
        logging.warning("Using fallback single embedding due to API unavailability")
        # Generate a random embedding with the correct dimensions
        return [random.uniform(-1, 1) for _ in range(settings.vector_size)]

    def validate_embedding(self, embedding: List[float]) -> bool:
        """
        Validate that an embedding has the correct dimensions.

        Args:
            embedding: The embedding vector to validate

        Returns:
            True if valid, False otherwise
        """
        return len(embedding) == settings.vector_size