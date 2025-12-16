from typing import List, Dict, Any, Optional
from ..models.chunk import TextChunk, Chunk
import logging
from .qdrant_client import QdrantService
from ..config import settings


class VectorStorageService:
    def __init__(self):
        self.qdrant_service = QdrantService()

    def create_session_storage(self, session_id: str) -> bool:
        """
        Create temporary storage for a session.

        Args:
            session_id: The session ID

        Returns:
            True if storage was created successfully, False otherwise
        """
        try:
            return self.qdrant_service.create_session_collection(session_id)
        except Exception as e:
            logging.error(f"Error creating session storage: {str(e)}")
            return False

    def store_text_chunks(self, session_id: str, chunk_embeddings: List[dict]) -> bool:
        """
        Store text chunks with their embeddings in the session storage.

        Args:
            session_id: The session ID
            chunk_embeddings: List of dictionaries with chunk_id, content, and embedding

        Returns:
            True if all chunks were stored successfully, False otherwise
        """
        try:
            success_count = 0
            for chunk_data in chunk_embeddings:
                success = self.qdrant_service.store_embeddings(
                    session_id=session_id,
                    chunk_id=chunk_data["chunk_id"],
                    text=chunk_data["content"],
                    embedding=chunk_data["embedding"],
                    metadata={
                        "position": chunk_data.get("position", 0),
                        "token_count": chunk_data.get("token_count", 0)
                    }
                )
                if success:
                    success_count += 1

            return success_count == len(chunk_embeddings)
        except Exception as e:
            logging.error(f"Error storing text chunks: {str(e)}")
            return False

    def retrieve_relevant_chunks(self, session_id: str, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve the most relevant text chunks for a query.

        Args:
            session_id: The session ID
            query: The query text
            top_k: Number of top results to return

        Returns:
            List of relevant chunks with their text and metadata
        """
        try:
            # Generate embedding for the query
            from .embedding_service import EmbeddingService
            embedding_service = EmbeddingService()
            query_embedding = embedding_service.generate_single_embedding(query)

            # Search for similar chunks
            results = self.qdrant_service.search_similar(
                session_id=session_id,
                query_embedding=query_embedding,
                top_k=top_k
            )

            return results
        except Exception as e:
            logging.error(f"Error retrieving relevant chunks: {str(e)}")
            return []

    def cleanup_session_storage(self, session_id: str) -> bool:
        """
        Delete all data for a session.

        Args:
            session_id: The session ID

        Returns:
            True if cleanup was successful, False otherwise
        """
        try:
            return self.qdrant_service.delete_collection(session_id)
        except Exception as e:
            logging.error(f"Error cleaning up session storage: {str(e)}")
            return False

    def store_chunks(self, chunks: List[Any]) -> bool:
        """
        Store general chunks (from knowledge pipeline) in persistent storage.

        Args:
            chunks: List of chunk objects (either TextChunk or Chunk)

        Returns:
            True if all chunks were stored successfully, False otherwise
        """
        try:
            success_count = 0

            for chunk in chunks:
                # Determine the chunk type and extract appropriate fields
                if hasattr(chunk, 'chunk_id'):  # TextChunk
                    chunk_id = chunk.chunk_id
                    text = chunk.content
                    session_id = chunk.session_id
                    metadata = {
                        "position": chunk.position,
                        "token_count": chunk.token_count,
                        "session_id": session_id
                    }
                elif hasattr(chunk, 'id'):  # General Chunk
                    chunk_id = chunk.id
                    text = chunk.text
                    session_id = f"doc_{chunk.document_id}"  # Use document ID as session
                    metadata = {
                        "position": chunk.position,
                        "token_count": chunk.token_count,
                        "document_id": chunk.document_id,
                        **chunk.metadata  # Include chunk-specific metadata
                    }
                else:
                    logging.warning(f"Unknown chunk type: {type(chunk)}")
                    continue

                # Generate embedding for the chunk
                from .embedding_service import EmbeddingService
                embedding_service = EmbeddingService()
                embedding = embedding_service.generate_single_embedding(text)

                # Store in Qdrant
                success = self.qdrant_service.store_embeddings(
                    session_id=session_id,
                    chunk_id=chunk_id,
                    text=text,
                    embedding=embedding,
                    metadata=metadata
                )

                if success:
                    success_count += 1

            return success_count == len(chunks)
        except Exception as e:
            logging.error(f"Error storing general chunks: {str(e)}")
            return False

    def session_storage_exists(self, session_id: str) -> bool:
        """
        Check if storage exists for a session.

        Args:
            session_id: The session ID

        Returns:
            True if storage exists, False otherwise
        """
        try:
            return self.qdrant_service.collection_exists(session_id)
        except Exception as e:
            logging.error(f"Error checking session storage: {str(e)}")
            return False