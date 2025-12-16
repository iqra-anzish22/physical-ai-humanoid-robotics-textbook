from typing import List, Dict, Any
import logging
from .vector_storage_service import VectorStorageService
from .embedding_service import EmbeddingService


class RetrievalService:
    def __init__(self):
        self.vector_storage_service = VectorStorageService()
        self.embedding_service = EmbeddingService()

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
            query_embedding = self.embedding_service.generate_single_embedding(query)

            # Search for similar chunks in vector storage
            results = self.vector_storage_service.retrieve_relevant_chunks(
                session_id=session_id,
                query=query_embedding,
                top_k=top_k
            )

            return results
        except Exception as e:
            logging.error(f"Error in retrieval service: {str(e)}")
            # Return empty list as a graceful degradation
            return []

    def retrieve_with_reranking(self, session_id: str, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant chunks and rerank them for better relevance.

        Args:
            session_id: The session ID
            query: The query text
            top_k: Number of top results to return

        Returns:
            List of reranked relevant chunks
        """
        try:
            # First retrieve relevant chunks
            initial_results = self.retrieve_relevant_chunks(session_id, query, top_k * 2)  # Get more for reranking

            if not initial_results:
                return []

            # Extract text contents for reranking
            documents = [chunk["text"] for chunk in initial_results]

            # Rerank using Cohere
            from .cohere_client import CohereService
            cohere_service = CohereService()
            reranked_results = cohere_service.rerank(query, documents, top_n=top_k)

            # Reorder results based on reranking
            reranked_chunks = []
            for rerank_result in reranked_results:
                original_chunk = initial_results[rerank_result["index"]]
                original_chunk["rerank_score"] = rerank_result["relevance_score"]
                reranked_chunks.append(original_chunk)

            # Return top_k results
            return reranked_chunks[:top_k]

        except Exception as e:
            logging.error(f"Error in retrieval with reranking: {str(e)}")
            # Fall back to regular retrieval if reranking fails
            try:
                return self.retrieve_relevant_chunks(session_id, query, top_k)
            except Exception:
                # If even the fallback fails, return empty list
                return []