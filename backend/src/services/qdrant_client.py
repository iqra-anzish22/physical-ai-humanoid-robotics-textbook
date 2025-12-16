from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any, Optional
import logging
from ..config import settings


class QdrantService:
    def __init__(self):
        # Initialize Qdrant client with settings
        self.client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=False  # Using REST API
        )
        self.collection_prefix = settings.qdrant_collection_prefix
        self.vector_size = settings.vector_size

    def create_session_collection(self, session_id: str) -> bool:
        """
        Create a new collection for a specific session.

        Args:
            session_id: Unique identifier for the session

        Returns:
            True if collection was created successfully, False otherwise
        """
        try:
            collection_name = f"{self.collection_prefix}{session_id}"

            # Check if collection already exists
            try:
                self.client.get_collection(collection_name)
                # Collection already exists, we can use it
                return True
            except:
                # Collection doesn't exist, create it
                self.client.create_collection(
                    collection_name=collection_name,
                    vectors_config=models.VectorParams(
                        size=self.vector_size,
                        distance=models.Distance.COSINE
                    )
                )
                return True
        except Exception as e:
            logging.error(f"Error creating session collection: {str(e)}")
            return False

    def store_embeddings(self, session_id: str, chunk_id: str, text: str, embedding: List[float],
                        metadata: Optional[Dict[str, Any]] = None) -> bool:
        """
        Store a text chunk with its embedding in the session collection.

        Args:
            session_id: Unique identifier for the session
            chunk_id: Unique identifier for the text chunk
            text: The text content
            embedding: The embedding vector
            metadata: Optional metadata to store with the chunk

        Returns:
            True if stored successfully, False otherwise
        """
        try:
            collection_name = f"{self.collection_prefix}{session_id}"

            # Prepare the payload
            payload = {
                "text": text,
                "chunk_id": chunk_id
            }
            if metadata:
                payload.update(metadata)

            # Store the embedding
            self.client.upsert(
                collection_name=collection_name,
                points=[
                    models.PointStruct(
                        id=chunk_id,
                        vector=embedding,
                        payload=payload
                    )
                ]
            )
            return True
        except Exception as e:
            logging.error(f"Error storing embedding: {str(e)}")
            return False

    def search_similar(self, session_id: str, query_embedding: List[float],
                      top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Search for similar text chunks based on the query embedding.

        Args:
            session_id: Unique identifier for the session
            query_embedding: The embedding to search for similar items
            top_k: Number of top results to return

        Returns:
            List of similar chunks with their text and metadata
        """
        try:
            collection_name = f"{self.collection_prefix}{session_id}"

            # Perform search
            results = self.client.search(
                collection_name=collection_name,
                query_vector=query_embedding,
                limit=top_k
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "chunk_id": result.payload.get("chunk_id"),
                    "text": result.payload.get("text"),
                    "score": result.score,
                    "metadata": {k: v for k, v in result.payload.items()
                               if k not in ["text", "chunk_id"]}
                })

            return formatted_results
        except Exception as e:
            logging.error(f"Error searching for similar chunks: {str(e)}")
            return []

    def delete_collection(self, session_id: str) -> bool:
        """
        Delete the collection for a specific session.

        Args:
            session_id: Unique identifier for the session

        Returns:
            True if collection was deleted successfully, False otherwise
        """
        try:
            collection_name = f"{self.collection_prefix}{session_id}"
            self.client.delete_collection(collection_name)
            return True
        except Exception as e:
            logging.error(f"Error deleting collection: {str(e)}")
            return False

    def collection_exists(self, session_id: str) -> bool:
        """
        Check if a collection for the session exists.

        Args:
            session_id: Unique identifier for the session

        Returns:
            True if collection exists, False otherwise
        """
        try:
            collection_name = f"{self.collection_prefix}{session_id}"
            self.client.get_collection(collection_name)
            return True
        except:
            return False