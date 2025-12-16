import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
import uuid
from typing import List, Optional
import logging
from datetime import datetime

from ..config import settings

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RAGPipeline:
    def __init__(self):
        # Initialize Cohere client
        self.cohere_client = cohere.Client(settings.cohere_api_key)

        # Initialize Qdrant client
        self.qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )

        # Collection name will be based on session ID for isolation
        self.collection_prefix = "session_"

        logger.info("RAG Pipeline initialized successfully")

    async def process_query(self, session_id: str, query: str, text_content: Optional[str] = None) -> str:
        """
        Process a user query using the RAG pipeline
        """
        try:
            # Create collection for this session if it doesn't exist
            collection_name = f"{self.collection_prefix}{session_id}"
            await self._ensure_collection_exists(collection_name)

            # If text_content is provided, ingest it into the session's collection
            if text_content:
                await self._ingest_text_content(collection_name, text_content)

            # Embed the query using Cohere
            query_embedding = await self._embed_text([query])

            # Search for relevant context in the session's collection
            relevant_chunks = await self._search_context(collection_name, query_embedding[0])

            # If no relevant context found, return appropriate response
            if not relevant_chunks:
                return "This topic is not covered in the textbook."

            # Generate response using the retrieved context
            response = await self._generate_response(query, relevant_chunks)

            return response
        except Exception as e:
            logger.error(f"Error in RAG pipeline: {str(e)}")
            raise

    async def _ensure_collection_exists(self, collection_name: str):
        """
        Ensure that the Qdrant collection exists for this session
        """
        try:
            # Check if collection exists
            self.qdrant_client.get_collection(collection_name)
        except:
            # Create collection if it doesn't exist
            self.qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE),  # Cohere embeddings are 1024-dim
            )
            logger.info(f"Created collection: {collection_name}")

    async def _ingest_text_content(self, collection_name: str, text_content: str):
        """
        Ingest text content into the session's collection
        """
        # Simple chunking - in a real implementation, you'd want more sophisticated chunking
        chunks = self._chunk_text(text_content)

        # Embed the chunks using Cohere
        embeddings = await self._embed_text(chunks)

        # Prepare points for Qdrant
        points = []
        for i, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "text": chunk,
                    "chunk_id": i,
                    "created_at": datetime.utcnow().isoformat()
                }
            )
            points.append(point)

        # Upload to Qdrant
        if points:
            self.qdrant_client.upsert(
                collection_name=collection_name,
                points=points
            )
            logger.info(f"Ingested {len(points)} chunks into collection: {collection_name}")

    def _chunk_text(self, text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
        """
        Simple text chunking function
        """
        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size
            chunk = text[start:end]
            chunks.append(chunk)
            start = end - overlap

        return chunks

    async def _embed_text(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for text using Cohere
        """
        response = self.cohere_client.embed(
            texts=texts,
            model="embed-english-v3.0",
            input_type="search_document"
        )
        return [embedding for embedding in response.embeddings]

    async def _search_context(self, collection_name: str, query_embedding: List[float], top_k: int = 5) -> List[str]:
        """
        Search for relevant context in the collection
        """
        try:
            search_results = self.qdrant_client.search(
                collection_name=collection_name,
                query_vector=query_embedding,
                limit=top_k
            )

            # Extract text content from the results
            relevant_chunks = [result.payload.get("text", "") for result in search_results if result.payload]

            return relevant_chunks
        except Exception as e:
            logger.error(f"Error searching context: {str(e)}")
            return []

    async def _generate_response(self, query: str, context_chunks: List[str]) -> str:
        """
        Generate a response using the query and retrieved context
        """
        # Combine context chunks into a single context string
        context = "\n".join(context_chunks)

        # Create a prompt for Cohere
        prompt = f"""
        Based on the following context, please answer the question. If the context doesn't contain information to answer the question, respond with "This topic is not covered in the textbook."

        Context: {context}

        Question: {query}

        Answer:
        """

        # Generate response using Cohere
        response = self.cohere_client.generate(
            model="command-r-plus",
            prompt=prompt,
            max_tokens=500,
            temperature=0.3
        )

        return response.generations[0].text.strip()

    def cleanup_session_data(self, session_id: str):
        """
        Clean up session-specific data from Qdrant
        """
        try:
            collection_name = f"{self.collection_prefix}{session_id}"
            self.qdrant_client.delete_collection(collection_name)
            logger.info(f"Cleaned up collection: {collection_name}")
        except Exception as e:
            logger.error(f"Error cleaning up session data: {str(e)}")