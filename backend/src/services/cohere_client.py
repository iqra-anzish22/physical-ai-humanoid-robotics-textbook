import cohere
from typing import List, Optional
import logging
import time
from ..config import settings


class CohereService:
    def __init__(self):
        self.client = cohere.Client(api_key=settings.cohere_api_key)
        self.embedding_model = settings.cohere_embedding_model
        self.generation_model = settings.cohere_generation_model
        self.timeout = 30  # 30 second timeout for API calls

    def generate_embeddings(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts using Cohere.

        Args:
            texts: List of text strings to embed

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        start_time = time.time()
        try:
            response = self.client.embed(
                texts=texts,
                model=self.embedding_model,
                input_type="search_document"  # Using search_document for text chunks
            )
            elapsed = time.time() - start_time
            if elapsed > self.timeout:
                logging.warning(f"Embedding generation took {elapsed:.2f}s, exceeding timeout of {self.timeout}s")
            return [embedding for embedding in response.embeddings]
        except Exception as e:
            elapsed = time.time() - start_time
            logging.error(f"Error generating embeddings after {elapsed:.2f}s: {str(e)}")
            raise

    def generate_response(self, prompt: str, context_texts: List[str] = None) -> str:
        """
        Generate a response using Cohere based on the prompt and optional context.

        Args:
            prompt: The question or prompt to generate a response for
            context_texts: Optional list of context texts to provide as background

        Returns:
            Generated response text
        """
        start_time = time.time()
        try:
            # If context is provided, include it in the prompt
            if context_texts:
                context = "\n\n".join(context_texts)
                full_prompt = f"Based on the following text:\n\n{context}\n\nQuestion: {prompt}\n\nAnswer:"
            else:
                full_prompt = prompt

            response = self.client.generate(
                model=self.generation_model,
                prompt=full_prompt,
                max_tokens=500,
                temperature=0.3,
                stop_sequences=["\n\n"]
            )

            elapsed = time.time() - start_time
            if elapsed > self.timeout:
                logging.warning(f"Response generation took {elapsed:.2f}s, exceeding timeout of {self.timeout}s")

            return response.generations[0].text.strip()
        except Exception as e:
            elapsed = time.time() - start_time
            logging.error(f"Error generating response after {elapsed:.2f}s: {str(e)}")
            raise

    def rerank(self, query: str, documents: List[str], top_n: int = 5) -> List[dict]:
        """
        Rerank documents based on relevance to the query.

        Args:
            query: The search query
            documents: List of documents to rerank
            top_n: Number of top results to return

        Returns:
            List of dictionaries with index and relevance score
        """
        start_time = time.time()
        try:
            response = self.client.rerank(
                model="rerank-english-v2.0",
                query=query,
                documents=documents,
                top_n=top_n
            )
            elapsed = time.time() - start_time
            if elapsed > self.timeout:
                logging.warning(f"Reranking took {elapsed:.2f}s, exceeding timeout of {self.timeout}s")
            return [{"index": r.index, "relevance_score": r.relevance_score} for r in response.results]
        except Exception as e:
            elapsed = time.time() - start_time
            logging.error(f"Error reranking documents after {elapsed:.2f}s: {str(e)}")
            raise