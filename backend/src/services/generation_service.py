from typing import List, Dict, Any
import logging
from .cohere_client import CohereService


class GenerationService:
    def __init__(self):
        self.cohere_service = CohereService()

    def generate_answer(self, question: str, context_chunks: List[Dict[str, Any]]) -> str:
        """
        Generate an answer based on the question and context chunks.

        Args:
            question: The user's question
            context_chunks: List of relevant context chunks

        Returns:
            Generated answer
        """
        try:
            # Extract text from context chunks
            context_texts = [chunk["text"] for chunk in context_chunks]

            # Generate response using Cohere
            response = self.cohere_service.generate_response(question, context_texts)

            return response
        except Exception as e:
            logging.error(f"Error in generation service: {str(e)}")
            raise

    def validate_answer(self, answer: str, context_chunks: List[Dict[str, Any]]) -> bool:
        """
        Validate that the answer is grounded in the provided context.

        Args:
            answer: The generated answer
            context_chunks: The context chunks that were used

        Returns:
            True if the answer is grounded in context, False otherwise
        """
        # This is a basic validation - in a real implementation, you might use more sophisticated
        # techniques to check if the answer is grounded in the provided context
        context_text = " ".join([chunk["text"] for chunk in context_chunks])

        # Check if the answer contains information that's not in the context
        # This is a simplified check - real implementation would need more sophisticated grounding validation
        if not answer or not context_text:
            return False

        # Basic check: ensure answer is not completely unrelated to context
        answer_lower = answer.lower()
        context_lower = context_text.lower()

        # Check if answer has some connection to context (at least some common words)
        answer_words = set(answer_lower.split()[:10])  # Check first 10 words
        context_words = set(context_lower.split())

        # If less than 20% of answer words appear in context, it might not be grounded
        common_words = answer_words.intersection(context_words)
        if len(common_words) < max(1, len(answer_words) * 0.2):
            logging.warning("Generated answer may not be well-grounded in the provided context")
            return False

        return True

    def generate_answer_with_validation(self, question: str, context_chunks: List[Dict[str, Any]]) -> str:
        """
        Generate an answer and validate it's grounded in the context.

        Args:
            question: The user's question
            context_chunks: List of relevant context chunks

        Returns:
            Generated answer (validated to be grounded in context)
        """
        answer = self.generate_answer(question, context_chunks)

        # Validate the answer is grounded in the provided context
        is_valid = self.validate_answer(answer, context_chunks)

        if not is_valid:
            # If validation fails, generate a response acknowledging the limitation
            return "I cannot answer this question as it appears to require information not contained in the provided text."

        return answer