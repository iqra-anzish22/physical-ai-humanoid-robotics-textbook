from fastapi import HTTPException, status
from typing import Optional


class RAGException(Exception):
    """Base exception for RAG chatbot errors"""
    pass


class SessionNotFoundException(RAGException):
    """Raised when a session is not found"""
    pass


class TextProcessingException(RAGException):
    """Raised when text processing fails"""
    pass


class EmbeddingGenerationException(RAGException):
    """Raised when embedding generation fails"""
    pass


class VectorStorageException(RAGException):
    """Raised when vector storage operations fail"""
    pass


class QueryProcessingException(RAGException):
    """Raised when query processing fails"""
    pass


def handle_validation_error(detail: str):
    """Helper function to create validation errors"""
    raise HTTPException(
        status_code=status.HTTP_400_BAD_REQUEST,
        detail=detail
    )


def handle_not_found_error(detail: str = "Resource not found"):
    """Helper function to create not found errors"""
    raise HTTPException(
        status_code=status.HTTP_404_NOT_FOUND,
        detail=detail
    )


def handle_internal_error(detail: str = "Internal server error"):
    """Helper function to create internal server errors"""
    raise HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail=detail
    )