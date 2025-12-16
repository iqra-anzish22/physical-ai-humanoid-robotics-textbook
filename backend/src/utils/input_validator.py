from typing import Any, Dict, List, Optional, Union
import re
import uuid
from pydantic import BaseModel, Field, validator
from enum import Enum


class SessionRequest(BaseModel):
    """Model for session creation request with validation."""
    text: str = Field(..., min_length=10, max_length=100000, description="Text content to process")
    chunk_size: Optional[int] = Field(default=600, ge=500, le=800, description="Chunk size in tokens")
    overlap: Optional[int] = Field(default=100, ge=0, le=200, description="Overlap in tokens")

    @validator('text')
    def validate_text_content(cls, v):
        if not v or len(v.strip()) < 10:
            raise ValueError('Text must be at least 10 characters long')
        if not re.search(r'[a-zA-Z0-9]', v):
            raise ValueError('Text must contain alphanumeric characters')
        return v.strip()


class QueryRequest(BaseModel):
    """Model for query request with validation."""
    question: str = Field(..., min_length=1, max_length=1000, description="Question to ask")

    @validator('question')
    def validate_question(cls, v):
        if not v or len(v.strip()) < 1:
            raise ValueError('Question cannot be empty')
        if len(v.strip()) > 1000:
            raise ValueError('Question is too long (max 1000 characters)')
        return v.strip()


class SessionIdValidator:
    """Utility class for validating session IDs."""

    @staticmethod
    def is_valid_session_id(session_id: str) -> bool:
        """
        Validate session ID format.

        Args:
            session_id: Session ID to validate

        Returns:
            True if valid, False otherwise
        """
        if not isinstance(session_id, str):
            return False

        # Check if it's a valid UUID format
        try:
            uuid.UUID(session_id)
            return True
        except ValueError:
            # If not UUID, check for a reasonable alphanumeric format
            return bool(re.match(r'^[a-zA-Z0-9\-_]{10,100}$', session_id))


class InputValidator:
    """Comprehensive input validation utility."""

    @staticmethod
    def validate_session_request(data: Dict[str, Any]) -> SessionRequest:
        """
        Validate session creation request data.

        Args:
            data: Request data to validate

        Returns:
            Validated SessionRequest object
        """
        try:
            return SessionRequest(**data)
        except Exception as e:
            raise ValueError(f"Invalid session request data: {str(e)}")

    @staticmethod
    def validate_query_request(data: Dict[str, Any]) -> QueryRequest:
        """
        Validate query request data.

        Args:
            data: Request data to validate

        Returns:
            Validated QueryRequest object
        """
        try:
            return QueryRequest(**data)
        except Exception as e:
            raise ValueError(f"Invalid query request data: {str(e)}")

    @staticmethod
    def validate_session_id(session_id: str) -> bool:
        """
        Validate session ID format.

        Args:
            session_id: Session ID to validate

        Returns:
            True if valid, False otherwise
        """
        return SessionIdValidator.is_valid_session_id(session_id)

    @staticmethod
    def sanitize_text(text: str) -> str:
        """
        Sanitize text input to prevent injection attacks.

        Args:
            text: Text to sanitize

        Returns:
            Sanitized text
        """
        if not isinstance(text, str):
            return ""

        # Remove control characters except common whitespace
        sanitized = re.sub(r'[\x00-\x08\x0B\x0C\x0E-\x1F\x7F]', '', text)

        # Limit length to prevent buffer overflow
        if len(sanitized) > 100000:  # 100KB limit
            sanitized = sanitized[:100000]

        return sanitized.strip()

    @staticmethod
    def validate_chunk_parameters(chunk_size: int, overlap: int) -> tuple[bool, str]:
        """
        Validate chunk size and overlap parameters.

        Args:
            chunk_size: Size of chunks
            overlap: Overlap between chunks

        Returns:
            Tuple of (is_valid, error_message)
        """
        if chunk_size < 500 or chunk_size > 800:
            return False, "Chunk size must be between 500 and 800 tokens"

        if overlap < 0 or overlap > 200:
            return False, "Overlap must be between 0 and 200 tokens"

        if overlap >= chunk_size:
            return False, "Overlap must be smaller than chunk size"

        return True, ""