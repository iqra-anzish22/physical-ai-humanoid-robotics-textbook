from fastapi import APIRouter, HTTPException, Request
from typing import Dict, Any, Optional
import logging
from datetime import datetime

from src.services.session_manager import SessionManager
from src.services.text_processor import TextProcessor
from src.services.embedding_service import EmbeddingService
from src.services.vector_storage_service import VectorStorageService
from src.services.generation_service import GenerationService
from src.services.retrieval_service import RetrievalService
from src.utils.input_validator import InputValidator
from src.utils.logger import logger
from src.models.session import SessionStatus

# Initialize router
router = APIRouter(prefix="/api", tags=["chatbot"])

# Initialize services
session_manager = SessionManager()
text_processor = TextProcessor()
embedding_service = EmbeddingService()
vector_storage_service = VectorStorageService()
retrieval_service = RetrievalService()
generation_service = GenerationService()

@router.post("/session/create")
async def create_session():
    """
    Create a new chat session.
    """
    try:
        # Create a new session
        session_id = session_manager.create_session()
        session = session_manager.get_session(session_id)

        if not session:
            raise HTTPException(status_code=500, detail="Failed to create session")

        return {
            "session_id": session.session_id,
            "created_at": session.created_at.isoformat(),
            "expires_at": session.expires_at.isoformat(),
            "status": "active"
        }
    except Exception as e:
        logger.error(f"Error creating session: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/session/{session_id}")
async def get_session(session_id: str):
    """
    Check session status.
    """
    try:
        # Validate session ID format
        if not InputValidator.validate_session_id(session_id):
            raise HTTPException(status_code=400, detail="Invalid session ID format")

        session = session_manager.get_session(session_id)

        if not session:
            raise HTTPException(status_code=404, detail="Session not found")

        return {
            "session_id": session.session_id,
            "status": session.status,
            "expires_at": session.expires_at.isoformat(),
            "conversation_history": session.get_conversation_history()
        }
    except HTTPException:
        # Re-raise HTTP exceptions to preserve their status codes
        raise
    except Exception as e:
        logger.error(f"Error getting session {session_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.post("/chat/{session_id}")
async def chat_with_bot(session_id: str, request: Request):
    """
    Send a message to the chatbot within a session.
    """
    try:
        # Validate session ID format
        if not InputValidator.validate_session_id(session_id):
            raise HTTPException(status_code=400, detail="Invalid session ID format")

        # Parse request data
        data = await request.json()
        message = data.get("message", "")
        text_content = data.get("text_content", "")

        # Validate inputs
        if not message:
            raise HTTPException(status_code=400, detail="Message is required")

        # Check if session exists
        session = session_manager.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found or expired")

        # If text_content is provided, process it for this session
        if text_content:
            # Process the text into chunks
            chunks = text_processor.chunk_text(text_content, chunk_size=600, overlap=100)

            # Generate embeddings for the chunks
            chunk_embeddings = embedding_service.generate_embeddings_for_chunks(chunks)

            # Store the chunk embeddings
            storage_success = vector_storage_service.store_text_chunks(session_id, chunk_embeddings)
            if not storage_success:
                raise HTTPException(status_code=500, detail="Failed to store text chunks")

            # Update session with chunk count
            session_manager.update_session_chunks(session_id, len(chunks))

        # Retrieve relevant chunks for the question using the retrieval service
        relevant_chunks = retrieval_service.retrieve_with_reranking(
            session_id, message, top_k=5
        )

        if not relevant_chunks:
            # If no chunks found, we can still generate a response based on the message
            response = generation_service.generate_answer_with_validation(message, [])
        else:
            # Generate response using the generation service
            response = generation_service.generate_answer_with_validation(message, relevant_chunks)

        # Add the conversation to session history
        session_obj = session_manager.get_session(session_id)
        if session_obj:
            session_obj.add_message_to_history("user", message)
            session_obj.add_message_to_history("assistant", response)

        return {
            "response": response,
            "session_id": session_id,
            "message_id": f"msg_{int(datetime.now().timestamp())}"
        }

    except ValueError as e:
        logger.error(f"Validation error processing chat for session {session_id}: {str(e)}")
        raise HTTPException(status_code=422, detail=str(e))
    except HTTPException:
        # Re-raise HTTP exceptions to preserve their status codes
        raise
    except Exception as e:
        logger.error(f"Unexpected error processing chat for session {session_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to process query")


@router.post("/process-text/{session_id}")
async def process_text_for_session(session_id: str, request: Request):
    """
    Process user-provided text for RAG within a session.
    """
    try:
        # Validate session ID format
        if not InputValidator.validate_session_id(session_id):
            raise HTTPException(status_code=400, detail="Invalid session ID format")

        # Parse request data
        data = await request.json()
        text_content = data.get("text_content", "")
        chunk_size = data.get("chunk_size", 600)

        # Validate inputs
        if not text_content:
            raise HTTPException(status_code=400, detail="Text content is required")

        # Check if session exists
        session = session_manager.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found or expired")

        # Process the text into chunks
        chunks = text_processor.chunk_text(text_content, chunk_size, 100)  # Default overlap of 100

        # Generate embeddings for the chunks
        chunk_embeddings = embedding_service.generate_embeddings_for_chunks(chunks)

        # Create storage for the session if it doesn't exist
        storage_created = vector_storage_service.create_session_storage(session_id)
        if not storage_created:
            raise HTTPException(status_code=503, detail="Vector storage service unavailable")

        # Store the chunk embeddings
        storage_success = vector_storage_service.store_text_chunks(session_id, chunk_embeddings)
        if not storage_success:
            raise HTTPException(status_code=500, detail="Failed to store text chunks")

        # Update session with chunk count
        session_manager.update_session_chunks(session_id, len(chunks))

        return {
            "status": "processed",
            "chunks_count": len(chunks),
            "session_id": session_id
        }

    except ValueError as e:
        logger.error(f"Validation error processing text for session {session_id}: {str(e)}")
        raise HTTPException(status_code=422, detail=str(e))
    except HTTPException:
        # Re-raise HTTP exceptions to preserve their status codes
        raise
    except Exception as e:
        logger.error(f"Unexpected error processing text for session {session_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Failed to process text content")