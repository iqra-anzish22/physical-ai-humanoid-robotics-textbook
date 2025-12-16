from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional, Dict, Any
import uuid
import logging
from datetime import datetime, timedelta

from .config import settings
from .models.session import ChatSession, SessionStatus
from .models.message import UserQuery, QueryStatus
from .services.session_manager import SessionManager
from .services.rag_pipeline import RAGPipeline

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="Secure AI Chatbot API",
    description="API for secure AI chatbot with RAG functionality for textbook content",
    version="1.0.0"
)

# Add CORS middleware to allow frontend communication
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific frontend origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize services
session_manager = SessionManager()
rag_pipeline = RAGPipeline()

class CreateSessionRequest(BaseModel):
    pass  # No input required for session creation

class CreateSessionResponse(BaseModel):
    session_id: str

class ChatRequest(BaseModel):
    session_id: str
    message: str
    text_content: Optional[str] = None

class ChatResponse(BaseModel):
    answer: str

class ErrorResponse(BaseModel):
    error: str
    code: str
    details: Optional[Dict[str, Any]] = None

@app.post("/api/session/create", response_model=CreateSessionResponse)
async def create_session(request: CreateSessionRequest = None):
    """
    Create a new chat session
    """
    try:
        session_id = session_manager.create_session()
        logger.info(f"Session created: {session_id}")
        return CreateSessionResponse(session_id=session_id)
    except Exception as e:
        logger.error(f"Error creating session: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Send a message to the chatbot and get a response
    """
    try:
        # Validate session exists
        if not session_manager.session_exists(request.session_id):
            raise HTTPException(status_code=404, detail="Session not found")

        # Process the message through RAG pipeline
        response = await rag_pipeline.process_query(
            session_id=request.session_id,
            query=request.message,
            text_content=request.text_content
        )

        return ChatResponse(answer=response)
    except HTTPException:
        # Re-raise HTTP exceptions (like 404)
        raise
    except Exception as e:
        logger.error(f"Error processing chat request: {str(e)}")
        raise HTTPException(status_code=500, detail="Processing error")

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "timestamp": datetime.utcnow()}

# Error handlers
@app.exception_handler(404)
async def custom_http_exception_handler(request, exc):
    if exc.status_code == 404:
        # Log 404 errors to help identify missing endpoints
        logger.warning(f"404 error: {request.url.path} not found")
    return {"detail": "Not Found"}, exc.status_code

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)