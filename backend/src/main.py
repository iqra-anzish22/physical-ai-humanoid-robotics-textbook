from fastapi import FastAPI, HTTPException, BackgroundTasks, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
from typing import Dict, Any
import time
import logging
import asyncio
from .models.session import TextSession, SessionStatus
from .services.session_manager import SessionManager
from .services.text_processor import TextProcessor
from .services.embedding_service import EmbeddingService
from .services.vector_storage_service import VectorStorageService
from .services.retrieval_service import RetrievalService
from .services.generation_service import GenerationService
from .utils.performance_monitor import performance_monitor
from .utils.input_sanitizer import InputSanitizer
from .utils.input_validator import InputValidator, SessionRequest, QueryRequest
from .config import settings
from .api.knowledge_pipeline_api import router as knowledge_pipeline_router
from .api.chatbot_api import router as chatbot_router

# Initialize services
session_manager = SessionManager()
text_processor = TextProcessor()
embedding_service = EmbeddingService()
vector_storage_service = VectorStorageService()
retrieval_service = RetrievalService()
generation_service = GenerationService()

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)

# Create FastAPI app
app = FastAPI(
    title="Isolated On-Demand RAG Chatbot API",
    description="API for the isolated, privacy-first RAG chatbot that answers questions ONLY from user-provided text",
    version="1.0.0"
)

# Add rate limiter to app
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# Mount static files
app.mount("/static", StaticFiles(directory="src/static"), name="static")

# Add CORS middleware to allow embedding in other domains
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Allow frontend from localhost:3000
    allow_credentials=True,
    allow_methods=["*"],  # Allow all methods for development
    allow_headers=["*"],  # Allow all headers for development
)

# Include API routes
app.include_router(knowledge_pipeline_router)
app.include_router(chatbot_router)

# Add logging configuration
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# Custom exception handlers
@app.exception_handler(ValueError)
async def value_error_handler(request: Request, exc: ValueError):
    logger.error(f"ValueError: {type(exc).__name__}")
    return JSONResponse(
        status_code=422,
        content={"detail": str(exc)}
    )


@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    logger.error(f"Unhandled exception: {type(exc).__name__}")
    return JSONResponse(
        status_code=500,
        content={"detail": "An internal server error occurred"}
    )


@app.post("/api/v1/session")
@limiter.limit("5/minute")  # Limit to 5 requests per minute per IP
@performance_monitor.measure_time
async def create_session(request: Request, data: Dict[str, Any]):
    """
    Create a new session with user-provided text.
    """
    start_time = time.time()

    try:
        # Validate input using Pydantic model
        validated_request = InputValidator.validate_session_request(data)

        # Additional validation can be done here if needed
        text = validated_request.text
        chunk_size = validated_request.chunk_size
        overlap = validated_request.overlap

        # Create a new session
        session_id = session_manager.create_session()

        # Process the text into chunks
        chunks = text_processor.chunk_text(text, chunk_size, overlap)

        # Generate embeddings for the chunks
        chunk_embeddings = embedding_service.generate_embeddings_for_chunks(chunks)

        # Create storage for the session
        storage_created = vector_storage_service.create_session_storage(session_id)
        if not storage_created:
            raise HTTPException(status_code=503, detail="Vector storage service unavailable")

        # Store the chunk embeddings
        storage_success = vector_storage_service.store_text_chunks(session_id, chunk_embeddings)
        if not storage_success:
            raise HTTPException(status_code=500, detail="Failed to store text chunks")

        # Update session with chunk count
        session_manager.update_session_chunks(session_id, len(chunks))

        processing_time = time.time() - start_time

        logger.info(f"Session {session_id} created with {len(chunks)} chunks in {processing_time:.2f}s")

        return {
            "session_id": session_id,
            "chunk_count": len(chunks),
            "status": "completed",
            "processing_time": processing_time
        }

    except ValueError as e:
        logger.error(f"Validation error creating session: {str(e)}")
        raise HTTPException(status_code=422, detail=str(e))
    except HTTPException:
        # Re-raise HTTP exceptions to preserve their status codes
        raise
    except Exception as e:
        logger.error(f"Unexpected error creating session: {type(e).__name__}")
        raise HTTPException(status_code=500, detail="Failed to process text content")


@app.post("/api/v1/session/{session_id}/query")
@limiter.limit("20/minute")  # Limit to 20 queries per minute per IP
@performance_monitor.measure_time
async def submit_query(request: Request, session_id: str, data: Dict[str, Any]):
    """
    Submit a question for a specific session and get a response.
    """
    start_time = time.time()

    try:
        # Validate session ID format
        if not InputValidator.validate_session_id(session_id):
            raise HTTPException(status_code=400, detail="Invalid session ID format")

        # Validate input using Pydantic model
        validated_request = InputValidator.validate_query_request(data)
        question = validated_request.question

        # Check if session exists
        session = session_manager.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found or expired")

        # Retrieve relevant chunks for the question using the retrieval service
        relevant_chunks = retrieval_service.retrieve_with_reranking(
            session_id, question, top_k=5
        )

        if not relevant_chunks:
            raise HTTPException(status_code=503, detail="No relevant chunks found - service temporarily unavailable")

        # Generate response using the generation service
        response = generation_service.generate_answer_with_validation(question, relevant_chunks)

        processing_time = time.time() - start_time

        # Extract source chunk IDs
        source_chunk_ids = [chunk["chunk_id"] for chunk in relevant_chunks]

        logger.info(f"Query for session {session_id} processed in {processing_time:.2f}s")

        return {
            "response": response,
            "sources": source_chunk_ids,
            "processing_time": processing_time
        }

    except ValueError as e:
        logger.error(f"Validation error processing query for session {session_id}: {str(e)}")
        raise HTTPException(status_code=422, detail=str(e))
    except HTTPException:
        # Re-raise HTTP exceptions to preserve their status codes
        raise
    except Exception as e:
        logger.error(f"Unexpected error processing query for session {session_id}: {type(e).__name__}")
        raise HTTPException(status_code=500, detail="Failed to process query")


@app.delete("/api/v1/session/{session_id}")
@limiter.limit("10/minute")  # Limit to 10 deletions per minute per IP
@performance_monitor.measure_time
async def delete_session(request: Request, session_id: str, background_tasks: BackgroundTasks):
    """
    Explicitly delete a session and its data.
    """
    try:
        # Validate session ID format
        if not InputValidator.validate_session_id(session_id):
            raise HTTPException(status_code=400, detail="Invalid session ID format")

        # Check if session exists
        session = session_manager.get_session(session_id)
        if not session:
            raise HTTPException(status_code=404, detail="Session not found or already deleted")

        # Delete session from session manager
        session_deleted = session_manager.delete_session(session_id)

        # Delete vector storage in background to avoid blocking the response
        background_tasks.add_task(vector_storage_service.cleanup_session_storage, session_id)

        if session_deleted:
            logger.info(f"Session {session_id} marked for deletion")
            return {"status": "Session and associated data marked for deletion"}
        else:
            raise HTTPException(status_code=500, detail="Failed to delete session from memory")

    except HTTPException:
        # Re-raise HTTP exceptions to preserve their status codes
        raise
    except Exception as e:
        logger.error(f"Unexpected error deleting session {session_id}: {type(e).__name__}")
        raise HTTPException(status_code=500, detail="Failed to delete session")


@app.get("/")
async def read_root():
    """
    Root endpoint that returns a success message.
    """
    return {"message": "Backend is running!"}


@app.get("/embed")
async def get_embed_instructions():
    """
    Serve the embed instructions page.
    """
    with open("src/static/embed.html", "r") as file:
        content = file.read()
    return HTMLResponse(content=content)


@app.get("/chat")
async def get_chat_interface():
    """
    Serve the chat interface.
    """
    chat_html = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>RAG Chatbot</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }
        .chat-container {
            max-width: 800px;
            margin: 0 auto;
            background: white;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            height: 80vh;
            display: flex;
            flex-direction: column;
        }
        .chat-header {
            background: #4f46e5;
            color: white;
            padding: 15px;
            border-top-left-radius: 8px;
            border-top-right-radius: 8px;
        }
        .chat-messages {
            flex: 1;
            padding: 20px;
            overflow-y: auto;
            display: flex;
            flex-direction: column;
            gap: 15px;
        }
        .message {
            max-width: 80%;
            padding: 10px 15px;
            border-radius: 18px;
            margin-bottom: 10px;
        }
        .user-message {
            align-self: flex-end;
            background: #4f46e5;
            color: white;
        }
        .bot-message {
            align-self: flex-start;
            background: #e5e7eb;
            color: #374151;
        }
        .input-container {
            display: flex;
            padding: 20px;
            border-top: 1px solid #e5e7eb;
        }
        #user-input {
            flex: 1;
            padding: 10px;
            border: 1px solid #d1d5db;
            border-radius: 4px;
            margin-right: 10px;
        }
        #send-btn {
            padding: 10px 20px;
            background: #4f46e5;
            color: white;
            border: none;
            border-radius: 4px;
            cursor: pointer;
        }
        #send-btn:disabled {
            background: #9ca3af;
            cursor: not-allowed;
        }
        .session-controls {
            padding: 10px 20px;
            background: #f9fafb;
            border-bottom: 1px solid #e5e7eb;
        }
        .session-info {
            font-size: 12px;
            color: #6b7280;
        }
        .loading {
            font-style: italic;
            color: #6b7280;
        }
    </style>
</head>
<body>
    <div class="chat-container">
        <div class="chat-header">
            <h2>AI Chatbot</h2>
        </div>
        <div class="session-controls">
            <div class="session-info" id="session-info">No session active</div>
            <button id="new-session-btn" style="margin-top: 5px; padding: 5px 10px; background: #10b981; color: white; border: none; border-radius: 4px; cursor: pointer;">New Session</button>
        </div>
        <div class="chat-messages" id="chat-messages">
            <div class="message bot-message">
                Hello! I'm your AI assistant. Please create a new session to start chatting about your documents.
            </div>
        </div>
        <div class="input-container">
            <input type="text" id="user-input" placeholder="Type your question here..." autocomplete="off">
            <button id="send-btn">Send</button>
        </div>
    </div>

    <script>
        let currentSessionId = null;
        let isProcessing = false;

        document.addEventListener('DOMContentLoaded', function() {
            const chatMessages = document.getElementById('chat-messages');
            const userInput = document.getElementById('user-input');
            const sendBtn = document.getElementById('send-btn');
            const newSessionBtn = document.getElementById('new-session-btn');
            const sessionInfo = document.getElementById('session-info');

            // Create a new session
            async function createNewSession() {
                try {
                    // For now, we'll use a simple text to create a session
                    // In a real implementation, you might upload documents
                    const response = await fetch('/api/v1/session', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({
                            text: "This is a default session for the chatbot. You can ask questions about AI, robotics, or any topic you'd like to discuss.",
                            chunk_size: 600,
                            overlap: 100
                        })
                    });

                    if (response.ok) {
                        const data = await response.json();
                        currentSessionId = data.session_id;
                        sessionInfo.textContent = `Session: ${currentSessionId.substring(0, 8)}...`;
                        addMessage('System', `New session created: ${currentSessionId.substring(0, 8)}...`);
                    } else {
                        const error = await response.json();
                        addMessage('System', `Error creating session: ${error.detail || 'Unknown error'}`);
                    }
                } catch (error) {
                    addMessage('System', `Error creating session: ${error.message}`);
                }
            }

            // Send a message
            async function sendMessage() {
                const message = userInput.value.trim();
                if (!message || isProcessing) return;

                if (!currentSessionId) {
                    addMessage('System', 'Please create a new session first!');
                    return;
                }

                // Add user message to chat
                addMessage('You', message);
                userInput.value = '';
                isProcessing = true;
                sendBtn.disabled = true;

                try {
                    // Show loading message
                    const loadingMsg = addMessage('AI', 'Thinking...', true);

                    const response = await fetch(`/api/v1/session/${currentSessionId}/query`, {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({
                            question: message
                        })
                    });

                    // Remove loading message
                    chatMessages.removeChild(loadingMsg);

                    if (response.ok) {
                        const data = await response.json();
                        addMessage('AI', data.response);
                    } else {
                        const error = await response.json();
                        addMessage('AI', `Error: ${error.detail || 'Unknown error'}`);
                    }
                } catch (error) {
                    addMessage('AI', `Error: ${error.message}`);
                } finally {
                    isProcessing = false;
                    sendBtn.disabled = false;
                }
            }

            // Add message to chat
            function addMessage(sender, text, isTemporary = false) {
                const messageDiv = document.createElement('div');
                messageDiv.className = `message ${sender === 'You' ? 'user-message' : 'bot-message'}`;

                if (sender === 'AI' && isTemporary) {
                    messageDiv.className += ' loading';
                }

                messageDiv.innerHTML = `<strong>${sender}:</strong> ${text}`;
                chatMessages.appendChild(messageDiv);

                // Scroll to bottom
                chatMessages.scrollTop = chatMessages.scrollHeight;

                if (isTemporary) {
                    return messageDiv;
                }
            }

            // Event listeners
            sendBtn.addEventListener('click', sendMessage);
            newSessionBtn.addEventListener('click', createNewSession);

            userInput.addEventListener('keypress', function(e) {
                if (e.key === 'Enter') {
                    sendMessage();
                }
            });

            // Create initial session
            createNewSession();
        });
    </script>
</body>
</html>
    """
    return HTMLResponse(content=chat_html)


@app.post("/chat")
async def chat_endpoint(request: Request):
    """
    Chat endpoint that accepts a question and returns an answer.
    This is a placeholder endpoint that returns a simple response.
    In a real implementation, this would connect to your RAG system.
    """
    try:
        data = await request.json()
        question = data.get("question", "")

        if not question:
            return {"error": "Question is required"}

        # Placeholder AI response logic
        # In a real implementation, this would connect to your RAG system
        response = f"I received your question: '{question}'. This is a placeholder response. In a real implementation, this would connect to your RAG system to provide an answer based on your book content."

        return {"question": question, "answer": response}
    except Exception as e:
        return {"error": str(e)}


@app.get("/health")
@limiter.limit("60/minute")  # Limit to 60 health checks per minute per IP
@performance_monitor.measure_time
async def health_check(request: Request):
    """
    Health check endpoint.
    """
    return {"status": "healthy", "timestamp": time.time()}


# Background task for automatic session cleanup
async def cleanup_expired_sessions():
    """Background task to periodically clean up expired sessions."""
    while True:
        try:
            # Clean up expired sessions in memory
            session_manager.cleanup_expired_sessions()

            # Sleep for 5 minutes before next cleanup
            await asyncio.sleep(300)
        except Exception as e:
            logging.error(f"Error during session cleanup: {str(e)}")


# Create __init__.py files to make directories proper Python packages
@app.on_event('startup')
def startup_event():
    import os
    # Ensure __init__.py files exist for proper Python package structure
    init_files = [
        "src/__init__.py",
        "src/services/__init__.py",
        "src/models/__init__.py",
        "src/utils/__init__.py"
    ]

    for init_file in init_files:
        full_path = os.path.join(os.path.dirname(__file__), "..", init_file)
        if not os.path.exists(full_path):
            with open(full_path, 'w') as f:
                f.write("# Generated by FastAPI app startup\n")

    # Start the background cleanup task
    asyncio.create_task(cleanup_expired_sessions())


@app.on_event('shutdown')
def shutdown_event():
    """Cleanup on app shutdown."""
    logging.info("Shutting down - cleaning up sessions...")
    # Perform final cleanup
    session_manager.cleanup_expired_sessions()