# Secure Environment Setup + AI Chatbot Integration

This directory contains the implementation of a secure AI chatbot with RAG functionality as specified in the feature requirements. The implementation includes secure environment variable loading, API endpoints, RAG pipeline, and frontend integration.

## Implementation Overview

The system implements all requirements from the specification:

### 🔐 Secure Environment Setup
- **Environment Configuration**: Uses Pydantic BaseSettings for secure environment variable loading
- **Required Variables**: `QDRANT_API_KEY`, `QDRANT_URL`, `COHERE_API_KEY`, `NEON_DATABASE_URL` (optional)
- **Fail-Fast Mechanism**: Validates all required variables on startup and fails with clear error messages if any are missing
- **Secure Logging**: Only logs variable names (not values) for debugging

### 🧩 Backend APIs
- **Session Creation**: `POST /api/session/create` returns `{ session_id }`
- **Chat Endpoint**: `POST /api/chat` returns `{ answer }`
- **No 404 Errors**: All endpoints properly implemented and accessible
- **Session Management**: In-memory session storage with expiration

### 🤖 RAG Pipeline
- **Content Ingestion**: Accepts user-provided text content only (no pre-ingested content)
- **Chunking & Embedding**: Text chunking with overlap, Cohere embeddings
- **Vector Storage**: Qdrant Cloud for temporary session-scoped storage
- **Context Retrieval**: Similarity search to find relevant content
- **Answer Generation**: Cohere LLM generates responses based only on retrieved context
- **Content Isolation**: Each session has isolated vector storage

### 💬 Frontend Chatbot UI
- **Floating Widget**: `AI Assistant ▼` button in bottom-right corner
- **Toggle Functionality**: Open/close chat interface
- **Session Management**: Automatically creates session on first open
- **Message Flow**: Sends messages to backend, displays responses
- **Loading States**: Proper UI feedback during processing
- **Error Handling**: Graceful fallbacks for connection issues

## File Structure

```
backend/
├── config.py                 # Secure environment configuration with BaseSettings
├── main.py                   # FastAPI application with CORS middleware
├── start_server.py           # Startup script with environment validation
├── test_api.py               # API endpoint testing script
├── chatbot_widget.html       # Frontend floating chatbot widget
├── models/                   # Data models (session, message)
│   ├── session.py
│   └── message.py
└── services/                 # Business logic services
    ├── session_manager.py    # Session management
    └── rag_pipeline.py       # RAG pipeline implementation
```

## Security Compliance

✅ **No Hard-Coded Secrets**: All API keys loaded from environment variables only
✅ **Environment Validation**: Fails fast if required variables are missing
✅ **Secure Logging**: No sensitive data exposed in logs
✅ **Content Isolation**: Session-scoped vector storage
✅ **Temporary Data**: No persistent storage of user content beyond session

## API Contract Compliance

- `POST /api/session/create` → `{ session_id: string }`
- `POST /api/chat` → `{ answer: string }`
- Proper error handling (404 for missing sessions, 500 for processing errors)
- No 404 errors for valid endpoints

## Frontend Integration

The chatbot widget (`chatbot_widget.html`) can be embedded in any webpage. It features:
- Floating toggle button that doesn't modify existing layout
- Session creation on first open
- Text content input area for pasting textbook content
- Real-time chat interface with user and AI messages
- Loading indicators and error handling

## Setup Instructions

1. **Install dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

2. **Create .env file** with required variables:
   ```env
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=your_qdrant_url
   COHERE_API_KEY=your_cohere_api_key
   NEON_DATABASE_URL=your_neon_db_url  # Optional
   ```

3. **Start the server**:
   ```bash
   python start_server.py
   ```

4. **Access the chatbot** by opening `chatbot_widget.html` in a browser

## Testing

Run the API tests to verify all endpoints are working:
```bash
python test_api.py
```

This will test:
- Health endpoint
- Session creation
- Chat functionality
- 404 error handling

## Constitutional Compliance

The implementation fully complies with all constitutional requirements:
- ✅ No pre-ingested content access
- ✅ Responses only from user-provided text
- ✅ Zero hallucinations policy
- ✅ Temporary storage that gets deleted after session
- ✅ No user text persistence beyond session scope
- ✅ No logging of user-provided content
- ✅ Uses required tech stack (FastAPI, Cohere, Qdrant)
- ✅ No OpenAI usage