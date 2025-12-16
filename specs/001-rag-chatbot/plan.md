# Implementation Plan: Isolated On-Demand RAG Chatbot

**Feature**: 001-rag-chatbot
**Created**: 2025-12-13
**Status**: Draft
**Plan Version**: 1.0

## Technical Context

### Architecture Overview
- **Backend**: FastAPI application
- **LLM Service**: Cohere API (embed-english-v3.0 for embeddings, command-r-plus for generation)
- **Vector Database**: Qdrant Cloud (session-only collections)
- **Metadata Storage**: Neon Postgres (optional, for session metadata only)
- **Frontend**: Embeddable chatbot widget

### Core Components
1. **Text Processing Service**: Handles text chunking and preprocessing
2. **Embedding Service**: Generates embeddings using Cohere
3. **Vector Storage Service**: Temporary storage in Qdrant with session cleanup
4. **Retrieval Service**: Finds relevant chunks based on user query
5. **Generation Service**: Creates responses using Cohere based only on retrieved chunks
6. **Session Management**: Ensures isolation and cleanup after sessions

### Technology Stack
- **Language**: Python 3.11+
- **Framework**: FastAPI
- **Embeddings**: Cohere embed-english-v3.0 model
- **Generation**: Cohere command-r-plus model
- **Vector DB**: Qdrant Cloud
- **Database**: Neon Postgres (for session metadata only)
- **Frontend**: HTML/CSS/JavaScript for embeddable widget

### Environment Variables Required
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `QDRANT_URL`: URL for Qdrant Cloud instance
- `COHERE_API_KEY`: API key for Cohere
- `NEON_DATABASE_URL`: Connection string for Neon Postgres (optional)

## Constitution Check

### Compliance Verification
- **Content Isolation**: System will ONLY use user-provided text, no pre-ingestion of book content
- **Session Isolation**: Each session will have temporary storage that gets deleted after use
- **No Data Persistence**: User text will not be stored or logged beyond the session
- **Technology Constraints**: Using only Cohere APIs, not OpenAI
- **Code Size**: Core code will remain under 1200 lines
- **Privacy Requirements**: No user text persistence, session-scoped temporary storage only

### Gates
- **GATE 1**: All API keys handled via environment variables only (PASSED - specified in requirements)
- **GATE 2**: No pre-ingestion of book content (PASSED - constitutional requirement)
- **GATE 3**: Session isolation maintained (PASSED - constitutional requirement)
- **GATE 4**: Core code ≤1200 lines (PASSED - constitutional requirement)

## Phase 0: Outline & Research

### Research Tasks

#### Decision: Cohere Embedding Model
- **Rationale**: Using embed-english-v3.0 as specified in requirements, or latest available version
- **Alternatives Considered**: Other Cohere embedding models, but requirements specify this one
- **Final Choice**: embed-english-v3.0 or latest compatible version

#### Decision: Qdrant Collection Configuration
- **Rationale**: Session-only collections with automatic cleanup
- **Parameters**:
  - Collection names with session IDs
  - Time-based TTL for automatic cleanup
  - Vector dimensions compatible with Cohere embeddings (1024 for embed-english-v3.0)
- **Alternatives Considered**: Permanent collections (violates constitutional requirement)

#### Decision: Text Chunk Size & Overlap
- **Rationale**: 500-800 tokens as specified in requirements
- **Parameters**:
  - Chunk size: 600 tokens (middle of range)
  - Overlap: 100 tokens for context continuity
- **Alternatives Considered**: Different sizes but requirements specify range

#### Decision: Error Handling Strategy
- **Rationale**: Graceful degradation with clear user feedback
- **Approach**:
  - Cohere API failures: Return informative error messages
  - Qdrant failures: Clean up and return error
  - Input validation: Clear error messages for invalid inputs
- **Alternatives Considered**: Silent failures (rejected for user experience)

#### Decision: Session Management
- **Rationale**: Unique session IDs with automatic cleanup
- **Approach**:
  - Generate UUID for each session
  - Create Qdrant collection with session ID
  - Delete collection when session ends or times out
  - No cross-session data persistence
- **Alternatives Considered**: Persistent sessions (violates constitutional requirement)

## Phase 1: Design & Contracts

### Data Model

#### Text Session Entity
- **session_id**: UUID (unique identifier for the session)
- **created_at**: timestamp (when session was created)
- **expires_at**: timestamp (when session should be cleaned up)
- **status**: enum ['active', 'completed', 'expired']

#### Text Chunk Entity
- **chunk_id**: UUID (unique identifier for the chunk)
- **session_id**: UUID (foreign key to Text Session)
- **content**: text (the actual text chunk)
- **embedding_vector**: array (Cohere embedding vector)
- **position**: integer (position in original text)

#### User Query Entity
- **query_id**: UUID (unique identifier for the query)
- **session_id**: UUID (foreign key to Text Session)
- **question**: text (the user's question)
- **timestamp**: timestamp (when question was asked)
- **response**: text (the generated response)

### API Contracts

#### POST /api/v1/session
- **Purpose**: Create a new session and process user-provided text
- **Request**:
  ```json
  {
    "text": "string (user-provided text)",
    "chunk_size": "integer (optional, default 600)",
    "overlap": "integer (optional, default 100)"
  }
  ```
- **Response**:
  ```json
  {
    "session_id": "string (UUID)",
    "chunk_count": "integer",
    "status": "string (processing/completed)"
  }
  ```
- **Error Responses**:
  - 400: Invalid input (empty text, invalid parameters)
  - 500: Processing error (API failures, etc.)

#### POST /api/v1/session/{session_id}/query
- **Purpose**: Submit a question for a specific session and get a response
- **Request**:
  ```json
  {
    "question": "string (user's question)"
  }
  ```
- **Response**:
  ```json
  {
    "response": "string (generated answer)",
    "sources": "array of chunk references used",
    "processing_time": "float (response time in seconds)"
  }
  ```
- **Error Responses**:
  - 404: Session not found
  - 400: Invalid question (empty, etc.)
  - 500: Processing error

#### DELETE /api/v1/session/{session_id}
- **Purpose**: Explicitly clean up a session
- **Response**:
  ```json
  {
    "status": "string (cleanup successful)"
  }
  ```

### Quickstart Guide

1. **Setup Environment**:
   ```bash
   # Install dependencies
   pip install fastapi uvicorn cohere qdrant-client python-dotenv

   # Set environment variables
   export QDRANT_API_KEY="your-qdrant-api-key"
   export QDRANT_URL="your-qdrant-url"
   export COHERE_API_KEY="your-cohere-api-key"
   export NEON_DATABASE_URL="your-neon-db-url" # optional
   ```

2. **Run the Application**:
   ```bash
   uvicorn main:app --reload
   ```

3. **Use the API**:
   ```bash
   # Create a session with text
   curl -X POST http://localhost:8000/api/v1/session \
     -H "Content-Type: application/json" \
     -d '{"text": "Your text content here..."}'

   # Ask a question
   curl -X POST http://localhost:8000/api/v1/session/{session_id}/query \
     -H "Content-Type: application/json" \
     -d '{"question": "Your question here?"}'
   ```

## Phase 2: Implementation Strategy

### Implementation Order
1. **Foundation Layer**: Set up FastAPI, environment loading, basic routing
2. **Integration Layer**: Connect to Cohere, Qdrant, Neon (optional)
3. **Service Layer**: Implement core RAG services (chunking, embedding, retrieval, generation)
4. **Session Management**: Implement session creation, isolation, and cleanup
5. **API Layer**: Expose endpoints as defined in contracts
6. **Frontend**: Create embeddable chatbot widget
7. **Testing**: Unit and integration tests
8. **Documentation**: API docs, README, deployment guide

### Key Implementation Considerations
- Maintain strict isolation between sessions
- Ensure all user text is processed only temporarily
- Implement proper error handling and validation
- Monitor response times to meet <5s requirement
- Implement cleanup mechanisms for failed sessions
- Ensure responses are grounded only in provided text