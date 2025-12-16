# Implementation Plan: AI Chatbot Integration for Existing Textbook

**Feature**: 001-chatbot-integration
**Created**: 2025-12-15
**Status**: Draft
**Author**: Claude

## Technical Context

This plan addresses the integration of an AI chatbot into an existing textbook frontend, connecting to a FastAPI backend. The system needs to handle session management properly and fix document processing failures. The architecture follows the constitutional requirements for content isolation and strict RAG flow.

**Architecture Overview**:
- Frontend: Embedded chatbot widget in textbook UI
- Communication: Frontend ↔ FastAPI backend (localhost:3000 ↔ 800x)
- Backend: FastAPI service with session management and RAG pipeline
- Vector Storage: Qdrant Cloud (session-scoped, temporary)
- LLM: Cohere embed-english-v3.0 and command-r-plus

**Key Technologies**:
- Frontend: HTML/CSS/JavaScript (lightweight embeddable widget)
- Backend: FastAPI
- Vector DB: Qdrant Cloud
- LLM: Cohere API
- Session Management: In-memory with temporary persistence

**Resolved unknowns**:
- Session storage method: In-memory storage with automatic cleanup for development
- Chat endpoint contract: RESTful API with JSON payloads (see contracts/api-spec.yaml)
- Fallback behavior: User-friendly error messages with option to retry or start new session
- Frontend integration method: Embedded widget that appears as a floating panel
- CORS scope: Localhost-only CORS for development

## Constitution Check

### Content Isolation Compliance
- ✅ All processing will be per-query with user-provided text only
- ✅ No pre-ingested book content will be accessed
- ✅ Responses will be generated only from user-pasted/selected text
- ✅ Zero hallucinations policy will be enforced

### Tech Stack Compliance
- ✅ Using FastAPI for backend (as required)
- ✅ Using Cohere for embeddings and LLM (as required)
- ✅ Using Qdrant Cloud for vector storage (as required)
- ✅ No OpenAI usage (as required)

### Privacy and Data Handling
- ✅ No user text persistence beyond session scope
- ✅ Temporary vector storage that gets deleted after session
- ✅ No logging of user-provided content
- ✅ Session-scoped processing only

### Performance Standards
- ✅ Target <5 seconds response time
- ✅ ≥95% grounded accuracy goal
- ✅ Session-based processing to maintain isolation

## Gates

### Gate 1: Architecture Feasibility
- [x] FastAPI backend can handle session management
- [x] Frontend can embed lightweight widget without layout changes
- [x] Qdrant Cloud integration is feasible for temporary storage
- [x] Cohere API integration is supported

### Gate 2: Constitutional Compliance
- [x] Content isolation requirements can be met
- [x] Tech stack constraints are adhered to
- [x] Privacy requirements can be satisfied
- [x] Performance standards are achievable

### Gate 3: Technical Dependencies
- [x] FastAPI can run on localhost port 800x
- [x] Frontend can communicate with backend on localhost
- [x] Required APIs (Cohere, Qdrant) are accessible
- [x] Session management can be implemented reliably

## Phase 0: Research & Unknown Resolution

### Research Tasks

#### Research Task 1: Session Storage Method
**Objective**: Determine optimal session storage approach for temporary, in-memory sessions
**Focus**: In-memory vs simple persistence for local development

#### Research Task 2: API Contract Design
**Objective**: Define clear request/response schema for chat endpoints
**Focus**: Session creation, message exchange, and error handling

#### Research Task 3: Fallback Behavior
**Objective**: Design graceful error handling for document processing failures
**Focus**: User-friendly error messages and recovery options

#### Research Task 4: Frontend Integration Method
**Objective**: Choose between embedded vs floating widget approach
**Focus**: Minimal UI impact while maintaining functionality

#### Research Task 5: CORS Configuration
**Objective**: Set up proper CORS for localhost development
**Focus**: Secure but functional communication between frontend and backend

### Expected Outcomes
- All [NEEDS CLARIFICATION] markers resolved
- Clear technical decisions with rationale
- Architecture sketches and API specifications
- Implementation approach validated

## Phase 1: Design & Contracts

### Data Model Design

#### Chat Session Entity
- **session_id**: Unique identifier for each session
- **created_at**: Timestamp when session was created
- **expires_at**: Timestamp when session expires
- **conversation_history**: Temporary storage of message pairs (user query, AI response)
- **vector_storage_ref**: Reference to temporary Qdrant collection for this session

#### User Query Entity
- **query_id**: Unique identifier for the query
- **session_id**: Reference to parent session
- **content**: The text content provided by user
- **timestamp**: When the query was made
- **processed_status**: Current state of processing

#### AI Response Entity
- **response_id**: Unique identifier for the response
- **query_id**: Reference to parent query
- **content**: The generated response text
- **timestamp**: When the response was generated
- **source_chunks**: References to the text chunks used to generate response

### API Contract Specifications

#### Session Management Endpoints

**POST /api/session/create**
- **Description**: Create a new chat session
- **Request**: `{}`
- **Response**:
  ```json
  {
    "session_id": "string",
    "created_at": "timestamp",
    "expires_at": "timestamp",
    "status": "active"
  }
  ```
- **Error Responses**:
  - 500: Internal server error

**GET /api/session/{session_id}**
- **Description**: Check session status
- **Request**: `{}`
- **Response**:
  ```json
  {
    "session_id": "string",
    "status": "active|expired",
    "expires_at": "timestamp"
  }
  ```

#### Chat Endpoints

**POST /api/chat/{session_id}**
- **Description**: Send a message to the chatbot within a session
- **Request**:
  ```json
  {
    "message": "string",
    "text_content": "string (user-provided text to analyze)"
  }
  ```
- **Response**:
  ```json
  {
    "response": "string",
    "session_id": "string",
    "message_id": "string"
  }
  ```
- **Error Responses**:
  - 404: Session not found
  - 410: Session expired
  - 500: Processing error

#### Document Processing Endpoints

**POST /api/process-text/{session_id}**
- **Description**: Process user-provided text for RAG
- **Request**:
  ```json
  {
    "text_content": "string",
    "chunk_size": "number (default: 600)"
  }
  ```
- **Response**:
  ```json
  {
    "status": "processed",
    "chunks_count": "number",
    "session_id": "string"
  }
  ```
- **Error Responses**:
  - 404: Session not found
  - 500: Processing failed

### Frontend Integration Design

#### Widget Implementation
- **Method**: Embedded div that appears in textbook pages
- **Position**: Floating panel that doesn't interfere with content
- **Trigger**: Button or icon in textbook UI
- **Style**: Minimal, consistent with textbook design

#### Communication Layer
- **Protocol**: REST API calls to backend
- **CORS**: Configured for localhost:3000 to localhost:800x
- **Error Handling**: Graceful fallbacks for connection issues
- **Session Management**: Automatic session creation and maintenance

### Quickstart Guide

#### Prerequisites
1. Python 3.8+ with pip
2. Node.js for frontend development
3. Cohere API key
4. Qdrant Cloud account and API key

#### Backend Setup
1. Install dependencies: `pip install fastapi uvicorn python-multipart cohere qdrant-client`
2. Set environment variables:
   ```bash
   export COHERE_API_KEY="your_key_here"
   export QDRANT_API_KEY="your_key_here"
   export QDRANT_URL="your_qdrant_url"
   ```
3. Run backend: `uvicorn main:app --host 0.0.0.0 --port 8000`

#### Frontend Integration
1. Add chatbot widget to textbook pages
2. Configure API endpoints to localhost:8000
3. Test session creation and chat functionality

## Phase 2: Implementation Approach

### Implementation Phases

#### Phase 2A: Foundation & Session Fix
1. Implement basic FastAPI backend with session management
2. Fix session-related errors ("No session active", etc.)
3. Create temporary session storage
4. Test session creation and validation endpoints

#### Phase 2B: Chat API & Core Functionality
1. Implement RAG pipeline with Cohere and Qdrant
2. Create chat endpoint with proper error handling
3. Implement document processing functionality
4. Test basic chat functionality

#### Phase 2C: Frontend Integration
1. Create lightweight chatbot widget
2. Integrate widget into textbook UI
3. Connect frontend to backend API
4. Implement error handling and fallbacks

#### Phase 2D: Validation & Testing
1. End-to-end testing of chat functionality
2. Session management validation
3. Error scenario testing
4. Performance validation (<5s response time)

## Architectural Decision Records (ADRs)

### ADR-001: Session Storage Approach
**Status**: Decided
**Decision**: In-memory storage with automatic cleanup for development
**Rationale**: For local development and testing, in-memory storage is sufficient, simple to implement, and aligns with the temporary nature of sessions. This approach avoids complexity of database management while meeting the requirement for temporary, session-scoped data that gets cleaned up automatically.

### ADR-002: Frontend Integration Method
**Status**: Decided
**Decision**: Embedded widget that appears as a floating panel
**Rationale**: A floating panel widget can be integrated without modifying existing textbook content or layout, meeting the requirement of not changing the book UI while providing the chat functionality. This approach provides a non-intrusive way to add chat capabilities.

### ADR-003: API Communication Pattern
**Status**: Decided
**Decision**: RESTful API with JSON payloads
**Rationale**: REST is standard, well-understood, and appropriate for the integration. JSON is the natural choice for web communication and works well with both frontend and backend technologies. This approach provides simplicity and broad compatibility.

### ADR-004: Error Handling Strategy
**Status**: Decided
**Decision**: User-friendly error messages with option to retry or start new session
**Rationale**: When document processing fails, users should receive clear, non-technical messages with actionable steps rather than seeing technical error details. This approach maintains good user experience while providing necessary information for troubleshooting.

## Risk Assessment

### High Risk Items
1. **Session Management**: Critical for maintaining conversation context
2. **CORS Configuration**: Required for frontend-backend communication
3. **API Rate Limits**: Could impact performance with Cohere/Qdrant

### Mitigation Strategies
1. **Session Management**: Implement robust error handling and validation
2. **CORS Configuration**: Start with localhost-only, expand as needed
3. **API Limits**: Implement caching and request optimization

## Success Criteria Validation

- [ ] Sessions are created automatically without errors
- [ ] Chat responses appear in textbook UI without layout changes
- [ ] Document processing failures are handled gracefully
- [ ] Frontend-backend communication works on localhost
- [ ] Response time is under 5 seconds
- [ ] No book content is pre-ingested or accessed inappropriately