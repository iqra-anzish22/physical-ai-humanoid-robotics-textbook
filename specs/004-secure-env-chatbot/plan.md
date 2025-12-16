# Implementation Plan: Secure Environment Setup + AI Chatbot Integration

**Feature**: 004-secure-env-chatbot
**Created**: 2025-12-15
**Status**: Draft
**Author**: Claude

## Technical Context

This plan addresses the implementation of a secure Docusaurus AI chatbot with RAG pipeline. The system must load environment variables securely from .env file using BaseSettings, implement backend APIs (session creation and chat), create a RAG pipeline for textbook content, and provide a frontend chatbot UI. The architecture follows constitutional requirements for content isolation and strict RAG flow.

**Architecture Overview**:
- Frontend: Floating chatbot widget in Docusaurus textbook
- Communication: Frontend ↔ FastAPI backend
- Backend: FastAPI service with secure environment configuration
- Vector Storage: Qdrant Cloud (session-scoped, temporary)
- LLM: Cohere API
- Session Management: In-memory with temporary persistence

**Key Technologies**:
- Frontend: HTML/CSS/JavaScript (lightweight embeddable widget)
- Backend: FastAPI with Pydantic BaseSettings for env management
- Vector DB: Qdrant Cloud
- LLM: Cohere API
- Database: Neon Postgres (optional metadata)

**Resolved unknowns**:
- Environment variable loading: Using Pydantic BaseSettings with validation
- API endpoints: POST /api/session/create and POST /api/chat as specified
- RAG pipeline: Ingestion, chunking, embedding, retrieval, and answering flow
- Frontend integration: Floating widget approach similar to previous implementation

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
- [x] FastAPI backend can handle secure environment configuration
- [x] Frontend can embed lightweight widget without layout changes
- [x] Qdrant Cloud integration is feasible for temporary storage
- [x] Cohere API integration is supported

### Gate 2: Constitutional Compliance
- [x] Content isolation requirements can be met
- [x] Tech stack constraints are adhered to
- [x] Privacy requirements can be satisfied
- [x] Performance standards are achievable

### Gate 3: Technical Dependencies
- [x] Required APIs (Cohere, Qdrant) are accessible
- [x] Secure environment loading with BaseSettings is supported
- [x] Session management can be implemented reliably

## Phase 0: Research & Unknown Resolution

### Research Tasks

#### Research Task 1: Secure Environment Loading
**Objective**: Implement Pydantic BaseSettings for secure environment variable loading
**Focus**: Proper validation and fail-fast mechanism for missing variables

#### Research Task 2: API Contract Design
**Objective**: Define clear API contracts for session and chat endpoints
**Focus**: Proper error handling and response structures

#### Research Task 3: RAG Pipeline Implementation
**Objective**: Design RAG pipeline for textbook content processing
**Focus**: Content ingestion, chunking, and retrieval patterns

#### Research Task 4: Frontend Integration Method
**Objective**: Choose between embedded vs floating widget approach
**Focus**: Minimal UI impact while maintaining functionality

#### Research Task 5: Security Best Practices
**Objective**: Ensure no secrets are exposed in code or logs
**Focus**: Proper logging and error handling practices

### Expected Outcomes
- All technical decisions with rationale
- Architecture sketches and API specifications
- Security compliance validation

## Phase 1: Design & Contracts

### Data Model Design

#### Environment Settings Entity
- **qdrant_api_key** (string, required): API key for Qdrant Cloud
- **qdrant_url** (string, required): URL for Qdrant Cloud instance
- **cohere_api_key** (string, required): API key for Cohere
- **neon_database_url** (string, optional): Database connection string
- **validation_status** (boolean): Whether all required variables are loaded

#### Chat Session Entity
- **session_id** (string, required): Unique identifier for the session
- **created_at** (timestamp, required): When the session was created
- **expires_at** (timestamp, required): When the session expires
- **status** (enum, required): Current status (active, expired)
- **conversation_history** (array, optional): Array of message objects (user query, AI response pairs)

#### User Query Entity
- **query_id** (string, required): Unique identifier for the query
- **session_id** (string, required): Reference to parent session
- **content** (string, required): The text content provided by user
- **timestamp** (timestamp, required): When the query was made
- **processed_status** (enum, required): Current state of processing (pending, processing, completed, failed)

#### AI Response Entity
- **response_id** (string, required): Unique identifier for the response
- **query_id** (string, required): Reference to parent query
- **content** (string, required): The generated response text
- **timestamp** (timestamp, required): When the response was generated
- **source_chunks** (array, optional): Array of text chunk references used to generate response

### API Contract Specifications

#### Session Management Endpoints

**POST /api/session/create**
- **Description**: Create a new chat session
- **Request**: `{}`
- **Response**:
  ```json
  {
    "session_id": "string"
  }
  ```
- **Error Responses**:
  - 500: Internal server error

#### Chat Endpoints

**POST /api/chat**
- **Description**: Send a message to the chatbot
- **Request**:
  ```json
  {
    "session_id": "string",
    "message": "string",
    "text_content": "string (optional user-provided text)"
  }
  ```
- **Response**:
  ```json
  {
    "answer": "string"
  }
  ```
- **Error Responses**:
  - 400: Invalid request (missing session_id or message)
  - 404: Session not found
  - 500: Processing error

### Frontend Integration Design

#### Widget Implementation
- **Method**: Floating button that expands to chat interface
- **Position**: Bottom-right corner of screen
- **Trigger**: Click on "AI Assistant ▼" button
- **Style**: Minimal, consistent with textbook design

#### Communication Layer
- **Protocol**: REST API calls to backend
- **Session Management**: Automatic session creation on first open
- **Error Handling**: Graceful fallbacks for connection issues

### Quickstart Guide

#### Prerequisites
1. Python 3.8+ with pip
2. Node.js for Docusaurus development
3. Required API keys (Cohere, Qdrant, Neon)
4. .env file with required variables

#### Backend Setup
1. Install dependencies: `pip install fastapi uvicorn python-dotenv pydantic-settings cohere qdrant-client`
2. Create .env file with required variables:
   ```
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=your_qdrant_url
   COHERE_API_KEY=your_cohere_api_key
   NEON_DATABASE_URL=your_neon_db_url (optional)
   ```
3. Run backend: `uvicorn main:app --host 0.0.0.0 --port 8000`

#### Frontend Integration
1. Add chatbot widget to Docusaurus site
2. Configure API endpoints to backend
3. Test session creation and chat functionality

## Phase 2: Implementation Approach

### Phase 2A: Environment Setup & Validation
1. Implement BaseSettings class for secure environment loading
2. Add validation for required environment variables
3. Implement fail-fast mechanism for missing variables
4. Add secure logging for variable names only

### Phase 2B: Backend API Implementation
1. Implement POST /api/session/create endpoint
2. Implement POST /api/chat endpoint
3. Add proper error handling and validation
4. Test API endpoints for 404 errors

### Phase 2C: RAG Pipeline Implementation
1. Implement textbook content ingestion
2. Create chunking and embedding functionality
3. Implement Qdrant storage and retrieval
4. Create LLM response generation with content isolation

### Phase 2D: Frontend Chatbot UI
1. Create floating chatbot widget
2. Implement toggle open/close functionality
3. Connect to backend APIs
4. Handle session management and message flow

### Phase 2E: Integration & Validation
1. End-to-end testing of chat functionality
2. Environment validation testing
3. Error scenario testing
4. Performance validation (<5s response time)

## Architectural Decision Records (ADRs)

### ADR-001: Environment Configuration Approach
**Status**: Decided
**Decision**: Use Pydantic BaseSettings for secure environment loading
**Rationale**: Provides type safety, validation, and fail-fast capabilities while following security best practices.

### ADR-002: API Design Pattern
**Status**: Decided
**Decision**: REST API with JSON payloads
**Rationale**: Standard approach that works well with both frontend and backend technologies, providing simplicity and broad compatibility.

### ADR-003: Session Management Strategy
**Status**: Decided
**Decision**: In-memory with temporary persistence for development
**Rationale**: For local development and testing, in-memory storage is sufficient and aligns with the temporary nature of sessions while meeting constitutional requirements.

### ADR-004: RAG Pipeline Architecture
**Status**: Decided
**Decision**: Session-scoped temporary storage with strict content isolation
**Rationale**: Follows constitutional requirements for content isolation and temporary storage that gets deleted after session, ensuring privacy and compliance.

## Risk Assessment

### High Risk Items
1. **Environment Security**: Critical for protecting API keys and sensitive data
2. **Content Isolation**: Essential for constitutional compliance
3. **Session Management**: Required for maintaining conversation context

### Mitigation Strategies
1. **Environment Security**: Use BaseSettings with validation and secure logging practices
2. **Content Isolation**: Implement strict RAG flow per constitutional requirements
3. **Session Management**: Implement robust error handling and validation

## Success Criteria Validation

- [ ] Application starts successfully with valid environment variables
- [ ] Application fails fast with clear error messages for missing variables
- [ ] Backend API endpoints return 200 status (not 404)
- [ ] Session creation API returns valid session ID within 2 seconds
- [ ] Chat API returns responses within 5 seconds
- [ ] RAG pipeline successfully retrieves relevant context for textbook questions
- [ ] AI responses are generated from textbook content only (no hallucination)
- [ ] No secrets are exposed in source code, logs, or error messages
- [ ] "This topic is not covered in the textbook." response for off-topic questions