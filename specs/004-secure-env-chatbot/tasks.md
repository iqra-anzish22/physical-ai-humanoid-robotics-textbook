# Implementation Tasks: Secure Environment Setup + AI Chatbot Integration

**Feature**: 004-secure-env-chatbot
**Created**: 2025-12-15
**Status**: Draft
**Author**: Claude

## Implementation Strategy

This tasks document follows an incremental delivery approach where each user story represents a complete, independently testable increment. The implementation begins with foundational setup and environment validation, then progresses through core functionality in priority order (P1, P2, P3). Each user story includes all necessary components to deliver complete value.

**MVP Scope**: User Story 1 (Secure Environment Setup) provides the foundational infrastructure needed for all other functionality.

## Dependencies

User stories are designed to be independent where possible:
- User Story 1 (P1) must be completed before User Story 2 (P2) and User Story 3 (P3) can function properly
- User Story 2 (P2) and User Story 3 (P3) can be developed in parallel after User Story 1 is complete
- Foundational components (models, services) support all user stories

## Parallel Execution Examples

Per user story, tasks that can execute in parallel:
- **User Story 1**: [P] Environment model implementation can run in parallel with [P] validation logic implementation
- **User Story 2**: [P] Session endpoint implementation can run in parallel with [P] Chat endpoint implementation
- **User Story 3**: [P] Ingestion service can run in parallel with [P] Retrieval service

---

## Phase 1: Setup

Initialize project structure and dependencies for secure environment configuration.

- [X] T001 Create project structure with backend directory
- [X] T002 Create requirements.txt with FastAPI, Pydantic Settings, Cohere, Qdrant dependencies
- [X] T003 Set up .env.example file with required variable placeholders

---

## Phase 2: Foundational Components

Implement blocking prerequisites needed by all user stories.

- [X] T004 [P] Create EnvironmentSettings model in backend/config.py using Pydantic BaseSettings
- [X] T005 [P] Implement environment validation logic with fail-fast mechanism
- [X] T006 [P] Create ChatSession model in backend/models/session.py
- [X] T007 [P] Create UserQuery and AIResponse models in backend/models/message.py
- [X] T008 [P] Create SessionManager service in backend/services/session_manager.py
- [X] T009 [P] Create RAGPipeline service in backend/services/rag_pipeline.py

---

## Phase 3: User Story 1 - Secure Environment Setup (Priority: P1)

Developer starts the application and the system validates that all required environment variables (QDRANT_API_KEY, QDRANT_URL, COHERE_API_KEY, NEON_DATABASE_URL) are properly loaded from the .env file. The system fails fast with clear error messages if any required variables are missing, preventing runtime errors.

**Independent Test Criteria**: Application starts with valid .env and loads all variables successfully; application fails with clear error when variables are missing.

- [X] T010 [US1] Implement BaseSettings class for environment configuration in backend/config.py
- [X] T011 [US1] Add validation for required environment variables (QDRANT_API_KEY, QDRANT_URL, COHERE_API_KEY)
- [X] T012 [US1] Implement fail-fast mechanism for missing variables with clear error messages
- [X] T013 [US1] Add secure logging that shows variable names only (not values)
- [X] T014 [US1] Test environment loading with complete .env file
- [X] T015 [US1] Test environment loading with missing variables to verify fail-fast behavior

---

## Phase 4: User Story 2 - Backend API with Secure Configuration (Priority: P2)

Student interacts with the AI chatbot and the backend APIs function correctly, using the securely loaded configuration to connect to Qdrant and Cohere services. The APIs handle session creation and chat functionality without 404 errors.

**Independent Test Criteria**: POST /api/session/create returns session ID; POST /api/chat returns appropriate response.

- [X] T016 [US2] Create FastAPI application with CORS middleware in backend/main.py
- [X] T017 [US2] Implement POST /api/session/create endpoint returning session_id
- [X] T018 [US2] Implement POST /api/chat endpoint returning answer
- [X] T019 [US2] Add proper error handling (400, 404, 500 responses)
- [X] T020 [US2] Test session creation API for 200 status response
- [X] T021 [US2] Test chat API for 200 status response
- [X] T022 [US2] Test API endpoints do not return 404 errors

---

## Phase 5: User Story 3 - RAG Pipeline with Secure Services (Priority: P3)

The AI chatbot uses the securely configured services to implement a RAG pipeline that retrieves textbook content to answer user questions. The system properly ingests, chunks, embeds, and retrieves content using the secure API connections.

**Independent Test Criteria**: Questions about textbook content return responses from correct source material; off-topic questions return "This topic is not covered in the textbook."

- [X] T023 [US3] Implement textbook content ingestion in RAGPipeline service
- [X] T024 [US3] Create text chunking functionality with overlap
- [X] T025 [US3] Implement Cohere embedding generation for text chunks
- [X] T026 [US3] Implement Qdrant vector storage and retrieval
- [X] T027 [US3] Create LLM response generation with content isolation
- [X] T028 [US3] Implement "topic not covered" response for missing context
- [X] T029 [US3] Test RAG pipeline with textbook content questions
- [X] T030 [US3] Test RAG pipeline with off-topic questions

---

## Phase 6: User Story 4 - Frontend Chatbot UI Integration

Student interacts with a floating chatbot widget that connects to the backend APIs, creates sessions automatically, and handles message flow with proper loading and error states.

**Independent Test Criteria**: Widget appears as floating button, opens on click, creates session automatically, sends/receives messages.

- [X] T031 [US4] Create floating chatbot widget HTML/CSS in backend/chatbot_widget.html
- [X] T032 [US4] Implement toggle open/close functionality with smooth transitions
- [X] T033 [US4] Add automatic session creation on first widget open
- [X] T034 [US4] Implement message flow between frontend and backend APIs
- [X] T035 [US4] Add loading indicators and error handling states
- [X] T036 [US4] Test widget integration with backend APIs
- [X] T037 [US4] Verify no modification to existing textbook files

---

## Phase 7: Polish & Cross-Cutting Concerns

Final integration, testing, and documentation to ensure complete functionality.

- [X] T038 Create startup script with environment validation in backend/start_server.py
- [X] T039 Create API testing script in backend/test_api.py
- [X] T040 Document implementation in backend/IMPLEMENTATION.md
- [X] T041 Verify no secrets exposed in source code, logs, or error messages
- [X] T042 Test complete workflow: session creation → content ingestion → chat → response
- [X] T043 Validate all success criteria are met (response times, accuracy, etc.)
- [X] T044 Clean up temporary vector storage after sessions
- [X] T045 Final integration testing of all components together