# Feature Specification: Isolated On-Demand RAG Chatbot for Published Book

**Feature Branch**: `001-rag-chatbot`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "Project: Isolated On-Demand RAG Chatbot for Published Book
(STRICTLY bound to /sp.constitution)

OBJECTIVE:
Build an isolated, privacy-first RAG chatbot that answers questions
ONLY from user-pasted/selected text. No access to any existing book content.

TECH STACK:
- Backend: FastAPI
- LLM: Cohere
  - Embeddings: embed-english-v3.0 (or latest)
  - Generation: command-r-plus (or latest)
- Vector DB: Qdrant Cloud (free tier, session-only collections)
- DB (optional): Neon Postgres (session metadata ONLY)
- Frontend: Lightweight embeddable chatbot widget (iframe/script)

ENVIRONMENT VARIABLES (REQUIRED):
- QDRANT_API_KEY
- QDRANT_URL
- COHERE_API_KEY
- NEON_DATABASE_URL

NOTE:
- Keys are provided via environment variables ONLY
- No keys may appear in code, logs, or prompts

RAG PIPELINE (PER QUERY):
1. User provides text + question
2. Chunk text (500–800 tokens, overlap)
3. Generate embeddings via Cohere
4. Create temporary Qdrant session collection
5. Store embeddings (no persistence)
6. Retrieve top-k relevant chunks
7. Generate answer ONLY from retrieved chunks
8. Delete collection after session

HARD CONSTRAINTS:
- NO pre-ingestion of book content
- NO OpenAI SDKs or APIs
- NO storage/logging of user text
- NO cross-session memory
- Free tiers only
- Core codebase ≤1200 lines

UI REQUIREMENTS:
- User must paste/select text before asking
- Clear notice: "Answers are based only on the text you provide"
- Mobile responsive
- Embeddable via iframe/script

DELIVERABLES:
- Modular FastAPI backend
- Cohere-only RAG implementation
- Session-isolated Qdrant usage
- Frontend widget
- README with setup & env usage

SUCCESS CRITERIA:
- ≥95% grounded accuracy
- Zero hallucinations
- Zero data leakage
- <5s response time
- Manual audit confirms isolation"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Submit Text and Ask Question (Priority: P1)

As a user, I want to paste or select text and ask a question about it, so that I can get answers based only on the text I provide.

**Why this priority**: This is the core functionality of the RAG chatbot - without this basic interaction, the feature has no value.

**Independent Test**: Can be fully tested by pasting text into the interface, asking a question related to that text, and receiving an answer based only on the provided text.

**Acceptance Scenarios**:

1. **Given** I have text content available, **When** I paste that text and ask a question related to it, **Then** I receive an answer based only on the text I provided
2. **Given** I have no text content provided, **When** I try to ask a question, **Then** I receive an error or notice that I must provide text first

---

### User Story 2 - View Isolation Notice (Priority: P1)

As a user, I want to see a clear notice that answers are based only on the text I provide, so that I understand the privacy and data handling model.

**Why this priority**: This is essential for user trust and understanding of the system's limitations and privacy model.

**Independent Test**: Can be fully tested by viewing the interface and confirming the notice is clearly visible and understandable.

**Acceptance Scenarios**:

1. **Given** I access the chatbot interface, **When** I view the interface, **Then** I see a clear notice that answers are based only on the text I provide

---

### User Story 3 - Receive Fast Response (Priority: P2)

As a user, I want to receive responses quickly, so that I can have a smooth conversational experience.

**Why this priority**: Performance is critical for user experience and adoption of the chatbot.

**Independent Test**: Can be tested by measuring the time between question submission and response receipt, ensuring it's under 5 seconds.

**Acceptance Scenarios**:

1. **Given** I have provided text and asked a question, **When** I submit the question, **Then** I receive a response in under 5 seconds

---

### User Story 4 - Embed Chatbot in Other Applications (Priority: P3)

As a developer, I want to embed the chatbot in other applications via iframe or script, so that I can integrate it into various platforms.

**Why this priority**: This enables broader adoption and integration of the chatbot functionality.

**Independent Test**: Can be tested by embedding the chatbot in a simple HTML page and verifying it functions correctly.

**Acceptance Scenarios**:

1. **Given** I have the embed code, **When** I add it to an HTML page, **Then** the chatbot interface appears and functions correctly

---

### Edge Cases

- What happens when a user provides extremely large text blocks?
- How does the system handle malformed or empty questions?
- What occurs if the Cohere API is unavailable during processing?
- How does the system handle concurrent users and session isolation?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST allow users to paste or select text content before asking questions
- **FR-002**: System MUST generate embeddings using Cohere's embed-english-v3.0 model (or latest)
- **FR-003**: System MUST store embeddings temporarily in Qdrant Cloud session collections
- **FR-004**: System MUST retrieve relevant text chunks based on the user's question
- **FR-005**: System MUST generate answers using Cohere's command-r-plus model (or latest)
- **FR-006**: System MUST answer questions ONLY from the retrieved chunks, with no external knowledge
- **FR-007**: System MUST delete temporary embeddings after the session ends
- **FR-008**: System MUST display a clear notice that answers are based only on provided text
- **FR-009**: System MUST process queries in under 5 seconds
- **FR-010**: System MUST be embeddable via iframe or script
- **FR-011**: System MUST be mobile responsive
- **FR-012**: System MUST NOT store or log user-provided text
- **FR-013**: System MUST NOT access any pre-existing book content
- **FR-014**: System MUST NOT use OpenAI APIs or SDKs
- **FR-015**: System MUST NOT persist data across sessions

### Key Entities

- **Text Session**: Represents a user's interaction session with temporary text storage, with no persistence beyond the session
- **Question**: A user's query about the provided text, processed without external knowledge
- **Response**: An answer generated based only on the provided text chunks, with no hallucinations

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive answers based on ≥95% grounded accuracy from the provided text
- **SC-002**: Zero hallucinations occur in responses (responses only use provided text)
- **SC-003**: Zero data leakage occurs (no external content used in responses)
- **SC-004**: Response time is under 5 seconds for 95% of queries
- **SC-005**: Manual audit confirms complete isolation from any pre-existing book content
- **SC-006**: Users successfully complete the text-paste-and-ask workflow on first attempt 90% of the time