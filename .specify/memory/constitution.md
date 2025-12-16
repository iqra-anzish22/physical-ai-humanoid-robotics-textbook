<!--
Sync Impact Report:
- Version change: N/A (initial constitution) → 1.0.0
- Added sections: Content Isolation, RAG Flow, Tech Stack, UI Requirements
- Removed sections: None
- Templates requiring updates: N/A
- Follow-up TODOs: None
-->
# Isolated On-Demand RAG Chatbot Constitution

## Core Principles

### Content Isolation (NON-NEGOTIABLE)
NEVER access, ingest, reference, or infer from full/existing book content. Responses ONLY from user-pasted/selected text. If no text provided → refuse to answer. Zero hallucinations, zero external knowledge. No persistence across queries or sessions.

### Strict RAG Flow Per Query
Every query follows: User provides text + question → Chunk text (500–800 tokens, overlap) → Generate embeddings via Cohere → Store vectors TEMPORARILY in session-scoped Qdrant → Retrieve top-k chunks → Generate answer ONLY from retrieved chunks → Delete vectors after session.

### Technology Constraint Compliance
NO OpenAI (SDK/keys) allowed. NO pre-ingestion of book content. NO storage/logging of user text. Use only free tiers. API keys via env vars. Core code must remain ≤1200 lines.

### Session-Based Processing
All processing is isolated per session. Temporary vector storage that gets deleted after session. No cross-session data persistence. Each query is independent and self-contained.

### Accuracy and Grounding Standards
≥95% grounded accuracy. Zero content leakage. <5s response time. Auditable isolation. Answers clearly show they are based only on the text the user provides.

### Privacy and Data Handling
No user text persistence. No logging of user-provided content. Session-scoped temporary storage only. Strict privacy controls for user-provided text.

## Technical Requirements

### Tech Stack Mandates
- LLM: Cohere (embed-english-v3.0, command-r-plus or latest)
- Backend: FastAPI
- Vector DB: Qdrant Cloud (free, session-only)
- DB: Neon Postgres (optional, metadata ONLY)
- Frontend: Lightweight embeddable chatbot widget

### Performance Standards
- Response time: <5 seconds per query
- Accuracy: ≥95% grounded responses
- Availability: High uptime for on-demand usage
- Scalability: Handle concurrent user sessions

### Security Requirements
- Secure handling of API keys via environment variables
- No exposure of user-provided text to external systems beyond immediate processing
- Secure embedding generation and retrieval pipeline
- Protected session data isolation

## Development Workflow

### Code Quality Standards
- Maximum 1200 lines of core code
- Modular, well-documented components
- Type safety enforcement
- Comprehensive error handling

### Testing Requirements
- Unit tests for all core components
- Integration tests for RAG pipeline
- End-to-end tests for complete query flow
- Privacy compliance verification tests

### Review Process
- Verify compliance with content isolation rules
- Check for adherence to tech stack constraints
- Confirm no data persistence violations
- Validate performance requirements

## Governance

All development and changes must comply with these constitutional principles. Any deviation from content isolation, tech stack constraints, or privacy requirements is prohibited. All PRs/reviews must verify compliance with non-negotiable principles. Complexity must be justified with clear benefits that don't compromise isolation or privacy.

**Version**: 1.0.0 | **Ratified**: 2025-12-13 | **Last Amended**: 2025-12-13