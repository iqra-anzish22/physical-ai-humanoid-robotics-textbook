# Research Document: AI Chatbot Integration

## Research Task 1: Session Storage Method

**Decision**: In-memory storage with automatic cleanup for development
**Rationale**: For local development and testing, in-memory storage is sufficient, simple to implement, and aligns with the temporary nature of sessions. Production would require persistent storage, but for the current scope (localhost development), in-memory is appropriate.
**Alternatives considered**:
- Database storage: More complex, requires schema management
- File-based storage: Could persist between runs, harder to manage cleanup
- In-memory with automatic cleanup: Simple, temporary, automatically cleaned up

## Research Task 2: API Contract Design

**Decision**: RESTful API with JSON payloads
**Rationale**: REST is standard, well-understood, and appropriate for the integration. JSON is the natural choice for web communication and works well with both frontend and backend technologies.
**Alternatives considered**:
- GraphQL: More complex for this simple use case
- gRPC: Overkill for web frontend integration
- REST with JSON: Standard, simple, well-supported

## Research Task 3: Fallback Behavior

**Decision**: User-friendly error messages with option to retry or start new session
**Rationale**: When document processing fails, users should receive clear, non-technical messages with actionable steps rather than seeing technical error details.
**Alternatives considered**:
- Generic error message: Less helpful for users
- Technical error details: Confusing and unhelpful
- User-friendly with retry option: Clear, helpful, actionable

## Research Task 4: Frontend Integration Method

**Decision**: Embedded widget that appears as a floating panel
**Rationale**: A floating panel widget can be integrated without modifying existing textbook content or layout, meeting the requirement of not changing the book UI while providing the chat functionality.
**Alternatives considered**:
- Full-page integration: Would require layout changes
- Fixed sidebar: Would modify page layout significantly
- Floating widget panel: Non-intrusive, minimal impact, meets requirements

## Research Task 5: CORS Configuration

**Decision**: Localhost-only CORS for development
**Rationale**: For local development security, restrict CORS to localhost origins only. This prevents security issues while allowing frontend-backend communication on localhost.
**Alternatives considered**:
- Wildcard CORS: Security risk
- No CORS restrictions: Security risk
- Localhost-only: Secure, functional for development