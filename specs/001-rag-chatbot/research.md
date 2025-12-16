# Research Document: Isolated On-Demand RAG Chatbot

**Feature**: 001-rag-chatbot
**Created**: 2025-12-13
**Status**: Complete

## Decision: Cohere Embedding Model

**Rationale**: The requirements specifically call for Cohere's embed-english-v3.0 model (or latest). This model is well-suited for the task as it provides high-quality embeddings for English text and is compatible with the constitutional requirements that prohibit OpenAI usage.

**Alternatives Considered**:
- OpenAI embeddings (prohibited by constitutional requirement)
- Sentence Transformers (would require model hosting)
- Other Cohere models (embed-english-v3.0 is specified in requirements)

**Final Choice**: embed-english-v3.0 or latest compatible version

## Decision: Qdrant Collection Configuration

**Rationale**: To meet the constitutional requirement of session-only storage with no persistence, we'll use Qdrant collections that are created per session and deleted after use. This ensures complete isolation between users and prevents any data leakage.

**Parameters**:
- Collection names will use session UUIDs to ensure uniqueness
- Vector dimensions set to 1024 (compatible with Cohere embed-english-v3.0)
- Time-to-live (TTL) settings for automatic cleanup if needed
- No permanent collections will be created

**Alternatives Considered**:
- Permanent collections (violates constitutional requirement)
- Single collection with partitioning (risk of cross-session data access)

## Decision: Text Chunk Size & Overlap

**Rationale**: The requirements specify a range of 500-800 tokens with overlap. A 600-token chunk size provides a good balance between context preservation and processing efficiency. A 100-token overlap ensures continuity between chunks without excessive redundancy.

**Parameters**:
- Chunk size: 600 tokens
- Overlap: 100 tokens
- This falls within the specified range (500-800)

**Alternatives Considered**:
- Larger chunks (might lose context at boundaries)
- Smaller chunks (might lose context within chunks)
- No overlap (might break context across chunks)

## Decision: Error Handling Strategy

**Rationale**: A robust error handling strategy is essential for user experience while maintaining security and privacy requirements. The approach should provide helpful feedback without exposing system internals or compromising privacy.

**Approach**:
- Cohere API failures: Return generic "processing error" messages to users
- Qdrant failures: Attempt cleanup and return appropriate error
- Input validation: Clear, specific error messages for invalid inputs
- All errors logged internally (without user data) for debugging

**Alternatives Considered**:
- Silent failures (poor user experience)
- Detailed error messages (potential security risk)

## Decision: Session Management

**Rationale**: Session management is critical to maintaining the constitutional requirement of no cross-session memory. Each session must be completely isolated with its own temporary storage that gets cleaned up after use.

**Approach**:
- Generate UUID for each session
- Create Qdrant collection with session ID
- Associate all user data with the session ID
- Implement cleanup mechanism that deletes collection when session ends
- Automatic cleanup after timeout period
- No persistent storage of user text

**Alternatives Considered**:
- Persistent sessions (violates constitutional requirement)
- Server-side session storage (risk of data persistence)