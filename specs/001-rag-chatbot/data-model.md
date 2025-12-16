# Data Model: Isolated On-Demand RAG Chatbot

**Feature**: 001-rag-chatbot
**Created**: 2025-12-13
**Status**: Complete

## Text Session Entity

**Description**: Represents a user's interaction session with temporary text storage, with no persistence beyond the session

**Fields**:
- `session_id`: UUID (Primary Key) - unique identifier for the session
- `created_at`: timestamp - when session was created
- `expires_at`: timestamp - when session should be cleaned up
- `status`: enum ['active', 'completed', 'expired'] - current state of the session
- `chunk_count`: integer - number of text chunks in this session

**Validation Rules**:
- `session_id` must be unique
- `expires_at` must be after `created_at`
- `status` must be one of the defined enum values
- `chunk_count` must be non-negative

**State Transitions**:
- 'active' → 'completed' when session is explicitly completed
- 'active' → 'expired' when expiration time is reached
- 'completed' → 'expired' when cleanup occurs

## Text Chunk Entity

**Description**: Represents a chunk of text that has been processed and embedded for retrieval

**Fields**:
- `chunk_id`: UUID (Primary Key) - unique identifier for the chunk
- `session_id`: UUID (Foreign Key) - references the parent Text Session
- `content`: text - the actual text content of the chunk
- `embedding_vector`: array[float] - Cohere embedding vector (dimension: 1024)
- `position`: integer - sequential position of this chunk in the original text
- `token_count`: integer - number of tokens in this chunk

**Validation Rules**:
- `session_id` must reference an existing Text Session
- `embedding_vector` must have exactly 1024 dimensions
- `position` must be non-negative
- `token_count` must be positive and within chunk size limits

**Relationships**:
- One Text Session → Many Text Chunks (one-to-many)

## User Query Entity

**Description**: Represents a user's question and the system's response within a session

**Fields**:
- `query_id`: UUID (Primary Key) - unique identifier for the query
- `session_id`: UUID (Foreign Key) - references the parent Text Session
- `question`: text - the user's original question
- `timestamp`: timestamp - when the question was asked
- `response`: text - the generated response from the system
- `processing_time`: float - time taken to process the query in seconds
- `source_chunks`: array[UUID] - chunk_ids used to generate the response

**Validation Rules**:
- `session_id` must reference an existing Text Session
- `question` must not be empty
- `processing_time` must be positive
- `source_chunks` must reference existing Text Chunks in the same session

**Relationships**:
- One Text Session → Many User Queries (one-to-many)
- Many User Queries → Many Text Chunks (many-to-many through source_chunks)

## Session Metadata Entity (Optional, for Neon Postgres)

**Description**: Stores non-sensitive metadata about sessions (for analytics, monitoring, etc.)

**Fields**:
- `metadata_id`: UUID (Primary Key) - unique identifier for the metadata record
- `session_id`: UUID (Foreign Key) - references the Text Session
- `user_agent`: text - client information (optional, for analytics)
- `created_at`: timestamp - when metadata was created
- `request_count`: integer - number of queries made in this session
- `total_processing_time`: float - cumulative processing time for all queries

**Validation Rules**:
- `session_id` must reference an existing Text Session
- `request_count` must be non-negative
- `total_processing_time` must be non-negative

**Relationships**:
- One Text Session → One Session Metadata (one-to-one, optional)