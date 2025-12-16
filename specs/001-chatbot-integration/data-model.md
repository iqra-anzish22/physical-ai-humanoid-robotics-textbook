# Data Model: AI Chatbot Integration

## Entity: ChatSession

**Description**: Represents a single conversation context between user and AI

**Fields**:
- `session_id` (string, required): Unique identifier for the session
- `created_at` (timestamp, required): When the session was created
- `expires_at` (timestamp, required): When the session expires
- `status` (enum, required): Current status (active, expired)
- `conversation_history` (array, optional): Array of message objects (user query, AI response pairs)
- `vector_storage_ref` (string, optional): Reference to temporary Qdrant collection for this session

**Validation Rules**:
- `session_id` must be unique
- `expires_at` must be after `created_at`
- `status` must be one of ['active', 'expired']

## Entity: UserQuery

**Description**: A question or input from the student that is sent to the AI for processing

**Fields**:
- `query_id` (string, required): Unique identifier for the query
- `session_id` (string, required): Reference to parent session
- `content` (string, required): The text content provided by user
- `timestamp` (timestamp, required): When the query was made
- `processed_status` (enum, required): Current state of processing (pending, processing, completed, failed)

**Validation Rules**:
- `session_id` must reference an existing active session
- `content` must not be empty
- `processed_status` must be one of ['pending', 'processing', 'completed', 'failed']

## Entity: AIResponse

**Description**: The generated answer from the AI based on the user query and relevant textbook content

**Fields**:
- `response_id` (string, required): Unique identifier for the response
- `query_id` (string, required): Reference to parent query
- `content` (string, required): The generated response text
- `timestamp` (timestamp, required): When the response was generated
- `source_chunks` (array, optional): Array of text chunk references used to generate response

**Validation Rules**:
- `query_id` must reference an existing query
- `content` must not be empty

## Entity: TextChunk

**Description**: A segment of text content used for RAG processing

**Fields**:
- `chunk_id` (string, required): Unique identifier for the chunk
- `session_id` (string, required): Reference to parent session
- `content` (string, required): The text content of the chunk
- `chunk_index` (number, required): Sequential position in the original text
- `vector_id` (string, optional): Reference to the vector in Qdrant storage

**Validation Rules**:
- `session_id` must reference an existing active session
- `content` must not be empty
- `chunk_index` must be a non-negative integer

## State Transitions

### ChatSession State Transitions
- `active` → `expired`: When session reaches expiration time
- No other state transitions allowed

### UserQuery State Transitions
- `pending` → `processing`: When query is picked up for processing
- `processing` → `completed`: When processing finishes successfully
- `processing` → `failed`: When processing encounters an error

## Relationships

1. **ChatSession → UserQuery**: One-to-many (one session can have many queries)
2. **UserQuery → AIResponse**: One-to-one (each query generates one response)
3. **ChatSession → TextChunk**: One-to-many (one session can have many text chunks)
4. **UserQuery → TextChunk**: Many-to-many through RAG retrieval (query can reference multiple chunks)

## Constraints

1. All data is temporary and should be cleaned up when session expires
2. No cross-session data persistence
3. Each session operates independently
4. Data is only retained for the duration of the session