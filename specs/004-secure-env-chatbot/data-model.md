# Data Model: Secure Environment Setup + AI Chatbot Integration

## Entity: EnvironmentSettings

**Description**: Configuration loaded from .env file including API keys and URLs

**Fields**:
- `qdrant_api_key` (string, required): API key for Qdrant Cloud
- `qdrant_url` (string, required): URL for Qdrant Cloud instance
- `cohere_api_key` (string, required): API key for Cohere
- `neon_database_url` (string, optional): Database connection string
- `validation_status` (boolean, required): Whether all required variables are loaded

**Validation Rules**:
- `qdrant_api_key` must not be empty
- `qdrant_url` must be a valid URL
- `cohere_api_key` must not be empty
- `neon_database_url` if provided, must be a valid database URL

## Entity: ChatSession

**Description**: Represents a single conversation context between user and AI

**Fields**:
- `session_id` (string, required): Unique identifier for the session
- `created_at` (timestamp, required): When the session was created
- `expires_at` (timestamp, required): When the session expires
- `status` (enum, required): Current status (active, expired)
- `conversation_history` (array, optional): Array of message objects (user query, AI response pairs)

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

## Entity: TextbookContent

**Description**: The source material from the textbook that is ingested and stored for retrieval

**Fields**:
- `content_id` (string, required): Unique identifier for the content
- `source_file` (string, required): Original source file path
- `text_content` (string, required): The plain text content
- `created_at` (timestamp, required): When the content was ingested
- `chunks` (array, optional): Array of content chunks for RAG

**Validation Rules**:
- `content_id` must be unique
- `text_content` must not be empty
- `source_file` must be a valid file path

## Entity: RetrievedContext

**Description**: Specific segments of textbook content retrieved by the RAG system to answer a particular query

**Fields**:
- `context_id` (string, required): Unique identifier for the context
- `query_id` (string, required): Reference to the query that triggered retrieval
- `content_chunks` (array, required): Array of text chunks retrieved
- `relevance_scores` (array, optional): Array of relevance scores for each chunk
- `timestamp` (timestamp, required): When the context was retrieved

**Validation Rules**:
- `query_id` must reference an existing query
- `content_chunks` must not be empty

## Entity: Embeddings

**Description**: Vector representations of textbook content used for similarity search in Qdrant

**Fields**:
- `embedding_id` (string, required): Unique identifier for the embedding
- `content_id` (string, required): Reference to the original content
- `vector` (array, required): Array of float values representing the embedding
- `chunk_text` (string, required): The original text chunk that was embedded
- `created_at` (timestamp, required): When the embedding was created

**Validation Rules**:
- `content_id` must reference an existing content
- `vector` must not be empty
- `chunk_text` must not be empty

## State Transitions

### ChatSession State Transitions
- `active` → `expired`: When session reaches expiration time
- No other state transitions allowed

### UserQuery State Transitions
- `pending` → `processing`: When query is picked up for processing
- `processing` → `completed`: When processing finishes successfully
- `processing` → `failed`: When processing encounters an error

## Relationships

1. **EnvironmentSettings → ChatSession**: One-to-many (settings enable session creation)
2. **ChatSession → UserQuery**: One-to-many (one session can have many queries)
3. **UserQuery → AIResponse**: One-to-one (each query generates one response)
4. **TextbookContent → Embeddings**: One-to-many (content gets chunked and embedded)
5. **UserQuery → RetrievedContext**: One-to-one (each query retrieves context)
6. **RetrievedContext → Embeddings**: Many-to-many (context uses multiple embeddings)

## Constraints

1. All environment variables must be loaded from .env file
2. No hard-coding of API keys or URLs
3. All processing is isolated per session
4. Temporary vector storage that gets deleted after session
5. No cross-session data persistence
6. Each query is independent and self-contained
7. Responses only from user-provided text (no pre-ingested content)