# Quickstart Guide: Isolated On-Demand RAG Chatbot

## Prerequisites

- Python 3.11+
- pip package manager
- Access to Cohere API (API key)
- Access to Qdrant Cloud (API key and URL)
- (Optional) Neon Postgres database URL

## Setup

### 1. Clone the Repository

```bash
git clone <repository-url>
cd rag-chatbot
```

### 2. Create Virtual Environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install fastapi uvicorn cohere qdrant-client python-dotenv psycopg2-binary
```

### 4. Set Environment Variables

Create a `.env` file in the project root with the following variables:

```env
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_URL=your-qdrant-url
COHERE_API_KEY=your-cohere-api-key
NEON_DATABASE_URL=your-neon-db-url  # Optional
```

## Running the Application

### Development Mode

```bash
uvicorn main:app --reload
```

The API will be available at `http://localhost:8000`.

### Production Mode

```bash
uvicorn main:app --workers 4
```

## Using the API

### 1. Create a Session

Submit your text content to create a new session:

```bash
curl -X POST http://localhost:8000/api/v1/session \
  -H "Content-Type: application/json" \
  -d '{
    "text": "Your text content here...",
    "chunk_size": 600,
    "overlap": 100
  }'
```

Response:
```json
{
  "session_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
  "chunk_count": 5,
  "status": "completed"
}
```

### 2. Ask Questions

Use the session ID to ask questions about your text:

```bash
curl -X POST http://localhost:8000/api/v1/session/{session_id}/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Your question here?"
  }'
```

Response:
```json
{
  "response": "The answer based on your provided text...",
  "sources": ["chunk-id-1", "chunk-id-2"],
  "processing_time": 2.34
}
```

### 3. Clean Up Session (Optional)

Explicitly delete a session and its data:

```bash
curl -X DELETE http://localhost:8000/api/v1/session/{session_id}
```

## Frontend Integration

The chatbot can be embedded in other applications using the provided JavaScript widget:

```html
<div id="rag-chatbot-container"></div>
<script src="/static/chatbot-widget.js"></script>
<script>
  RAGChatbot.init({
    containerId: 'rag-chatbot-container',
    apiUrl: 'http://localhost:8000/api/v1'
  });
</script>
```

## Configuration Options

### Environment Variables

- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `QDRANT_URL`: URL for Qdrant Cloud instance
- `COHERE_API_KEY`: API key for Cohere
- `NEON_DATABASE_URL`: Connection string for Neon Postgres (optional)
- `MAX_TEXT_LENGTH`: Maximum length of text that can be processed (default: 100000 characters)
- `DEFAULT_CHUNK_SIZE`: Default size of text chunks in tokens (default: 600)
- `DEFAULT_OVERLAP`: Default overlap between chunks in tokens (default: 100)
- `SESSION_TIMEOUT`: Session timeout in minutes (default: 30)

## Error Handling

The API returns appropriate HTTP status codes:

- `200`: Success
- `201`: Resource created
- `400`: Bad request (invalid input)
- `404`: Resource not found
- `500`: Internal server error

Error responses include a JSON object with an "error" field:

```json
{
  "error": "Descriptive error message"
}
```

## Monitoring and Logging

The application logs important events to standard output. For production deployments, consider redirecting logs to a file or using a logging service.

## Troubleshooting

### Common Issues

1. **API Key Errors**: Verify that all required environment variables are set correctly
2. **Connection Issues**: Check that the QDRANT_URL and network connectivity are working
3. **Rate Limits**: If experiencing timeout errors, check your Cohere and Qdrant API usage limits

2. **Large Text Processing**: Very large texts may take longer to process. Consider breaking them into smaller segments if needed.