# Isolated On-Demand RAG Chatbot

An isolated, privacy-first RAG chatbot that answers questions ONLY from user-provided text. No access to any existing book content.

## Features

- Complete content isolation - answers only from user-provided text
- Session-based processing with automatic cleanup
- FastAPI backend with Cohere integration
- Qdrant vector database for temporary storage
- Embeddable widget for easy integration
- Mobile responsive design
- Document upload support (PDF, DOCX, PPTX, TXT, MD)
- Rate limiting to prevent abuse
- Input sanitization for security
- Comprehensive error handling with appropriate HTTP status codes
- Graceful degradation when external APIs are unavailable
- Comprehensive input validation
- Under 1200 lines of core code

## Prerequisites

- Python 3.11+
- Docker and Docker Compose (optional, for containerized deployment)
- Cohere API key
- Qdrant Cloud account and API key

## Setup

### Environment Variables

Create a `.env` file in the backend directory with the following variables:

```env
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_URL=your-qdrant-url
COHERE_API_KEY=your-cohere-api-key
NEON_DATABASE_URL=your-neon-db-url  # Optional
MAX_TEXT_LENGTH=100000  # Maximum length of text that can be processed
DEFAULT_CHUNK_SIZE=600  # Default size of text chunks in tokens
DEFAULT_OVERLAP=100  # Default overlap between chunks in tokens
SESSION_TIMEOUT=30  # Session timeout in minutes
```

### Local Development

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Run the application:
```bash
uvicorn src.main:app --reload
```

### Docker Deployment

1. Build and run with Docker Compose:
```bash
docker-compose up -d
```

The API will be available at `http://localhost:8000`.

## API Usage

### Create a Session

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

### Ask Questions

Use the session ID to ask questions about your text:

```bash
curl -X POST http://localhost:8000/api/v1/session/{session_id}/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "Your question here?"
  }'
```

### Clean Up Session (Optional)

Explicitly delete a session and its data:

```bash
curl -X DELETE http://localhost:8000/api/v1/session/{session_id}
```

### Document Processing (Knowledge Pipeline)

Upload and process documents through the knowledge pipeline:

```bash
curl -X POST http://localhost:8000/knowledge-pipeline/upload \
  -H "accept: application/json" \
  -F "file=@document.pdf"
```

Check supported formats:
```bash
curl -X GET http://localhost:8000/knowledge-pipeline/supported-formats
```

### Rate Limiting

The API implements rate limiting:
- Session creation: 5 requests per minute per IP
- Query requests: 20 requests per minute per IP
- Session deletion: 10 requests per minute per IP
- Health checks: 60 requests per minute per IP

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

## Architecture

The system follows a strict privacy-first architecture:

1. User provides text content
2. Text is chunked (600 tokens with 100-token overlap)
3. Embeddings are generated using Cohere
4. Embeddings are stored temporarily in Qdrant session collections
5. User questions are processed against the embeddings
6. Answers are generated using Cohere based only on retrieved chunks
7. All session data is automatically cleaned up after use

## Compliance

- Zero data persistence across sessions
- No access to pre-existing content
- No OpenAI APIs used (Cohere only)
- Core code under 1200 lines
- All API keys via environment variables only

## License

[Specify your license here]