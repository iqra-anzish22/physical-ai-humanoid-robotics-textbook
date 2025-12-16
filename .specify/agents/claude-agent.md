# AI Chatbot Integration Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-15

## Active Technologies

- FastAPI: Backend framework for the AI chatbot service
- Cohere API: Used for embeddings (embed-english-v3.0) and LLM (command-r-plus)
- Qdrant Cloud: Vector database for temporary session-based storage
- Python 3.8+: Backend runtime environment
- HTML/CSS/JavaScript: Frontend integration for textbook UI
- REST API: Communication protocol between frontend and backend
- In-memory session storage: Temporary session management for local development

## Project Structure

```text
specs/001-chatbot-integration/
├── spec.md                 # Feature specification
├── plan.md                 # Implementation plan
├── research.md             # Research findings and decisions
├── data-model.md           # Data model definitions
├── quickstart.md           # Quickstart guide
├── contracts/
│   └── api-spec.yaml       # OpenAPI specification
└── checklists/
    └── requirements.md     # Quality checklist
```

## Commands

### Backend Development
- `uvicorn main:app --host 0.0.0.0 --port 8000` - Run backend server
- `pip install fastapi uvicorn python-multipart cohere qdrant-client python-dotenv` - Install dependencies
- `curl http://localhost:8000/docs` - Access API documentation

### API Testing
- `curl -X POST "http://localhost:8000/api/session/create"` - Create session
- `curl -X POST "http://localhost:8000/api/chat/{session_id}" -H "Content-Type: application/json" -d '{"message": "test", "text_content": "test"}'` - Send chat message

### Environment Setup
- Set COHERE_API_KEY environment variable
- Set QDRANT_API_KEY environment variable
- Set QDRANT_URL environment variable

## Code Style

### Python (FastAPI)
- Use type hints for all function parameters and return values
- Follow PEP 8 style guide
- Use async/await for I/O operations
- Implement proper error handling with try/catch blocks
- Use environment variables for configuration

### JavaScript (Frontend)
- Use modern ES6+ features
- Implement proper error handling for API calls
- Use async/await for asynchronous operations
- Follow accessibility best practices
- Keep DOM manipulation minimal and efficient

## Recent Changes

- Feature 001-chatbot-integration: Added AI chatbot integration with textbook frontend
- Feature 001-chatbot-integration: Implemented session management with in-memory storage
- Feature 001-chatbot-integration: Created REST API with proper error handling and CORS configuration

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->