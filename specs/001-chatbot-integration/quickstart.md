# Quickstart Guide: AI Chatbot Integration

## Overview
This guide provides step-by-step instructions to set up and run the AI Chatbot Integration for your existing textbook frontend.

## Prerequisites

### System Requirements
- Python 3.8 or higher
- Node.js (if modifying frontend)
- pip package manager

### API Keys Required
- Cohere API key (for embeddings and LLM)
- Qdrant Cloud API key and URL (for vector storage)

## Backend Setup

### 1. Clone and Navigate
```bash
# Navigate to your project directory
cd your-project-directory
```

### 2. Install Python Dependencies
```bash
pip install fastapi uvicorn python-multipart cohere qdrant-client python-dotenv
```

### 3. Set Environment Variables
Create a `.env` file in your backend directory with the following:
```bash
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cloud_url_here
```

### 4. Run the Backend Server
```bash
# Run on port 8000 (or your preferred 800x port)
uvicorn main:app --host 0.0.0.0 --port 8000 --reload
```

The backend should now be running at `http://localhost:8000`.

## Frontend Integration

### 1. Add Chat Widget to Your Textbook
Include the following HTML element where you want the chat widget to appear:
```html
<div id="chatbot-container" style="position: fixed; bottom: 20px; right: 20px; z-index: 1000;">
  <div id="chatbot-widget" style="display: none;">
    <!-- Chat interface will be rendered here -->
  </div>
  <button id="chatbot-toggle" style="...">
    💬 Chat with Textbook
  </button>
</div>
```

### 2. Include JavaScript for Chat Functionality
Add the following JavaScript to handle communication with the backend:

```javascript
class TextbookChatbot {
  constructor(apiBaseUrl = 'http://localhost:8000') {
    this.apiBaseUrl = apiBaseUrl;
    this.sessionId = null;
    this.initializeWidget();
  }

  async initializeWidget() {
    // Create the chat interface
    this.createChatInterface();

    // Set up event listeners
    this.setupEventListeners();

    // Try to create a session automatically
    await this.createSession();
  }

  async createSession() {
    try {
      const response = await fetch(`${this.apiBaseUrl}/api/session/create`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' }
      });

      if (response.ok) {
        const data = await response.json();
        this.sessionId = data.session_id;
        console.log('Session created:', this.sessionId);
      } else {
        console.error('Failed to create session:', response.status);
      }
    } catch (error) {
      console.error('Error creating session:', error);
    }
  }

  async sendMessage(message, textContent = '') {
    if (!this.sessionId) {
      await this.createSession();
    }

    try {
      const response = await fetch(`${this.apiBaseUrl}/api/chat/${this.sessionId}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          message: message,
          text_content: textContent
        })
      });

      if (response.ok) {
        const data = await response.json();
        return data.response;
      } else if (response.status === 404) {
        // Session expired, create a new one
        await this.createSession();
        return "Session was expired. Please try sending your message again.";
      } else {
        const errorData = await response.json();
        return `Error: ${errorData.error || 'Failed to get response'}`;
      }
    } catch (error) {
      console.error('Error sending message:', error);
      return 'Error: Could not send message. Please try again.';
    }
  }

  createChatInterface() {
    // Implementation for creating the chat UI
    // This would include message display, input field, etc.
  }

  setupEventListeners() {
    // Implementation for handling UI events
  }
}

// Initialize the chatbot when the page loads
document.addEventListener('DOMContentLoaded', () => {
  const chatbot = new TextbookChatbot();
});
```

### 3. Enable CORS for Local Development
Make sure your FastAPI backend has CORS configured for localhost:

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],  # Your frontend port
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

## Testing the Integration

### 1. Verify Backend is Running
Visit `http://localhost:8000/docs` to access the API documentation and test endpoints.

### 2. Test Session Creation
```bash
curl -X POST "http://localhost:8000/api/session/create"
```

### 3. Test Chat Functionality
```bash
# First, create a session and note the session_id
SESSION_ID=$(curl -s -X POST "http://localhost:8000/api/session/create" | jq -r '.session_id')

# Then send a message
curl -X POST "http://localhost:8000/api/chat/$SESSION_ID" \
  -H "Content-Type: application/json" \
  -d '{"message": "What is this textbook about?", "text_content": "This is a sample textbook content..."}'
```

## Troubleshooting

### Common Issues

1. **"No session active" error**
   - Ensure the session creation endpoint is called before sending messages
   - Check that the session ID is being properly stored and passed

2. **CORS errors**
   - Verify that your frontend origin is allowed in the backend CORS configuration
   - Ensure the backend is running on the expected port

3. **API key issues**
   - Confirm that your Cohere and Qdrant API keys are valid and properly set in environment variables

4. **Connection issues**
   - Verify that the backend is running and accessible
   - Check that the frontend is configured to connect to the correct backend URL

## Next Steps

1. Customize the chat widget UI to match your textbook's design
2. Implement text extraction from the current textbook page to provide as context
3. Add additional error handling and user feedback mechanisms
4. Test with real textbook content to ensure proper functionality