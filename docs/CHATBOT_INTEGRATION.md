# Docusaurus Chatbot Integration

## Overview
This document explains how the AI chatbot is integrated into the Docusaurus textbook without modifying any existing content or layout.

## Files Created/Modified

### Frontend Files
- `docs/static/chatbot.js` - Main chatbot widget implementation
- `docs/docusaurus.config.js` - Added script injection configuration

### Backend Files
- `backend/src/api/chatbot_api.py` - API endpoints for chatbot functionality
- `backend/src/main.py` - CORS configuration and route inclusion

## API Endpoints Used
- `POST /api/session/create` - Create new chat sessions
- `POST /api/chat/{session_id}` - Send messages to chatbot
- `POST /api/process-text/{session_id}` - Process text content for RAG

## How It Works
1. Chatbot widget automatically initializes when the page loads
2. Automatically creates a session with the backend
3. Extracts page content to provide context for questions
4. Sends user questions to the backend API
5. Displays AI responses in the chat interface
6. Maintains conversation history within the session

## Configuration
- Backend URL: `http://127.0.0.1:8001`
- Widget appears as a floating panel in bottom-right corner
- Auto-hides when not in use
- Preserves all existing textbook functionality

## Troubleshooting
- If chatbot doesn't appear, ensure backend is running on port 8001
- Check browser console for CORS or network errors
- Verify API endpoints are accessible from frontend