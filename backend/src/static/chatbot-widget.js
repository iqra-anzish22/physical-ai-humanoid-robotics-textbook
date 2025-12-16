// RAG Chatbot Widget for Book Integration
class RAGChatbot {
    constructor(options = {}) {
        this.apiUrl = options.apiUrl || 'http://127.0.0.1:8001/api/v1';
        this.containerId = options.containerId || 'rag-chatbot-container';
        this.backendUrl = options.backendUrl || 'http://127.0.0.1:8001';
        this.sessionId = null;
        this.isProcessing = false;
        this.initialize();
    }

    initialize() {
        this.createWidget();
        this.loadStyles();
        this.createSession();
    }

    createWidget() {
        const container = document.getElementById(this.containerId);
        if (!container) {
            console.error(`Container with ID '${this.containerId}' not found`);
            return;
        }

        container.innerHTML = `
            <div id="chatbot-widget" class="rag-chatbot-container">
                <div class="chatbot-header">
                    <h3>AI Assistant</h3>
                    <button id="chatbot-toggle" class="chatbot-toggle-btn">▼</button>
                </div>
                <div id="chatbot-content" class="chatbot-content">
                    <div id="chatbot-messages" class="chatbot-messages">
                        <div class="message bot-message">
                            <strong>AI:</strong> Hello! I'm your AI assistant for this book. Ask me questions about AI, robotics, or the content of this textbook.
                        </div>
                    </div>
                    <div class="chatbot-input-container">
                        <input type="text" id="chatbot-input" placeholder="Ask a question about this book..." autocomplete="off">
                        <button id="chatbot-send" class="chatbot-send-btn">Send</button>
                    </div>
                    <div class="chatbot-session-info">
                        <small id="session-status">Initializing...</small>
                    </div>
                </div>
            </div>
        `;

        this.attachEventListeners();
    }

    loadStyles() {
        // Only add styles if they don't already exist
        if (document.getElementById('rag-chatbot-styles')) {
            return;
        }

        const style = document.createElement('style');
        style.id = 'rag-chatbot-styles';
        style.textContent = `
            .rag-chatbot-container {
                position: fixed;
                bottom: 20px;
                right: 20px;
                width: 350px;
                height: 500px;
                border: 1px solid #ddd;
                border-radius: 10px;
                box-shadow: 0 4px 12px rgba(0,0,0,0.15);
                background: white;
                z-index: 10000;
                display: flex;
                flex-direction: column;
                font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            }

            .chatbot-header {
                background: #4f46e5;
                color: white;
                padding: 10px 15px;
                border-top-left-radius: 9px;
                border-top-right-radius: 9px;
                display: flex;
                justify-content: space-between;
                align-items: center;
                cursor: pointer;
            }

            .chatbot-header h3 {
                margin: 0;
                font-size: 14px;
            }

            .chatbot-toggle-btn {
                background: none;
                border: none;
                color: white;
                font-size: 16px;
                cursor: pointer;
                padding: 0;
                margin: 0;
            }

            .chatbot-content {
                flex: 1;
                display: flex;
                flex-direction: column;
                overflow: hidden;
            }

            .chatbot-messages {
                flex: 1;
                padding: 15px;
                overflow-y: auto;
                display: flex;
                flex-direction: column;
                gap: 10px;
            }

            .message {
                max-width: 85%;
                padding: 8px 12px;
                border-radius: 18px;
                font-size: 14px;
                line-height: 1.4;
            }

            .user-message {
                align-self: flex-end;
                background: #4f46e5;
                color: white;
            }

            .bot-message {
                align-self: flex-start;
                background: #f3f4f6;
                color: #374151;
            }

            .chatbot-input-container {
                display: flex;
                padding: 10px;
                border-top: 1px solid #eee;
                background: white;
            }

            #chatbot-input {
                flex: 1;
                padding: 8px 12px;
                border: 1px solid #d1d5db;
                border-radius: 4px;
                font-size: 14px;
                outline: none;
            }

            #chatbot-input:focus {
                border-color: #4f46e5;
                box-shadow: 0 0 0 2px rgba(79, 70, 229, 0.1);
            }

            .chatbot-send-btn {
                padding: 8px 16px;
                background: #4f46e5;
                color: white;
                border: none;
                border-radius: 4px;
                margin-left: 8px;
                cursor: pointer;
                font-size: 14px;
            }

            .chatbot-send-btn:disabled {
                background: #9ca3af;
                cursor: not-allowed;
            }

            .chatbot-session-info {
                padding: 5px 10px;
                background: #f9fafb;
                border-top: 1px solid #e5e7eb;
                font-size: 11px;
                color: #6b7280;
            }

            .chatbot-hidden {
                display: none;
            }

            .chatbot-loading {
                font-style: italic;
                color: #6b7280;
            }
        `;
        document.head.appendChild(style);
    }

    attachEventListeners() {
        const toggleBtn = document.getElementById('chatbot-toggle');
        const input = document.getElementById('chatbot-input');
        const sendBtn = document.getElementById('chatbot-send');
        const widget = document.getElementById('chatbot-widget');

        toggleBtn.addEventListener('click', () => {
            const content = document.getElementById('chatbot-content');
            const isVisible = !content.classList.contains('chatbot-hidden');

            if (isVisible) {
                content.classList.add('chatbot-hidden');
                toggleBtn.textContent = '▲';
            } else {
                content.classList.remove('chatbot-hidden');
                toggleBtn.textContent = '▼';
            }
        });

        sendBtn.addEventListener('click', () => this.sendMessage());
        input.addEventListener('keypress', (e) => {
            if (e.key === 'Enter') {
                this.sendMessage();
            }
        });
    }

    async createSession() {
        try {
            const response = await fetch(`${this.backendUrl}/api/v1/session`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    text: "This is a session for the AI textbook. The user is reading about physical AI and humanoid robotics. Questions will be about these topics.",
                    chunk_size: 600,
                    overlap: 100
                })
            });

            if (response.ok) {
                const data = await response.json();
                this.sessionId = data.session_id;
                document.getElementById('session-status').textContent = `Session: ${this.sessionId.substring(0, 8)}...`;
                this.addMessage('AI', 'Session created! You can now ask questions about the book content.');
            } else {
                const error = await response.json();
                this.addMessage('System', `Error creating session: ${error.detail || 'Unknown error'}`);
            }
        } catch (error) {
            this.addMessage('System', `Error creating session: ${error.message}`);
        }
    }

    async sendMessage() {
        const input = document.getElementById('chatbot-input');
        const message = input.value.trim();

        if (!message || this.isProcessing) return;

        if (!this.sessionId) {
            this.addMessage('System', 'Please wait, initializing session...');
            return;
        }

        // Add user message
        this.addMessage('You', message);
        input.value = '';
        this.isProcessing = true;
        document.getElementById('chatbot-send').disabled = true;

        try {
            // Show loading message
            const loadingMsg = this.addMessage('AI', 'Thinking...', true);

            const response = await fetch(`${this.backendUrl}/api/v1/session/${this.sessionId}/query`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    question: message
                })
            });

            // Remove loading message
            const messagesContainer = document.getElementById('chatbot-messages');
            if (loadingMsg && messagesContainer.contains(loadingMsg)) {
                messagesContainer.removeChild(loadingMsg);
            }

            if (response.ok) {
                const data = await response.json();
                this.addMessage('AI', data.response);
            } else {
                const error = await response.json();
                this.addMessage('AI', `Error: ${error.detail || 'Unknown error'}`);
            }
        } catch (error) {
            this.addMessage('AI', `Error: ${error.message}`);
        } finally {
            this.isProcessing = false;
            document.getElementById('chatbot-send').disabled = false;
        }
    }

    addMessage(sender, text, isTemporary = false) {
        const messagesContainer = document.getElementById('chatbot-messages');
        const messageDiv = document.createElement('div');

        messageDiv.className = `message ${sender === 'You' ? 'user-message' : 'bot-message'}`;

        if (isTemporary) {
            messageDiv.classList.add('chatbot-loading');
        }

        messageDiv.innerHTML = `<strong>${sender}:</strong> ${text}`;
        messagesContainer.appendChild(messageDiv);

        // Scroll to bottom
        messagesContainer.scrollTop = messagesContainer.scrollHeight;

        if (isTemporary) {
            return messageDiv;
        }
    }

    // Static method to initialize the widget
    static init(options) {
        return new RAGChatbot(options);
    }
}

// Auto-initialize if container exists
document.addEventListener('DOMContentLoaded', function() {
    const container = document.getElementById('rag-chatbot-container');
    if (container) {
        window.ragChatbot = RAGChatbot.init();
    }
});