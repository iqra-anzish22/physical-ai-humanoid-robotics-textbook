// Docusaurus Chatbot Integration
class DocusaurusChatbot {
    constructor() {
        this.backendUrl = 'http://127.0.0.1:8001';
        this.sessionId = null;
        this.isProcessing = false;
        this.sessionInitialized = false;
        this.sessionInitPromise = null;
        this.initialize();
    }

    initialize() {
        this.createWidget();
        this.loadStyles();
        this.attachEventListeners();
        this.initSession(); // Initialize session separately to avoid blocking
    }

    createWidget() {
        // Create container if it doesn't exist
        let container = document.getElementById('docusaurus-chatbot-container');
        if (!container) {
            container = document.createElement('div');
            container.id = 'docusaurus-chatbot-container';
            document.body.appendChild(container);
        }

        container.innerHTML = `
            <div id="chatbot-widget" class="docusaurus-chatbot-container">
                <div class="chatbot-header">
                    <h3>AI Assistant</h3>
                    <button id="chatbot-toggle" class="chatbot-toggle-btn">▼</button>
                </div>
                <div id="chatbot-content" class="chatbot-content">
                    <div id="chatbot-messages" class="chatbot-messages">
                        <div class="message bot-message">
                            <strong>AI:</strong> Hello! I'm your AI assistant for this textbook. Ask me questions about the content on this page.
                        </div>
                    </div>
                    <div class="chatbot-input-container">
                        <input type="text" id="chatbot-input" placeholder="Ask a question about this page..." autocomplete="off">
                        <button id="chatbot-send" class="chatbot-send-btn">Send</button>
                    </div>
                    <div class="chatbot-session-info">
                        <small id="session-status">Initializing session...</small>
                    </div>
                </div>
            </div>
        `;
    }

    loadStyles() {
        // Only add styles if they don't already exist
        if (document.getElementById('docusaurus-chatbot-styles')) {
            return;
        }

        const style = document.createElement('style');
        style.id = 'docusaurus-chatbot-styles';
        style.textContent = `
            .docusaurus-chatbot-container {
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
        // Use event delegation to handle dynamically created elements
        document.addEventListener('click', (e) => {
            // Handle toggle button click
            if (e.target && e.target.id === 'chatbot-toggle') {
                this.toggleChatbot();
            }

            // Handle send button click
            if (e.target && e.target.id === 'chatbot-send') {
                this.sendMessage();
            }
        });

        // Handle input keypress
        document.addEventListener('keypress', (e) => {
            if (e.target && e.target.id === 'chatbot-input' && e.key === 'Enter') {
                this.sendMessage();
            }
        });
    }

    toggleChatbot() {
        const content = document.getElementById('chatbot-content');
        const toggleBtn = document.getElementById('chatbot-toggle');

        if (!content || !toggleBtn) return;

        const isHidden = content.classList.contains('chatbot-hidden');

        if (isHidden) {
            content.classList.remove('chatbot-hidden');
            toggleBtn.textContent = '▼';
        } else {
            content.classList.add('chatbot-hidden');
            toggleBtn.textContent = '▲';
        }
    }

    async initSession() {
        // Prevent multiple simultaneous session initializations
        if (this.sessionInitPromise) {
            return this.sessionInitPromise;
        }

        this.sessionInitPromise = this._createSessionInternal();
        return this.sessionInitPromise;
    }

    async _createSessionInternal() {
        if (this.sessionInitialized) {
            return; // Already initialized
        }

        try {
            const response = await fetch(`${this.backendUrl}/api/session/create`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                // Add timeout handling
                signal: AbortSignal.timeout(10000) // 10 second timeout
            });

            if (response.ok) {
                const data = await response.json();
                this.sessionId = data.session_id;
                this.sessionInitialized = true;

                // Update session status
                const statusElement = document.getElementById('session-status');
                if (statusElement) {
                    statusElement.textContent = `Session: ${this.sessionId.substring(0, 8)}...`;
                }

                // Add success message
                this.addMessage('AI', 'Session created! You can now ask questions about this page content.');
            } else {
                const error = await response.json();
                this.addMessage('System', `Error creating session: ${error.detail || 'Unknown error'}`);
                console.error('Session creation failed:', error);
            }
        } catch (error) {
            if (error.name === 'AbortError') {
                this.addMessage('System', 'Session creation timed out. Please check if the backend is running.');
            } else {
                this.addMessage('System', `Error creating session: ${error.message}`);
                console.error('Session creation error:', error);
            }
        } finally {
            this.sessionInitPromise = null;
        }
    }

    async sendMessage() {
        const input = document.getElementById('chatbot-input');
        if (!input) return;

        const message = input.value.trim();

        if (!message || this.isProcessing) return;

        // Wait for session to be initialized if not already
        if (!this.sessionInitialized) {
            this.addMessage('System', 'Initializing session, please wait...');
            await this.initSession();

            // Check if session was successfully created
            if (!this.sessionInitialized || !this.sessionId) {
                this.addMessage('System', 'Unable to create session. Please refresh the page or check if the backend is running.');
                return;
            }
        }

        // Add user message
        this.addMessage('You', message);
        input.value = '';
        this.isProcessing = true;

        const sendBtn = document.getElementById('chatbot-send');
        if (sendBtn) {
            sendBtn.disabled = true;
        }

        try {
            // Show loading message
            const loadingMsg = this.addMessage('AI', 'Thinking...', true);

            const response = await fetch(`${this.backendUrl}/api/chat/${this.sessionId}`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    message: message,
                    text_content: this.extractPageContent() // Extract current page content
                }),
                signal: AbortSignal.timeout(30000) // 30 second timeout for chat
            });

            // Remove loading message
            const messagesContainer = document.getElementById('chatbot-messages');
            if (loadingMsg && messagesContainer && messagesContainer.contains(loadingMsg)) {
                messagesContainer.removeChild(loadingMsg);
            }

            if (response.ok) {
                const data = await response.json();
                this.addMessage('AI', data.response);
            } else {
                const error = await response.json();
                this.addMessage('AI', `Error: ${error.detail || 'Unknown error'}`);
                console.error('Chat response error:', error);
            }
        } catch (error) {
            if (error.name === 'AbortError') {
                this.addMessage('AI', 'Request timed out. Please try again.');
            } else {
                this.addMessage('AI', `Error: ${error.message}`);
                console.error('Chat error:', error);
            }
        } finally {
            this.isProcessing = false;
            if (sendBtn) {
                sendBtn.disabled = false;
            }
        }
    }

    extractPageContent() {
        // Extract relevant content from the current page
        let content = '';

        // Try to get main content areas
        const mainContent = document.querySelector('main') ||
                           document.querySelector('.main') ||
                           document.querySelector('.container') ||
                           document.querySelector('article') ||
                           document.querySelector('body');

        if (mainContent) {
            // Get text content from various elements
            const selectors = 'h1, h2, h3, h4, h5, h6, p, li, .markdown, .markdown-body, .docItemContainer, .theme-doc-markdown';
            const textElements = mainContent.querySelectorAll(selectors);

            textElements.forEach(el => {
                // Skip if element is inside the chatbot widget
                if (el.closest('#chatbot-widget')) {
                    return;
                }
                content += el.textContent + ' ';
            });

            // Limit content to avoid overwhelming the API
            if (content.length > 5000) {
                content = content.substring(0, 5000);
            }
        }

        return content.trim() || "This is a textbook page. The user is reading content from this page and asking questions about it.";
    }

    addMessage(sender, text, isTemporary = false) {
        const messagesContainer = document.getElementById('chatbot-messages');
        if (!messagesContainer) return null;

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
        return null;
    }

    // Static method to initialize the widget
    static init() {
        if (!window.docusaurusChatbotInstance) {
            window.docusaurusChatbotInstance = new DocusaurusChatbot();
        }
        return window.docusaurusChatbotInstance;
    }
}

// Auto-initialize when DOM is loaded
document.addEventListener('DOMContentLoaded', function() {
    // Initialize with a small delay to ensure DOM is fully ready
    setTimeout(() => {
        DocusaurusChatbot.init();
    }, 500);
});

// Handle page navigation in SPAs
if (window.Docusaurus) {
    window.Docusaurus.onRouteUpdate = function() {
        // Re-initialize if needed after route change
        if (!window.docusaurusChatbotInstance) {
            setTimeout(() => {
                DocusaurusChatbot.init();
            }, 500);
        }
    };
}