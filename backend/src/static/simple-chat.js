// Simple Chat Component for Book Integration
class SimpleChatComponent {
  constructor(containerId, backendUrl = 'http://127.0.0.1:8001') {
    this.containerId = containerId;
    this.backendUrl = backendUrl;
    this.messages = [];
    this.init();
  }

  init() {
    this.createChatInterface();
    this.loadStyles();
  }

  createChatInterface() {
    const container = document.getElementById(this.containerId);
    if (!container) {
      console.error(`Container with ID '${this.containerId}' not found`);
      return;
    }

    container.innerHTML = `
      <div id="simple-chat-container" class="simple-chat-container">
        <div class="simple-chat-header">
          <h3>AI Assistant</h3>
        </div>
        <div id="simple-chat-messages" class="simple-chat-messages">
          <div class="simple-message simple-bot-message">
            <strong>AI:</strong> Hello! I'm your AI assistant for this book. Ask me questions about AI, robotics, or the content of this textbook.
          </div>
        </div>
        <div class="simple-chat-input-container">
          <input type="text" id="simple-user-input" placeholder="Ask a question..." />
          <button id="simple-send-btn">Send</button>
        </div>
      </div>
    `;

    this.attachEventListeners();
  }

  loadStyles() {
    // Only add styles if they don't already exist
    if (document.getElementById('simple-chat-styles')) {
      return;
    }

    const style = document.createElement('style');
    style.id = 'simple-chat-styles';
    style.textContent = `
      .simple-chat-container {
        max-width: 600px;
        margin: 20px auto;
        border: 1px solid #ddd;
        border-radius: 8px;
        box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        display: flex;
        flex-direction: column;
        font-family: Arial, sans-serif;
        background: white;
      }

      .simple-chat-header {
        background: #4f46e5;
        color: white;
        padding: 10px 15px;
        border-top-left-radius: 7px;
        border-top-right-radius: 7px;
      }

      .simple-chat-header h3 {
        margin: 0;
        font-size: 16px;
      }

      .simple-chat-messages {
        flex: 1;
        padding: 15px;
        height: 300px;
        overflow-y: auto;
        display: flex;
        flex-direction: column;
        gap: 10px;
      }

      .simple-message {
        max-width: 85%;
        padding: 8px 12px;
        border-radius: 18px;
        font-size: 14px;
        line-height: 1.4;
      }

      .simple-user-message {
        align-self: flex-end;
        background: #4f46e5;
        color: white;
      }

      .simple-bot-message {
        align-self: flex-start;
        background: #f3f4f6;
        color: #374151;
      }

      .simple-chat-input-container {
        display: flex;
        padding: 10px;
        border-top: 1px solid #eee;
        background: white;
      }

      #simple-user-input {
        flex: 1;
        padding: 8px 12px;
        border: 1px solid #d1d5db;
        border-radius: 4px;
        font-size: 14px;
        outline: none;
      }

      #simple-user-input:focus {
        border-color: #4f46e5;
        box-shadow: 0 0 0 2px rgba(79, 70, 229, 0.1);
      }

      #simple-send-btn {
        padding: 8px 16px;
        background: #4f46e5;
        color: white;
        border: none;
        border-radius: 4px;
        margin-left: 8px;
        cursor: pointer;
        font-size: 14px;
      }

      #simple-send-btn:hover {
        background: #4338ca;
      }

      #simple-send-btn:disabled {
        background: #9ca3af;
        cursor: not-allowed;
      }
    `;
    document.head.appendChild(style);
  }

  attachEventListeners() {
    const input = document.getElementById('simple-user-input');
    const sendBtn = document.getElementById('simple-send-btn');

    sendBtn.addEventListener('click', () => this.sendMessage());
    input.addEventListener('keypress', (e) => {
      if (e.key === 'Enter') {
        this.sendMessage();
      }
    });
  }

  async sendMessage() {
    const input = document.getElementById('simple-user-input');
    const message = input.value.trim();

    if (!message) return;

    // Add user message
    this.addMessage('You', message);
    input.value = '';

    try {
      // Show typing indicator
      const typingIndicator = this.addMessage('AI', 'Thinking...', true);

      // Send to backend
      const response = await fetch(`${this.backendUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ question: message })
      });

      // Remove typing indicator
      this.removeMessage(typingIndicator);

      if (response.ok) {
        const data = await response.json();
        this.addMessage('AI', data.answer);
      } else {
        const error = await response.json();
        this.addMessage('AI', `Error: ${error.detail || 'Unknown error'}`);
      }
    } catch (error) {
      this.addMessage('AI', `Error: ${error.message}`);
    }
  }

  addMessage(sender, text, isTemporary = false) {
    const messagesContainer = document.getElementById('simple-chat-messages');
    const messageDiv = document.createElement('div');

    messageDiv.className = `simple-message ${sender === 'You' ? 'simple-user-message' : 'simple-bot-message'}`;

    if (isTemporary) {
      messageDiv.classList.add('simple-loading');
    }

    messageDiv.innerHTML = `<strong>${sender}:</strong> ${text}`;
    messagesContainer.appendChild(messageDiv);

    // Scroll to bottom
    messagesContainer.scrollTop = messagesContainer.scrollHeight;

    if (isTemporary) {
      return messageDiv;
    }
  }

  removeMessage(messageElement) {
    const messagesContainer = document.getElementById('simple-chat-messages');
    if (messageElement && messagesContainer.contains(messageElement)) {
      messagesContainer.removeChild(messageElement);
    }
  }
}

// Auto-initialize if container exists
document.addEventListener('DOMContentLoaded', function() {
  const container = document.getElementById('simple-chat-container');
  if (container) {
    window.simpleChat = new SimpleChatComponent('simple-chat-container');
  }
});