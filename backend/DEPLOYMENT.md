# Deployment Guide for AI Native Book Backend

This document provides comprehensive instructions for deploying the AI Native Book backend application.

## Table of Contents
1. [Prerequisites](#prerequisites)
2. [Environment Setup](#environment-setup)
3. [Installation](#installation)
4. [Configuration](#configuration)
5. [Running the Application](#running-the-application)
6. [Production Deployment](#production-deployment)
7. [Monitoring and Maintenance](#monitoring-and-maintenance)
8. [Troubleshooting](#troubleshooting)

## Prerequisites

Before deploying the application, ensure your system meets the following requirements:

### System Requirements
- **Operating System**: Linux, macOS, or Windows
- **Python**: Version 3.8 or higher
- **Memory**: Minimum 2GB RAM (4GB+ recommended)
- **Storage**: At least 500MB free space
- **Architecture**: x86_64 or ARM64

### External Dependencies
- **Qdrant Vector Database**: Cloud instance or self-hosted
- **Cohere API Key**: For embeddings and generation
- **Docker** (optional, for containerized deployment)

## Environment Setup

### 1. Python Environment
```bash
# Create a virtual environment
python -m venv venv

# Activate the virtual environment
# On Linux/macOS:
source venv/bin/activate
# On Windows:
venv\Scripts\activate

# Upgrade pip
pip install --upgrade pip
```

### 2. Clone the Repository
```bash
git clone <repository-url>
cd backend
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

## Installation

### 1. Create Environment File
Create a `.env` file in the backend root directory with the following content:

```env
# Qdrant Configuration
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_url_here

# Cohere Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Database Configuration (if using Neon)
NEON_DATABASE_URL=your_neon_database_url_here

# Application Configuration
MAX_TEXT_LENGTH=100000
DEFAULT_CHUNK_SIZE=600
DEFAULT_OVERLAP=100
SESSION_TIMEOUT=30
VECTOR_SIZE=1024
```

### 2. Environment Variables Explained

| Variable | Description | Default | Required |
|----------|-------------|---------|----------|
| `QDRANT_API_KEY` | API key for Qdrant vector database | - | Yes |
| `QDRANT_URL` | URL for Qdrant instance | - | Yes |
| `COHERE_API_KEY` | API key for Cohere services | - | Yes |
| `NEON_DATABASE_URL` | Connection string for Neon PostgreSQL | - | No |
| `MAX_TEXT_LENGTH` | Maximum allowed text length in characters | 100000 | No |
| `DEFAULT_CHUNK_SIZE` | Default chunk size in tokens | 600 | No |
| `DEFAULT_OVERLAP` | Default overlap in tokens | 100 | No |
| `SESSION_TIMEOUT` | Session timeout in minutes | 30 | No |
| `VECTOR_SIZE` | Size of embedding vectors | 1024 | No |

## Configuration

### 1. Qdrant Setup
1. Sign up for Qdrant Cloud at [qdrant.tech](https://qdrant.tech/)
2. Create a new cluster or use an existing one
3. Get your API key and cluster URL
4. Update your `.env` file with the credentials

### 2. Cohere Setup
1. Sign up for Cohere at [cohere.ai](https://cohere.ai/)
2. Get your API key from the dashboard
3. Update your `.env` file with the API key

### 3. Optional: Neon Database Setup
1. Sign up for Neon at [neon.tech](https://neon.tech/)
2. Create a new project
3. Get the connection string
4. Update your `.env` file if using database features

## Running the Application

### 1. Development Mode
```bash
# Activate virtual environment
source venv/bin/activate  # Linux/macOS
# or
venv\Scripts\activate     # Windows

# Run the application
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### 2. Production Mode
```bash
# Using uvicorn with production settings
uvicorn src.main:app --host 0.0.0.0 --port 8000 --workers 4

# Or using gunicorn (install with: pip install "uvicorn[standard]")
gunicorn src.main:app:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

### 3. Using Docker
Create a `Dockerfile`:

```Dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 8000

CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Build and run:
```bash
docker build -t ai-native-book-backend .
docker run -p 8000:8000 --env-file .env ai-native-book-backend
```

## Production Deployment

### 1. Using a Process Manager (PM2)
```bash
# Install PM2 globally
npm install -g pm2

# Create an ecosystem file: ecosystem.config.js
cat > ecosystem.config.js << EOF
module.exports = {
  apps: [{
    name: 'ai-native-book-backend',
    script: 'uvicorn',
    args: 'src.main:app --host 0.0.0.0 --port 8000 --workers 4',
    instances: 1,
    autorestart: true,
    watch: false,
    max_memory_restart: '1G',
    env: {
      NODE_ENV: 'production'
    }
  }]
};
EOF

# Start the application
pm2 start ecosystem.config.js

# Save the process list
pm2 save

# Start on system boot
pm2 startup
```

### 2. Using Systemd (Linux)
Create a systemd service file at `/etc/systemd/system/ai-native-book.service`:

```ini
[Unit]
Description=AI Native Book Backend
After=network.target

[Service]
User=www-data
Group=www-data
WorkingDirectory=/path/to/your/backend
EnvironmentFile=/path/to/your/backend/.env
ExecStart=/path/to/your/venv/bin/uvicorn src.main:app --host 0.0.0.0 --port 8000 --workers 4
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

Enable and start the service:
```bash
sudo systemctl daemon-reload
sudo systemctl enable ai-native-book
sudo systemctl start ai-native-book
```

### 3. Environment Variables for Production
In production, consider using a more secure method for managing secrets:

```bash
# Using environment variables
export QDRANT_API_KEY="your_api_key"
export COHERE_API_KEY="your_cohere_key"

# Or using a secrets management tool like HashiCorp Vault
# Or using cloud-specific solutions like AWS Secrets Manager, Azure Key Vault, etc.
```

## Monitoring and Maintenance

### 1. Health Checks
The application provides a health check endpoint at `/health` that returns:
```json
{
  "status": "healthy",
  "timestamp": 1234567890.123
}
```

### 2. Logging
The application logs important events to stdout/stderr. Configure your deployment to capture these logs:

```bash
# Example log rotation configuration for systemd
sudo tee /etc/logrotate.d/ai-native-book << EOF
/var/log/ai-native-book/*.log {
    daily
    missingok
    rotate 52
    compress
    delaycompress
    notifempty
    copytruncate
}
EOF
```

### 3. Performance Monitoring
Monitor these key metrics:
- Response times for session creation and queries
- Memory and CPU usage
- Error rates
- Active sessions count

### 4. Session Cleanup
The application automatically cleans up expired sessions every 5 minutes. Sessions expire after 30 minutes by default (configurable via `SESSION_TIMEOUT`).

## Troubleshooting

### Common Issues

#### 1. Connection Errors
**Problem**: Cannot connect to Qdrant or Cohere
**Solution**:
- Verify API keys and URLs in your `.env` file
- Check network connectivity
- Ensure firewall rules allow outbound connections

#### 2. Memory Issues
**Problem**: Application crashes with memory errors
**Solution**:
- Increase available memory
- Reduce the number of workers
- Implement proper session cleanup

#### 3. Rate Limiting
**Problem**: API calls are being rate-limited
**Solution**:
- Check your Cohere and Qdrant usage limits
- Implement caching where appropriate
- Consider upgrading your service tiers

#### 4. Startup Issues
**Problem**: Application fails to start
**Solution**:
- Verify all required environment variables are set
- Check dependency installation
- Review error logs for specific details

### Debugging Tips

1. **Enable detailed logging**:
   ```bash
   export LOG_LEVEL=DEBUG
   ```

2. **Check application status**:
   ```bash
   curl http://localhost:8000/health
   ```

3. **Monitor resource usage**:
   ```bash
   # Check memory usage
   ps aux | grep uvicorn

   # Monitor in real-time
   top -p $(pgrep -f uvicorn)
   ```

### Support

For additional support:
- Check the application logs
- Review the API documentation at `/docs`
- Contact the development team
- Open an issue in the repository if you find a bug

---

**Note**: Always keep your API keys and sensitive information secure. Never commit them to version control.