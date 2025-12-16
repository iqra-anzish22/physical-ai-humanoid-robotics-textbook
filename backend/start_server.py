#!/usr/bin/env python3
"""
Startup script for the Secure AI Chatbot Backend
"""

import subprocess
import sys
import os
from pathlib import Path

def main():
    # Change to the backend directory
    backend_dir = Path(__file__).parent
    os.chdir(backend_dir)

    print("Starting Secure AI Chatbot Backend...")
    print("Loading environment variables and validating configuration...")

    try:
        # Import and validate settings to ensure environment is properly configured
        from config import settings
        print("✓ Environment variables loaded successfully")
    except Exception as e:
        print(f"✗ Error loading environment: {e}")
        print("Please ensure you have a .env file with the required variables:")
        print("  - QDRANT_API_KEY")
        print("  - QDRANT_URL")
        print("  - COHERE_API_KEY")
        print("  - NEON_DATABASE_URL (optional)")
        sys.exit(1)

    print("\nStarting server with uvicorn...")
    print("Server will be available at http://localhost:8000")
    print("Press Ctrl+C to stop the server\n")

    # Start the uvicorn server
    try:
        subprocess.run([
            sys.executable, "-m", "uvicorn",
            "main:app",
            "--host", "0.0.0.0",
            "--port", "8000",
            "--reload"
        ], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error starting server: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nServer stopped by user")
        sys.exit(0)

if __name__ == "__main__":
    main()