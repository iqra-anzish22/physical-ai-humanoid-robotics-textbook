import asyncio
import requests
import json
from datetime import datetime

# Test configuration
BASE_URL = "http://localhost:8000"

def test_session_creation():
    """Test the session creation endpoint"""
    print("Testing session creation endpoint...")

    try:
        response = requests.post(f"{BASE_URL}/api/session/create", json={})
        print(f"Status Code: {response.status_code}")

        if response.status_code == 200:
            data = response.json()
            print(f"Session ID: {data.get('session_id')}")
            print("✓ Session creation endpoint working correctly")
            return data.get('session_id')
        else:
            print(f"✗ Session creation failed with status: {response.status_code}")
            print(f"Response: {response.text}")
            return None
    except Exception as e:
        print(f"✗ Error testing session creation: {str(e)}")
        return None

def test_chat_endpoint(session_id):
    """Test the chat endpoint"""
    print("\nTesting chat endpoint...")

    if not session_id:
        print("✗ No session ID provided, cannot test chat endpoint")
        return False

    try:
        payload = {
            "session_id": session_id,
            "message": "Hello, how are you?",
            "text_content": "This is sample textbook content for testing purposes."
        }

        response = requests.post(f"{BASE_URL}/api/chat", json=payload)
        print(f"Status Code: {response.status_code}")

        if response.status_code == 200:
            data = response.json()
            print(f"Response: {data.get('answer', 'No answer field')}")
            print("✓ Chat endpoint working correctly")
            return True
        elif response.status_code == 404:
            print("✗ Chat endpoint returned 404 - Session not found")
            return False
        else:
            print(f"✗ Chat endpoint failed with status: {response.status_code}")
            print(f"Response: {response.text}")
            return False
    except Exception as e:
        print(f"✗ Error testing chat endpoint: {str(e)}")
        return False

def test_health_endpoint():
    """Test the health check endpoint"""
    print("\nTesting health endpoint...")

    try:
        response = requests.get(f"{BASE_URL}/health")
        print(f"Status Code: {response.status_code}")

        if response.status_code == 200:
            data = response.json()
            print(f"Health check response: {data}")
            print("✓ Health endpoint working correctly")
            return True
        else:
            print(f"✗ Health check failed with status: {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Error testing health endpoint: {str(e)}")
        return False

def test_404_handling():
    """Test that non-existent endpoints return 404"""
    print("\nTesting 404 error handling...")

    try:
        response = requests.get(f"{BASE_URL}/nonexistent/endpoint")
        print(f"Status Code: {response.status_code}")

        if response.status_code == 404:
            print("✓ 404 errors handled correctly for non-existent endpoints")
            return True
        else:
            print(f"✗ Expected 404 but got {response.status_code}")
            return False
    except Exception as e:
        print(f"✗ Error testing 404 handling: {str(e)}")
        return False

def main():
    print("Starting API endpoint tests...\n")

    # Test health endpoint first
    health_ok = test_health_endpoint()

    if not health_ok:
        print("\n✗ Backend server does not appear to be running. Please start the server with:")
        print("   cd backend")
        print("   uvicorn main:app --reload")
        return

    # Test session creation
    session_id = test_session_creation()

    # Test chat endpoint if session was created
    if session_id:
        chat_ok = test_chat_endpoint(session_id)
    else:
        chat_ok = False

    # Test 404 handling
    test_404_handling()

    print("\n" + "="*50)
    print("Test Summary:")
    print(f"  Health endpoint: {'✓' if health_ok else '✗'}")
    print(f"  Session creation: {'✓' if session_id is not None else '✗'}")
    print(f"  Chat endpoint: {'✓' if chat_ok else '✗'}")
    print("="*50)

if __name__ == "__main__":
    main()