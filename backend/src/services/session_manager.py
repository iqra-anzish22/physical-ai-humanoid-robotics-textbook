import uuid
from datetime import datetime, timedelta
from typing import Dict, Optional, List
import logging
from ..config import settings


class Session:
    def __init__(self, session_id: str):
        self.session_id = session_id
        self.created_at = datetime.utcnow()
        self.expires_at = self.created_at + timedelta(minutes=settings.session_timeout)
        self.status = "active"
        self.chunk_count = 0
        self.conversation_history = []
        self.text_content = ""  # Store only temporarily during processing

    def is_expired(self) -> bool:
        """Check if the session has expired."""
        return datetime.utcnow() > self.expires_at

    def extend_session(self):
        """Extend the session by the timeout period."""
        self.expires_at = datetime.utcnow() + timedelta(minutes=settings.session_timeout)

    def mark_completed(self):
        """Mark the session as completed."""
        self.status = "completed"

    def mark_expired(self):
        """Mark the session as expired."""
        self.status = "expired"

    def add_message_to_history(self, role: str, content: str):
        """Add a message to the conversation history."""
        message = {
            "role": role,
            "content": content,
            "timestamp": datetime.utcnow().isoformat()
        }
        self.conversation_history.append(message)

    def get_conversation_history(self) -> list:
        """Get the conversation history."""
        return self.conversation_history


class SessionManager:
    def __init__(self):
        self.sessions: Dict[str, Session] = {}

    def create_session(self) -> str:
        """
        Create a new session with a unique ID.

        Returns:
            The session ID
        """
        session_id = str(uuid.uuid4())
        session = Session(session_id)
        self.sessions[session_id] = session
        logging.info(f"Created new session: {session_id}")
        return session_id

    def get_session(self, session_id: str) -> Optional[Session]:
        """
        Get a session by its ID.

        Args:
            session_id: The session ID to retrieve

        Returns:
            The Session object if found and not expired, None otherwise
        """
        session = self.sessions.get(session_id)
        if session:
            if session.is_expired():
                self.delete_session(session_id)
                return None
            return session
        return None

    def update_session_chunks(self, session_id: str, chunk_count: int):
        """
        Update the chunk count for a session.

        Args:
            session_id: The session ID
            chunk_count: The number of chunks
        """
        session = self.get_session(session_id)
        if session:
            session.chunk_count = chunk_count

    def delete_session(self, session_id: str) -> bool:
        """
        Delete a session.

        Args:
            session_id: The session ID to delete

        Returns:
            True if deleted, False if not found
        """
        if session_id in self.sessions:
            del self.sessions[session_id]
            logging.info(f"Deleted session: {session_id}")
            return True
        return False

    def cleanup_expired_sessions(self):
        """
        Remove all expired sessions from memory.
        """
        expired_sessions = []
        for session_id, session in self.sessions.items():
            if session.is_expired():
                expired_sessions.append(session_id)

        for session_id in expired_sessions:
            del self.sessions[session_id]
            logging.info(f"Cleaned up expired session: {session_id}")

    def get_active_sessions_count(self) -> int:
        """
        Get the count of active sessions.

        Returns:
            Number of active sessions
        """
        self.cleanup_expired_sessions()  # Clean up before counting
        count = 0
        for session in self.sessions.values():
            if not session.is_expired():
                count += 1
        return count