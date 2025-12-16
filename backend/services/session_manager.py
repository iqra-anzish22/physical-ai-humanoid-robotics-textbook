from ..models.session import ChatSession, SessionStatus
from datetime import datetime, timedelta
import uuid
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class SessionManager:
    def __init__(self):
        # In-memory storage for sessions (for development)
        # In production, this would use a persistent database
        self.sessions = {}

    def create_session(self) -> str:
        """
        Create a new chat session with a unique ID
        """
        session_id = str(uuid.uuid4())

        # Create session that expires in 1 hour
        created_at = datetime.utcnow()
        expires_at = created_at + timedelta(hours=1)

        session = ChatSession(
            session_id=session_id,
            created_at=created_at,
            expires_at=expires_at,
            status=SessionStatus.active,
            conversation_history=[]
        )

        self.sessions[session_id] = session
        logger.info(f"Session created: {session_id}")

        return session_id

    def get_session(self, session_id: str) -> ChatSession:
        """
        Get a session by ID
        """
        return self.sessions.get(session_id)

    def session_exists(self, session_id: str) -> bool:
        """
        Check if a session exists and is active
        """
        session = self.sessions.get(session_id)
        if not session:
            return False

        # Check if session has expired
        if session.status == SessionStatus.expired or session.expires_at < datetime.utcnow():
            self.expire_session(session_id)
            return False

        return True

    def expire_session(self, session_id: str):
        """
        Mark a session as expired
        """
        session = self.sessions.get(session_id)
        if session:
            session.status = SessionStatus.expired
            logger.info(f"Session expired: {session_id}")