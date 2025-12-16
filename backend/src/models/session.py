from pydantic import BaseModel
from typing import Optional
from datetime import datetime
from enum import Enum


class SessionStatus(str, Enum):
    active = "active"
    completed = "completed"
    expired = "expired"


class TextSession(BaseModel):
    session_id: str
    created_at: datetime
    expires_at: datetime
    status: SessionStatus
    chunk_count: Optional[int] = 0
    conversation_history: Optional[list] = []