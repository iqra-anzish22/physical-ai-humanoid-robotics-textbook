from pydantic import BaseModel
from datetime import datetime
from enum import Enum
from typing import List, Optional
from .message import UserQuery

class SessionStatus(str, Enum):
    active = "active"
    expired = "expired"

class ChatSession(BaseModel):
    session_id: str
    created_at: datetime
    expires_at: datetime
    status: SessionStatus
    conversation_history: Optional[List[UserQuery]] = []

    class Config:
        use_enum_values = True