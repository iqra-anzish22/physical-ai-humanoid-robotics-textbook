from pydantic import BaseModel
from datetime import datetime
from enum import Enum
from typing import List, Optional

class QueryStatus(str, Enum):
    pending = "pending"
    processing = "processing"
    completed = "completed"
    failed = "failed"

class UserQuery(BaseModel):
    query_id: str
    session_id: str
    content: str
    timestamp: datetime
    processed_status: QueryStatus

class AIResponse(BaseModel):
    response_id: str
    query_id: str
    content: str
    timestamp: datetime
    source_chunks: Optional[List[str]] = []