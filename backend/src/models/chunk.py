from pydantic import BaseModel
from typing import Optional, List, Dict, Any
from datetime import datetime
import uuid


class TextChunk(BaseModel):
    """Chunk model for session-based RAG (original implementation)."""
    chunk_id: str
    session_id: str
    content: str
    embedding_vector: Optional[List[float]] = None
    position: int
    token_count: Optional[int] = 0
    created_at: Optional[datetime] = None


class Chunk(BaseModel):
    """General chunk model for knowledge pipeline."""
    id: str
    text: str
    document_id: str
    metadata: Dict[str, Any] = {}
    embedding_vector: Optional[List[float]] = None
    position: Optional[int] = 0
    token_count: Optional[int] = 0
    created_at: Optional[datetime] = None

    def __init__(self, **data):
        if 'id' not in data:
            data['id'] = str(uuid.uuid4())
        if 'created_at' not in data:
            data['created_at'] = datetime.utcnow()
        super().__init__(**data)