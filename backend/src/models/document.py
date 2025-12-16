from datetime import datetime
from enum import Enum
from typing import Optional, Dict, Any
from pydantic import BaseModel, Field
import uuid


class ProcessingStatus(str, Enum):
    """Enumeration for document processing status."""
    UPLOADED = "uploaded"
    PROCESSING = "processing"
    PROCESSED = "processed"
    FAILED = "failed"
    INDEXED = "indexed"


class Document(BaseModel):
    """Model representing a document in the knowledge pipeline."""
    id: str = Field(default_factory=lambda: str(uuid.uuid4()))
    filename: str
    original_filename: str
    content: str
    content_type: str
    size: int
    checksum: str
    metadata: Dict[str, Any] = Field(default_factory=dict)
    processing_status: ProcessingStatus = ProcessingStatus.UPLOADED
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    processed_at: Optional[datetime] = None
    error_message: Optional[str] = None

    def mark_processing(self):
        """Mark the document as processing."""
        self.processing_status = ProcessingStatus.PROCESSING
        self.updated_at = datetime.utcnow()

    def mark_processed(self):
        """Mark the document as processed."""
        self.processing_status = ProcessingStatus.PROCESSED
        self.processed_at = datetime.utcnow()
        self.updated_at = datetime.utcnow()

    def mark_indexed(self):
        """Mark the document as indexed."""
        self.processing_status = ProcessingStatus.INDEXED
        self.updated_at = datetime.utcnow()

    def mark_failed(self, error_message: str):
        """Mark the document as failed."""
        self.processing_status = ProcessingStatus.FAILED
        self.error_message = error_message
        self.updated_at = datetime.utcnow()


class DocumentProcessingRequest(BaseModel):
    """Request model for document processing."""
    filename: str
    content_type: str = "application/pdf"
    metadata: Dict[str, Any] = Field(default_factory=dict)


class DocumentProcessingResponse(BaseModel):
    """Response model for document processing."""
    document_id: str
    processing_status: ProcessingStatus
    message: str


class DocumentQuery(BaseModel):
    """Model for querying documents."""
    query: str
    limit: int = 10
    filters: Dict[str, Any] = Field(default_factory=dict)