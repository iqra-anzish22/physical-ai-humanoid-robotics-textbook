from fastapi import APIRouter, UploadFile, File, Form, HTTPException, Depends
from typing import Optional, List, Dict, Any
import tempfile
import os

from src.models.document import DocumentProcessingRequest, DocumentProcessingResponse
from src.services.document_loader import DocumentLoader
from src.services.document_processor import DocumentProcessor
from src.services.knowledge_pipeline_service import KnowledgePipelineOrchestrator
from src.services.embedding_service import EmbeddingService
from src.services.vector_storage_service import VectorStorageService
from src.utils.logger import logger

# Initialize router
router = APIRouter(prefix="/knowledge-pipeline", tags=["knowledge-pipeline"])

# Global service instances (in a real app, these would be dependency injected)
document_loader = DocumentLoader()
embedding_service = EmbeddingService()
vector_storage_service = VectorStorageService()
document_processor = DocumentProcessor(embedding_service)
knowledge_pipeline = KnowledgePipelineOrchestrator(
    document_loader=document_loader,
    document_processor=document_processor,
    vector_storage_service=vector_storage_service
)


@router.post("/upload", response_model=DocumentProcessingResponse)
async def upload_document(
    file: UploadFile = File(...),
    metadata: Optional[str] = Form(None)
) -> DocumentProcessingResponse:
    """
    Upload and process a document through the knowledge pipeline.

    This endpoint accepts a file upload, processes it through the knowledge pipeline
    (loading, chunking, embedding, and indexing), and returns the processing status.
    """
    try:
        # Validate file type
        file_ext = os.path.splitext(file.filename)[1].lower()
        if file_ext not in document_loader.SUPPORTED_FORMATS:
            raise HTTPException(
                status_code=400,
                detail=f"Unsupported file format: {file_ext}. Supported formats: {list(document_loader.SUPPORTED_FORMATS)}"
            )

        # Read file content
        content = await file.read()

        # Parse metadata if provided
        parsed_metadata = {}
        if metadata:
            try:
                import json
                parsed_metadata = json.loads(metadata)
            except json.JSONDecodeError:
                logger.warning(f"Could not parse metadata JSON for file {file.filename}, using empty metadata")

        # Process document through knowledge pipeline
        response = knowledge_pipeline.process_document_from_bytes(
            content_bytes=content,
            original_filename=file.filename,
            metadata=parsed_metadata
        )

        logger.info(f"Document {file.filename} processed successfully with ID {response.document_id}")
        return response

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing document upload {file.filename}: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing document: {str(e)}"
        )


@router.post("/upload-url", response_model=DocumentProcessingResponse)
async def upload_document_from_url(
    url: str = Form(...),
    metadata: Optional[str] = Form(None)
) -> DocumentProcessingResponse:
    """
    Process a document from a URL through the knowledge pipeline.

    This endpoint downloads a document from a URL and processes it through the knowledge pipeline.
    """
    try:
        # This would require implementing URL download functionality
        # For now, we'll raise a not implemented error
        raise HTTPException(
            status_code=501,
            detail="URL upload not implemented yet"
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error processing document from URL {url}: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing document from URL: {str(e)}"
        )


@router.post("/batch-upload")
async def batch_upload_documents(
    files: List[UploadFile] = File(...),
    metadata_list: Optional[str] = Form(None)
) -> List[DocumentProcessingResponse]:
    """
    Upload and process multiple documents in batch.

    This endpoint accepts multiple file uploads and processes them through the knowledge pipeline.
    """
    try:
        responses = []

        # Parse metadata list if provided
        parsed_metadata_list = []
        if metadata_list:
            try:
                import json
                parsed_metadata_list = json.loads(metadata_list)
                if not isinstance(parsed_metadata_list, list):
                    parsed_metadata_list = [parsed_metadata_list] * len(files)
            except json.JSONDecodeError:
                logger.warning("Could not parse metadata JSON list, using empty metadata for all files")

        # Process each file
        for i, file in enumerate(files):
            # Validate file type
            file_ext = os.path.splitext(file.filename)[1].lower()
            if file_ext not in document_loader.SUPPORTED_FORMATS:
                response = DocumentProcessingResponse(
                    document_id="",
                    processing_status="failed",
                    message=f"Unsupported file format: {file_ext}"
                )
                responses.append(response)
                continue

            # Read file content
            content = await file.read()

            # Get metadata for this file
            file_metadata = {}
            if i < len(parsed_metadata_list):
                file_metadata = parsed_metadata_list[i] or {}

            # Process document
            response = knowledge_pipeline.process_document_from_bytes(
                content_bytes=content,
                original_filename=file.filename,
                metadata=file_metadata
            )

            responses.append(response)

        logger.info(f"Batch upload completed for {len(files)} files")
        return responses

    except Exception as e:
        logger.error(f"Error processing batch document upload: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error processing batch upload: {str(e)}"
        )


@router.get("/status/{document_id}")
async def get_document_status(document_id: str):
    """
    Get the processing status of a document.

    This endpoint returns the current processing status of a document.
    """
    try:
        status = knowledge_pipeline.get_processing_status(document_id)
        if status is None:
            raise HTTPException(
                status_code=404,
                detail=f"Document with ID {document_id} not found"
            )

        return {
            "document_id": document_id,
            "status": status.value if hasattr(status, 'value') else status
        }
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting document status for ID {document_id}: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error getting document status: {str(e)}"
        )


@router.get("/supported-formats")
async def get_supported_formats() -> Dict[str, List[str]]:
    """
    Get the list of supported document formats.

    This endpoint returns the document formats that are supported by the knowledge pipeline.
    """
    try:
        formats = knowledge_pipeline.get_supported_formats()
        return {"supported_formats": formats}
    except Exception as e:
        logger.error(f"Error getting supported formats: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error getting supported formats: {str(e)}"
        )


@router.delete("/document/{document_id}")
async def delete_document(document_id: str):
    """
    Delete a processed document and its associated chunks.

    This endpoint removes a document and its chunks from the vector storage.
    """
    try:
        # In a real implementation, this would:
        # 1. Mark the document as deleted in the database
        # 2. Remove the document's chunks from vector storage
        # 3. Clean up any temporary files
        raise HTTPException(
            status_code=501,
            detail="Document deletion not implemented yet"
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error deleting document with ID {document_id}: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail=f"Error deleting document: {str(e)}"
        )