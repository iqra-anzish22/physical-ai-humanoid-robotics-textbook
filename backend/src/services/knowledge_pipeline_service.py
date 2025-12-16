import asyncio
import threading
from typing import List, Optional, Dict, Any
from concurrent.futures import ThreadPoolExecutor
import queue

from src.models.document import Document, ProcessingStatus, DocumentProcessingRequest, DocumentProcessingResponse
from src.services.document_loader import DocumentLoader
from src.services.document_processor import DocumentProcessor
from src.services.vector_storage_service import VectorStorageService
from src.utils.logger import logger


class KnowledgePipelineOrchestrator:
    """Orchestrates the entire knowledge pipeline: loading, processing, and indexing documents."""

    def __init__(
        self,
        document_loader: DocumentLoader,
        document_processor: DocumentProcessor,
        vector_storage_service: VectorStorageService
    ):
        self.document_loader = document_loader
        self.document_processor = document_processor
        self.vector_storage_service = vector_storage_service
        self.processing_queue = queue.Queue()
        self.active_processes = {}

    def process_document_upload(
        self,
        file_path: str,
        original_filename: str,
        metadata: Optional[Dict[str, Any]] = None
    ) -> DocumentProcessingResponse:
        """
        Process a document upload: load, process, and index.

        Args:
            file_path: Path to the uploaded file
            original_filename: Original filename
            metadata: Additional metadata

        Returns:
            Document processing response
        """
        try:
            # Load the document
            document = self.document_loader.load_document_from_file(
                file_path=file_path,
                original_filename=original_filename,
                metadata=metadata
            )

            # Process the document (create chunks)
            chunks = self.document_processor.process_document(document)

            # Index the chunks in vector storage
            self.vector_storage_service.store_chunks(chunks)

            # Mark document as indexed
            document.mark_indexed()

            response = DocumentProcessingResponse(
                document_id=document.id,
                processing_status=document.processing_status,
                message=f"Successfully processed and indexed document: {original_filename}"
            )

            logger.info(f"Knowledge pipeline completed for document {document.id}")
            return response

        except Exception as e:
            logger.error(f"Error in knowledge pipeline for {original_filename}: {str(e)}")

            # Create a minimal document to track the error
            error_doc = Document(
                filename=file_path,
                original_filename=original_filename,
                content="",
                content_type="",
                size=0,
                checksum=""
            )
            error_doc.mark_failed(str(e))

            response = DocumentProcessingResponse(
                document_id=error_doc.id,
                processing_status=error_doc.processing_status,
                message=f"Failed to process document {original_filename}: {str(e)}"
            )

            return response

    def process_document_from_bytes(
        self,
        content_bytes: bytes,
        original_filename: str,
        metadata: Optional[Dict[str, Any]] = None
    ) -> DocumentProcessingResponse:
        """
        Process a document from bytes: load, process, and index.

        Args:
            content_bytes: Raw document bytes
            original_filename: Original filename
            metadata: Additional metadata

        Returns:
            Document processing response
        """
        try:
            # Load the document
            document = self.document_loader.load_document_from_bytes(
                content_bytes=content_bytes,
                original_filename=original_filename,
                metadata=metadata
            )

            # Process the document (create chunks)
            chunks = self.document_processor.process_document(document)

            # Index the chunks in vector storage
            self.vector_storage_service.store_chunks(chunks)

            # Mark document as indexed
            document.mark_indexed()

            response = DocumentProcessingResponse(
                document_id=document.id,
                processing_status=document.processing_status,
                message=f"Successfully processed and indexed document: {original_filename}"
            )

            logger.info(f"Knowledge pipeline completed for document {document.id}")
            return response

        except Exception as e:
            logger.error(f"Error in knowledge pipeline for {original_filename}: {str(e)}")

            # Create a minimal document to track the error
            error_doc = Document(
                filename="",
                original_filename=original_filename,
                content="",
                content_type="",
                size=0,
                checksum=""
            )
            error_doc.mark_failed(str(e))

            response = DocumentProcessingResponse(
                document_id=error_doc.id,
                processing_status=error_doc.processing_status,
                message=f"Failed to process document {original_filename}: {str(e)}"
            )

            return response

    def process_documents_batch(
        self,
        file_paths: List[str],
        metadata_list: Optional[List[Dict[str, Any]]] = None
    ) -> List[DocumentProcessingResponse]:
        """
        Process multiple documents in batch.

        Args:
            file_paths: List of file paths to process
            metadata_list: Optional list of metadata for each document

        Returns:
            List of document processing responses
        """
        responses = []

        for i, file_path in enumerate(file_paths):
            original_filename = file_path.split('/')[-1]  # Extract filename from path
            metadata = metadata_list[i] if metadata_list and i < len(metadata_list) else {}

            response = self.process_document_upload(file_path, original_filename, metadata)
            responses.append(response)

        return responses

    def process_document_async(
        self,
        file_path: str,
        original_filename: str,
        metadata: Optional[Dict[str, Any]] = None
    ) -> str:
        """
        Process a document asynchronously.

        Args:
            file_path: Path to the document file
            original_filename: Original filename
            metadata: Additional metadata

        Returns:
            Document ID for tracking the processing job
        """
        # Add to processing queue
        job_data = {
            'file_path': file_path,
            'original_filename': original_filename,
            'metadata': metadata
        }

        document_id = self.processing_queue.put(job_data)
        thread = threading.Thread(
            target=self._process_job,
            args=(job_data, document_id)
        )
        thread.start()

        # Store thread reference for monitoring
        self.active_processes[document_id] = thread

        return document_id

    def _process_job(self, job_data: Dict[str, Any], document_id: str):
        """Internal method to process a queued job."""
        try:
            response = self.process_document_upload(
                file_path=job_data['file_path'],
                original_filename=job_data['original_filename'],
                metadata=job_data['metadata']
            )
            logger.info(f"Async processing completed for document {document_id}: {response.message}")
        except Exception as e:
            logger.error(f"Async processing failed for document {document_id}: {str(e)}")
        finally:
            # Clean up thread reference
            if document_id in self.active_processes:
                del self.active_processes[document_id]

    def get_processing_status(self, document_id: str) -> Optional[ProcessingStatus]:
        """Get the processing status of a document (if available)."""
        # In a real implementation, this would query a database or cache
        # For now, we'll return None since we don't have persistence
        return None

    def cleanup_document_resources(self, document: Document):
        """Clean up resources associated with a processed document."""
        try:
            # Clean up temporary files
            self.document_loader.cleanup_temp_files(document)

            logger.info(f"Cleaned up resources for document {document.id}")
        except Exception as e:
            logger.error(f"Error cleaning up resources for document {document.id}: {str(e)}")

    def validate_document_for_processing(self, file_path: str) -> bool:
        """Validate if a document is suitable for processing."""
        return self.document_loader.validate_document_format(file_path)

    def get_supported_formats(self) -> List[str]:
        """Get list of supported document formats."""
        return list(self.document_loader.SUPPORTED_FORMATS)