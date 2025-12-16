# Research: RAG Knowledge Pipeline

**Feature**: 002-rag-knowledge-pipeline
**Date**: 2025-12-12

## Decision: Backend Project Structure with uv

**Rationale**: uv is a modern, fast Python package manager that will provide efficient dependency management for the project. The user specifically requested using uv package manager for the backend folder initialization.

**Alternatives considered**:
- pip + venv (traditional approach)
- poetry (feature-rich but potentially slower)
- conda (good for data science but heavier)

## Decision: Docusaurus Website Deployment

**Rationale**: Docusaurus is a popular static site generator for documentation that's easy to deploy to GitHub Pages. It's well-suited for book content with good search capabilities and responsive design.

**Alternatives considered**:
- Jekyll (GitHub's default but less feature-rich for docs)
- Hugo (fast but steeper learning curve)
- Custom static site (more work than needed)

## Decision: Cohere for Embeddings

**Rationale**: Cohere provides high-quality embeddings with good performance and reliability. The user specifically requested Cohere for embedding generation.

**Alternatives considered**:
- OpenAI embeddings (costlier, API key requirements)
- Sentence Transformers (local but slower, requires model management)
- Hugging Face embeddings (similar to Sentence Transformers)

## Decision: Qdrant Cloud for Vector Storage

**Rationale**: Qdrant is a dedicated vector database that provides efficient similarity search. The cloud version offers scalability and managed infrastructure.

**Alternatives considered**:
- Pinecone (competitor but less familiar)
- Weaviate (alternative vector DB)
- PostgreSQL with pgvector (SQL-based but less optimized for vectors)

## Decision: Single main.py File Architecture

**Rationale**: The user explicitly requested that the implementation should be in only one file named main.py. This simplifies deployment and distribution.

**Alternatives considered**:
- Modular approach with separate files (more maintainable but violates user constraint)
- Package structure (more organized but violates user constraint)

## Decision: Document Processing Pipeline

**Rationale**: The pipeline will crawl the deployed website, extract text content, chunk it appropriately, generate embeddings, and store in Qdrant. This follows standard RAG pipeline patterns.

**Technical approach**:
- Use requests/beautifulsoup4 for web crawling to get-all-urls from http://localhost:3000/physical-ai-humanoid-robotics-textbook/
- Implement extract-text-from-url function for HTML content extraction
- Add PDF support using PyPDF2
- Implement chunk_text function with appropriate size and overlap
- Use Cohere API for embed function to generate embeddings
- Create collection named 'rag_embedding' in Qdrant
- Implement save_chunk_to_qdrant function to store embeddings with metadata
- All functions will be orchestrated in a main function

## Decision: Error Handling and Logging

**Rationale**: The pipeline must handle various failure scenarios gracefully and provide detailed logging for troubleshooting.

**Implementation approach**:
- Try-catch blocks around critical operations
- Comprehensive logging at different levels
- Progress tracking for long-running operations
- Validation of inputs and outputs at each stage