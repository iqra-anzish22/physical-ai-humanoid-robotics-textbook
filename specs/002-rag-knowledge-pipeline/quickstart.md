# Quickstart: RAG Knowledge Pipeline

**Feature**: 002-rag-knowledge-pipeline
**Date**: 2025-12-12

## Prerequisites

- Python 3.11 or higher
- uv package manager
- Access to Cohere API (API key)
- Qdrant Cloud account and API key
- Deployed Docusaurus website URL

## Setup

### 1. Initialize the Backend Project

```bash
# Create backend directory
mkdir backend
cd backend

# Initialize project with uv
uv init
```

### 2. Install Dependencies

Create a `pyproject.toml` file with the required dependencies:

```toml
[project]
name = "rag-knowledge-pipeline"
version = "0.1.0"
description = "RAG Knowledge Pipeline for book content"
requires-python = ">=3.11"

dependencies = [
    "requests>=2.31.0",
    "beautifulsoup4>=4.12.0",
    "cohere>=4.0.0",
    "qdrant-client>=1.9.0",
    "python-dotenv>=1.0.0",
    "PyPDF2>=3.0.0",
    "pytest>=7.0.0"
]
```

Install dependencies using uv:
```bash
uv sync
```

### 3. Environment Configuration

Create a `.env` file in the backend directory:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url
BOOK_WEBSITE_URL=https://your-book-website.com
QDRANT_COLLECTION_NAME=book_embeddings
```

## Usage

### Run the Complete Pipeline

```bash
cd backend
python main.py --website-url "https://your-book-website.com" --process-all
```

### Run Specific Pipeline Stages

```bash
# Only crawl and extract documents
python main.py --crawl-only

# Only generate embeddings (requires extracted documents)
python main.py --embed-only

# Only store in Qdrant (requires embeddings)
python main.py --store-only
```

### Validation

Run the validation script to test the complete pipeline:

```bash
python main.py --validate
```

## Pipeline Functions

1. **get-all-urls**: Fetch all URLs from http://localhost:3000/physical-ai-humanoid-robotics-textbook/
2. **extract-text-from-url**: Extract text content from each URL's HTML/PDF content
3. **chunk_text**: Split large documents into smaller chunks with appropriate size and overlap
4. **embed**: Generate vector embeddings using Cohere API
5. **create_collection**: Create 'rag_embedding' collection in Qdrant
6. **save_chunk_to_qdrant**: Store embeddings in Qdrant with metadata
7. **main function**: Orchestrate all the above functions in the correct sequence

## Configuration Options

The pipeline can be configured via command-line arguments or environment variables:

- `--website-url`: Base URL of the book website to crawl
- `--chunk-size`: Size of text chunks for embedding (default: 1000 characters)
- `--chunk-overlap`: Overlap between chunks (default: 200 characters)
- `--collection-name`: Qdrant collection name (default: book_embeddings)
- `--batch-size`: Number of documents to process in each batch (default: 10)

## Validation

The validation process will:
1. Verify access to the book website
2. Test Cohere API connectivity
3. Test Qdrant connectivity
4. Run a small sample through the complete pipeline
5. Verify that embeddings can be retrieved from Qdrant