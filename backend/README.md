# RAG Pipeline

A pipeline that ingests content from Docusaurus book URLs, generates embeddings using Cohere's embedding API, and stores vectors in Qdrant Cloud for downstream retrieval.

## Installation

```bash
cd backend
uv sync
```

## Configuration

Copy the example environment file and configure your API keys:

```bash
cp .env.example .env
# Edit .env with your credentials
```

## Usage

```bash
# Run the pipeline
python main.py

# Run verification
python main.py --verify

# Process specific URLs
python main.py --urls https://example.com/docs
```

## Features

- URL ingestion with HTML content extraction
- Semantic text chunking with metadata
- Cohere embedding generation with retry logic
- Qdrant Cloud vector storage with HNSW indexing
- Idempotent upsert operations
- Pipeline orchestration and verification
