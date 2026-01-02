# Quickstart: RAG Pipeline

This guide covers setting up and running the RAG embedding pipeline.

## Prerequisites

- Python 3.11 or higher
- UV package manager (install via `pip install uv`)
- Cohere API key (sign up at [cohere.com](https://cohere.com))
- Qdrant Cloud account (sign up at [qdrant.tech](https://qdrant.tech))

## Installation

### 1. Clone and Setup

```bash
cd backend/
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv sync
```

### 2. Configure Environment

Copy the example environment file and fill in your credentials:

```bash
cp .env.example .env
```

Edit `.env` with your API keys:

```env
# Required
QDRANT_URL=https://your-cluster.qdrant.cloud:6333
QDRANT_API_KEY=your-qdrant-api-key
COHERE_API_KEY=your-cohere-api-key
SOURCE_URLS=https://book.example.com/docs,https://book.example.com/api

# Optional (defaults shown)
COLLECTION_NAME=book-rag-embeddings
EMBEDDING_MODEL=embed-english-v3.0
CHUNK_MAX_TOKENS=512
CHUNK_OVERLAP=50
REQUEST_TIMEOUT=30.0
BATCH_SIZE=96
LOG_LEVEL=INFO
```

## Usage

### Run the Full Pipeline

```bash
python main.py
```

This executes:
1. URL fetching and content extraction
2. Text chunking with metadata
3. Embedding generation via Cohere
4. Vector storage in Qdrant Cloud

### Run Verification

```bash
python main.py --verify
```

Verifies:
- Collection exists and is accessible
- Vector count matches expected
- Sample similarity search returns results

### Run with Custom URLs

```bash
python main.py --urls https://docs.example.com/page1 https://docs.example.com/page2
```

## Monitoring

Logs are written to stdout with configurable verbosity:

```bash
# Debug mode
LOG_LEVEL=DEBUG python main.py

# Quiet mode
LOG_LEVEL=WARNING python main.py
```

## Troubleshooting

### Qdrant Connection Failed

- Verify `QDRANT_URL` is correct (format: `https://<cluster>.qdrant.cloud:6333`)
- Ensure your API key has write permissions
- Check network connectivity to Qdrant Cloud

### Cohere API Errors

- Verify `COHERE_API_KEY` is valid
- Check API rate limits (Free Tier: 100K tokens/month)
- Review Cohere status at [status.cohere.ai](https://status.cohere.ai)

### Empty Results

- Check that target URLs are publicly accessible
- Verify robots.txt allows crawling
- Review logs for extraction errors

## Expected Output

```
2025-12-31 10:00:00 INFO Starting RAG pipeline
2025-12-31 10:00:00 INFO Fetching 5 URLs
2025-12-31 10:00:05 INFO Extracted 150 chunks from https://example.com/docs/intro
2025-12-31 10:00:06 INFO Extracted 200 chunks from https://example.com/docs/getting-started
2025-12-31 10:00:10 INFO Generated 350 embeddings (2 batches)
2025-12-31 10:00:12 INFO Stored 350 vectors in Qdrant
2025-12-31 10:00:12 INFO Pipeline complete: 5 URLs, 350 chunks, 350 vectors in 12.5s
```
