# Implementation Tasks: RAG Pipeline

**Feature**: RAG Embeddings Pipeline | **Branch**: `1-rag-embeddings-pipeline` | **Date**: 2025-12-31

## Summary

- **Total Tasks**: 22
- **Parallel Opportunities**: 4 tasks marked [P]
- **Completed**: 22/22 (100%)
- **Status**: All tasks completed

---

## Dependencies Graph

```
Phase 1 (Setup)
    │
    ▼
Phase 2 (Config) ────────┐
    │                    │
    ▼                    ▼
T003 (main.py)    T004-T008 (modules)
    │                    │
    └────────────────────┘
            │
            ▼
    Phase 3 (US1: Crawler)
            │
            ▼
    Phase 4 (US2: Chunker)
            │
            ▼
    Phase 5 (US3: Embedder)
            │
            ▼
    Phase 6 (US4: Qdrant Store)
            │
            ▼
    Phase 7 (US5+US6: Orchestrator)
```

---

## Phase 1: Project Setup

- [x] T001 Create `backend/` directory structure per plan.md
- [x] T002 [P] Create `backend/pyproject.toml` with UV config and dependencies

## Phase 2: Configuration

- [x] T003 Create `backend/src/config.py` with PipelineConfig using pydantic.BaseSettings

## Phase 3: User Story 1 - Content Ingestion (Priority: P1)

**Goal**: Fetch and extract text from configured URLs

**Independent Test**: Run `python main.py --urls https://example.com` and verify extracted text saved to file

- [x] T004 [P] [US1] Create `backend/src/crawler.py` with URL fetching and HTML extraction
- [x] T005 [P] [US1] Implement `Crawler.extract_page()` using BeautifulSoup for main content
- [x] T006 [P] [US1] Add error handling for 404s, timeouts, and auth failures in crawler

## Phase 4: User Story 2 - Content Chunking (Priority: P1)

**Goal**: Split content into semantic chunks with metadata

**Independent Test**: Verify chunk size limits and metadata completeness

- [x] T007 [P] [US2] Create `backend/src/chunker.py` with token-based chunking using tiktoken
- [x] T008 [P] [US2] Implement `Chunker.chunk()` with max_tokens=512 and overlap=50
- [x] T009 [P] [US2] Add metadata (source_url, section_path, heading, chunk_index) to each chunk

## Phase 5: User Story 3 - Embedding Generation (Priority: P1)

**Goal**: Generate consistent embeddings via Cohere API

**Independent Test**: Verify fixed-dimension output and deterministic results

- [x] T010 [P] [US3] Create `backend/src/embedder.py` with Cohere client integration
- [x] T011 [P] [US3] Implement `Embedder.embed()` with batch processing (batch_size=96)
- [x] T012 [P] [US3] Add retry logic with exponential backoff for API rate limits

## Phase 6: User Story 4 - Vector Storage (Priority: P1)

**Goal**: Store embeddings in Qdrant with proper indexing

**Independent Test**: Verify vectors are searchable with similarity scores

- [x] T013 [P] [US4] Create `backend/src/qdrant_store.py` with Qdrant client
- [x] T014 [P] [US4] Implement collection creation with HNSW index and payload indexes
- [x] T015 [P] [US4] Add idempotent upsert using SHA-256 content hashing for point IDs

## Phase 7: Polish & Cross-Cutting (Priority: P1 + P2)

**Goal**: Orchestrate pipeline, verify results, enable idempotent runs

- [x] T016 [US5] Create `backend/main.py` with `main()` orchestrator function
- [x] T017 [US5] Implement pipeline flow: fetch → chunk → embed → store
- [x] T018 [US5] Add logging and PipelineState tracking for audit/debug
- [x] T019 [US6] Implement `--verify` flag to confirm vector storage and sample search
- [x] T020 [US6] Add graceful error handling with per-URL error tracking

## Testing

- [x] T021 [P] Create `backend/tests/conftest.py` with pytest fixtures for mocking
- [x] T022 [P] Write unit tests for chunker, embedder, and qdrant_store modules

---

## Files Created

```
backend/
├── pyproject.toml           # UV package configuration
├── .env.example             # Environment variable template
├── main.py                  # Single entry point with main() orchestrator
├── src/
│   ├── __init__.py
│   ├── config.py            # Configuration loading from env
│   ├── crawler.py           # URL fetching and HTML extraction
│   ├── chunker.py           # Text chunking with metadata
│   ├── embedder.py          # Cohere embedding generation
│   └── qdrant_store.py      # Qdrant collection and upsert logic
└── tests/
    ├── conftest.py          # Pytest fixtures
    ├── test_chunker.py      # Chunking tests
    ├── test_embedder.py     # Embedding tests
    └── test_qdrant.py       # Storage tests
```

---

## Usage

```bash
# Install dependencies
cd backend
uv sync

# Configure environment
cp .env.example .env
# Edit .env with your API keys

# Run the pipeline
python main.py

# Verify installation
python main.py --verify
```
