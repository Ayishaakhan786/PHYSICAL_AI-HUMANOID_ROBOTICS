# Implementation Plan: Retrieval Pipeline Validation and Testing

**Branch**: `2-retrieval-validation` | **Date**: 2025-12-31 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/2-retrieval-validation/spec.md`

## Summary

This feature implements a standalone validation script (`retrieve.py`) that connects to the existing Qdrant Cloud collection and validates the RAG retrieval pipeline. The script performs semantic search queries against stored embeddings, verifies result relevance, checks metadata accuracy, and measures retrieval latency. All validation is read-only and uses existing infrastructure without re-embedding.

## Technical Context

**Language/Version**: Python 3.11+ (existing project uses Python 3.14, target 3.11+ for compatibility)
**Primary Dependencies**: qdrant-client, cohere, python-dotenv (already in project)
**Storage**: Qdrant Cloud (read-only connection to existing `book-rag-embeddings` collection)
**Testing**: Manual validation script execution, pytest optional for future expansion
**Target Platform**: Linux/Windows development environment (CLI tool)
**Project Type**: Single script utility (retrieve.py at repository root)
**Performance Goals**: p95 query latency < 3 seconds, per spec success criteria
**Constraints**: Read-only access, no data modification, no re-embedding, backend-only
**Scale/Scope**: 5+ test queries against ~54 stored chunks, validation only

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Check | Status |
|-----------|-------|--------|
| III. Spec-Driven Consistency | Plan follows spec → plan → tasks workflow | PASS |
| I. Accuracy Through Verification | Validation script verifies retrieval correctness | PASS |
| IV. Reproducibility | Script is runnable with documented test queries | PASS |
| V. AI-Assisted Rigor | Implementation validated against spec requirements | PASS |

**Result**: All gates PASS - proceeding to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/2-retrieval-validation/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Not needed - no unknowns to resolve
├── data-model.md        # Phase 1 output (inline in this plan)
├── quickstart.md        # Phase 1 output
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
# Single script structure for validation utility
retrieve.py              # Main validation script (ALL validation logic)
.env                     # Already exists, no changes needed
```

**Structure Decision**: Single-file script `retrieve.py` at repository root as specified in requirements. Leverages existing backend modules for configuration and embedding.

## Data Model

### Entities for Retrieval Validation

**TestQuery**: Represents a validation test case
- `query_text`: Natural language search string
- `expected_topics`: List of topics that should appear in results
- `min_results`: Minimum number of results expected (default: 3)
- `relevance_threshold`: Minimum similarity score (default: 0.5)

**RetrievalResult**: Represents a single chunk from search
- `id`: Unique point identifier from Qdrant
- `score`: Similarity score (0-1)
- `text`: Chunk content text
- `source_url`: Original documentation URL
- `section_path`: Hierarchical path in documentation
- `heading`: Page heading
- `chunk_index`: Position in chunking sequence

**ValidationOutcome**: Result of a single test query
- `query`: TestQuery that was executed
- `results`: List of RetrievalResult
- `passed`: Boolean indicating all criteria met
- `latency_ms`: Query execution time
- `errors`: List of validation failures if any

**ValidationReport**: Aggregate validation results
- `total_queries`: Number of test queries executed
- `passed`: Count of passing tests
- `failed`: Count of failing tests
- `avg_latency_ms`: Average query time
- `p95_latency_ms`: 95th percentile latency
- `outcomes`: List of ValidationOutcome

## Implementation Details

### Configuration Loading

The script will use the existing `.env` file and backend configuration:
- `QDRANT_URL`: Qdrant Cloud endpoint
- `QDRANT_API_KEY`: Authentication key
- `COHERE_API_KEY`: For query embedding generation
- `COLLECTION_NAME`: Defaults to `book-rag-embeddings`
- `EMBEDDING_MODEL`: Defaults to `embed-english-v3.0`

### Query Flow

```
User Input Query
       ↓
Generate embedding via Cohere (embed-english-v3.0)
       ↓
Execute Qdrant search (cosine similarity)
       ↓
Retrieve chunks with metadata
       ↓
Validate results (relevance, metadata, latency)
       ↓
Output validation report (JSON + console)
```

### Test Queries (5 predefined)

Based on actual documentation content from physical-ai-humanoid-robotics book:

1. "humanoid robot kinematics" - Tests kinematics/motion content
2. "Isaac Sim GPU simulation" - Tests module-4-isaac content
3. "vision language action models" - Tests module-5-vla content
4. "domain randomization sim-to-real" - Tests sim-to-real content
5. "human interaction humanoid robots" - Tests module-6-humanoid content

### Validation Checks

For each test query:
1. **Result Count**: At least `min_results` returned
2. **Relevance**: All results have score >= `relevance_threshold`
3. **Metadata**: All results have valid `source_url`, `section_path`, `heading`
4. **Latency**: Query completes within time limit

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No violations - the implementation follows all constitution principles:
- Single script approach is simple and focused
- Read-only validation doesn't risk data integrity
- Uses existing infrastructure without duplication
- Validation is reproducible and documented

## Quickstart

```bash
# Run validation with default 5 test queries
python retrieve.py

# Run with custom queries
python retrieve.py --query "your search query"

# Run with verbose output
python retrieve.py --verbose

# Output results to file
python retrieve.py --output results.json
```

## Phase 0: Research Status

**Research Required**: None

All technical decisions are already determined by existing infrastructure:
- Qdrant client usage patterns (from existing backend/src/qdrant_store.py)
- Cohere embedding API (from existing backend/src/embedder.py)
- Configuration loading (from existing backend/src/config.py)

**Result**: Proceeding directly to Phase 1
