# Implementation Plan: RAG Agent with OpenAI Agents SDK

**Branch**: `003-rag-agent` | **Date**: 2025-12-31 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-rag-agent/spec.md`

## Summary

This feature implements a RAG (Retrieval-Augmented Generation) agent using the OpenAI Agents SDK that answers questions about the Physical AI and Humanoid Robotics book. The agent connects to the validated retrieval pipeline from Spec-2 to fetch relevant content, then generates grounded responses using the LLM. Single-agent architecture with clear separation between agent logic and retrieval, suitable for backend API integration.

## Technical Context

**Language/Version**: Python 3.11+ (existing project uses Python 3.12)
**Primary Dependencies**: OpenAI Agents SDK, qdrant-client, cohere, python-dotenv
**Storage**: Qdrant Cloud (existing `book-rag-embeddings` collection)
**Testing**: pytest, manual validation script execution
**Target Platform**: Linux/Windows server (CLI tool + API ready)
**Project Type**: Single script utility (`backend/agent.py`)
**Performance Goals**: Response time under 3 seconds for 90% of requests (SC-005)
**Constraints**: OpenAI Agents SDK only (no custom LLM wrappers), single-agent architecture, retrieval from Spec-2 pipeline only
**Scale/Scope**: Book content with 53 chunks, single-agent for Q&A interactions

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Check | Status |
|-----------|-------|--------|
| I. Accuracy Through Verification | Agent responses grounded in retrieved content with source attribution | PASS |
| II. Clarity for Developer Audience | Agent outputs clear answers with source URLs for verification | PASS |
| III. Spec-Driven Consistency | Plan follows spec → plan → tasks workflow per constitution | PASS |
| IV. Reproducibility | Agent is runnable with documented queries, responses traceable to sources | PASS |
| V. AI-Assisted Rigor with Human Validation | Uses OpenAI Agents SDK (verified SDK), responses verified against retrieval | PASS |

**Result**: All gates PASS - proceeding to Phase 1

## Project Structure

### Documentation (this feature)

```text
specs/003-rag-agent/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (not needed - all tech choices defined)
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Single script structure for RAG agent
backend/
├── agent.py             # Main RAG agent (ALL agent logic)
├── retrieve.py          # Retrieval pipeline from Spec-2
├── src/
│   ├── config.py        # Configuration loading
│   ├── embedder.py      # Cohere embedding
│   └── qdrant_store.py  # Qdrant storage
├── .env                 # Configuration (already exists)
└── pyproject.toml       # Dependencies (already exists)
```

**Structure Decision**: Single-file agent (`backend/agent.py`) as specified in the planning input. Leverages existing Spec-2 retrieval pipeline for content fetching. Uses OpenAI Agents SDK's built-in tool system for retrieval integration.

## Data Model

### Entities for RAG Agent

**RAGAgent**: Main agent that orchestrates retrieval and response generation
- `system_prompt`: Instructions for how agent should behave
- `retriever`: Reference to retrieval pipeline
- `conversation_history`: Message history for context

**ConversationMessage**: Single message in conversation
- `role`: "user" or "assistant"
- `content`: Text content
- `sources`: List of retrieved chunks used for this message
- `timestamp`: When message was created

**RetrievedChunk**: Content from vector database (from Spec-2)
- `id`: Unique point identifier
- `score`: Similarity score (0-1)
- `text`: Chunk content text
- `source_url`: Original documentation URL
- `section_path`: Hierarchical path in documentation
- `heading`: Page heading

**AgentResponse**: Final output from agent
- `answer_text`: Generated response
- `sources`: List of source URLs used
- `confidence`: Indicator of response quality
- `latency_ms`: Response generation time

## Implementation Details

### Agent Initialization

The OpenAI Agent will be initialized with:
- System instructions defining its role as a book Q&A assistant
- A retrieval tool powered by the Spec-2 pipeline
- Instructions to always cite sources in responses

### Retrieval Integration

The agent will use the OpenAI Agents SDK's tool system:
1. User query is received
2. Agent invokes retrieval tool with the query
3. Retrieved chunks are returned to agent
4. Agent generates response grounded in the chunks
5. Response includes source attribution

### API Design

```
POST /agent/chat
{
  "message": "Explain embodied intelligence",
  "session_id": "optional",
  "max_results": 5,
  "threshold": 0.5
}

Response:
{
  "answer": "Based on the book...",
  "sources": [
    {"url": "...", "heading": "...", "score": 0.71}
  ],
  "session_id": "...",
  "latency_ms": 1234
}
```

## Quickstart

```bash
# Install dependencies
cd backend
pip install openai-agents

# Run the agent
python agent.py "Explain humanoid robot kinematics"

# Interactive mode
python agent.py --interactive

# API mode (when server is running)
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "What is VLA?"}'
```

## Complexity Tracking

No constitution violations - the implementation follows all principles:
- Single-file agent is simple and focused
- Uses existing retrieval infrastructure without duplication
- Agent logic and retrieval are cleanly separated
- Responses are verifiable through source attribution

## Quick Reference

**Agent Type**: OpenAI Agents SDK single agent
**Retrieval Source**: Spec-2 pipeline (`backend/retrieve.py`)
**Vector Database**: Qdrant Cloud (`book-rag-embeddings`)
**Embedding Model**: Cohere embed-english-v3.0
**Output**: CLI responses + API-ready JSON
**Test Queries**: "Explain embodied intelligence", "What is VLA?", "How does ROS 2 work?"
