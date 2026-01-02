# Implementation Tasks: RAG Agent with OpenAI Agents SDK

**Feature**: RAG Agent Construction | **Branch**: `003-rag-agent` | **Date**: 2025-12-31

## Summary

- **Total Tasks**: 10
- **Parallel Opportunities**: 2 tasks marked [P]
- **Status**: Pending implementation

---

## Dependencies Graph

```
Phase 1 (Setup)
    │
    ▼
T001-T002 (Dependencies & Config)
    │
    ▼
Phase 2 (Foundational)
    │
    ▼
T003-T004 (Agent Core)
    │
    ▼
Phase 3 (US1 - Basic Q&A)
    │
    ▼
Phase 4 (US2 - Context)
    │
    ▼
Phase 5 (US3 - API)
    │
    ▼
Phase 6 (Polish)
```

---

## Phase 1: Setup

- [X] T001 Install openai-agents package in backend/pyproject.toml
- [X] T002 [P] Create .env template with OPENAI_API_KEY placeholder

---

## Phase 2: Foundational

- [X] T003 [P] [US1] Create backend/agent.py with CLI argument parsing (--query, --interactive, --api)
- [X] T004 [P] [US1] Create retrieval wrapper class in backend/agent.py to interface with retrieve.py

---

## Phase 3: User Story 1 - Ask Questions About Book Content (P1)

**Goal**: Agent answers book questions with source attribution

**Independent Test**: Run `python agent.py "Explain embodied intelligence"` and verify response cites sources

- [X] T005 [US1] Implement OpenAI Agent initialization with system prompt in backend/agent.py
- [X] T006 [US1] Create retrieval tool using OpenAI Agents SDK Tool class in backend/agent.py
- [X] T007 [US1] Implement response generation with source citation formatting in backend/agent.py
- [X] T008 [US1] Handle out-of-scope queries with "cannot answer" response in backend/agent.py

---

## Phase 4: User Story 2 - Context-Aware Follow-up Questions (P2)

**Goal**: Agent maintains conversation context across turns

**Independent Test**: Run interactive mode and verify follow-up responses reference earlier context

- [X] T009 [US2] Implement conversation history management in backend/agent.py

---

## Phase 5: User Story 3 - API Integration for Backend Services (P3)

**Goal**: Expose agent via REST API for backend integration

**Independent Test**: Make API call and verify response format matches contracts/agent-api.yaml

- [X] T010 [US3] Add FastAPI server with /chat endpoint in backend/agent.py

---

## Phase 6: Polish & Cross-Cutting

- [ ] T011 Validate agent with sample queries: "Explain embodied intelligence", "What is VLA?", "How does ROS 2 work?"
- [ ] T012 Update quickstart.md with usage examples

---

## Files Created

```
backend/agent.py              # Main RAG agent (all logic)
backend/.env.template         # Environment template
```

---

## Usage

```bash
# Setup
cd backend
pip install openai-agents

# Run single query
python agent.py "Explain embodied intelligence"

# Interactive mode
python agent.py --interactive

# API server
python agent.py --api
```
