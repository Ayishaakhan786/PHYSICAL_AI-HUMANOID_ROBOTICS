---
id: 002
title: rag-chatbot-integration-plan-creation
stage: plan
date: 2026-01-01
surface: agent
model: claude-sonnet-4.5-20250929
feature: 001-rag-chatbot-integration
branch: 001-rag-chatbot-integration
user: user
command: /sp.plan
labels: ["plan-creation", "rag", "chatbot", "integration", "fastapi", "docusaurus"]
links:
  spec: specs/001-rag-chatbot-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/001-rag-chatbot-integration/plan.md
 - specs/001-rag-chatbot-integration/research.md
 - specs/001-rag-chatbot-integration/data-model.md
 - specs/001-rag-chatbot-integration/quickstart.md
 - specs/001-rag-chatbot-integration/contracts/chat-api.yaml
tests:
 - N/A (planning phase)
---

## Prompt

Spec-4: Backend-Frontend Integration

- Frontend: Use existing Docusaurus book UI; add a Chatbot UI component inside the `book` folder without altering existing pages
- Backend setup: In `backend/`, create a single `api.py` file and initialize FastAPI
- API design: Expose one `/chat` endpoint that accepts user query + optional selected text
- Agent integration: Import and use the retrieval agent from `agent.py` to generate grounded responses
- Validation: Verify local end-to-end flow (Docusaurus UI → FastAPI → Agent → Response)

## Response snapshot

Completed implementation plan for RAG chatbot integration including:

**Phase 0 - Research** (`research.md`):
- API design decision: Single `/chat` endpoint with optional streaming via SSE
- Frontend integration: Docusaurus swizzling to inject ChatWidget
- Session management: sessionStorage with in-memory backend
- Error handling: Toast notifications with exponential backoff
- Agent integration: Direct import from agent.py, singleton via FastAPI lifespan
- All 8 research questions resolved with documented rationales

**Phase 1 - Design Artifacts**:
- `data-model.md`: Complete entity definitions (ChatRequest, ChatResponse, SourceCitation, ChatSession, ErrorMessage)
- `contracts/chat-api.yaml`: OpenAPI 3.0 spec with /chat and /health endpoints
- `quickstart.md`: Step-by-step guide for local dev setup and testing

**Plan Structure** (`plan.md`):
- Constitution check: All 5 principles PASSED
- Technical context: Python 3.11+ (backend), React (frontend), FastAPI, OpenAI Agents SDK, Qdrant, Cohere, Docusaurus
- Backend architecture diagram showing request flow from API → RAGAgent → Retrieval → Qdrant
- Frontend architecture diagram showing Docusaurus → ChatWidget → HTTP Client → FastAPI
- Component hierarchy: ChatWidget with ChatMessages, ChatInput, SourceCitation subcomponents
- Error handling strategy (frontend toast notifications, backend validation)
- Session management (sessionStorage + in-memory dict)
- Performance optimizations (connection pooling, caching, async operations, lazy loading)
- Security considerations (input validation, CORS configuration)
- Testing strategy (pytest backend, manual/automated frontend)
- Success criteria validation plan for all 8 spec criteria

**Project Structure**:
- Backend: New `backend/api.py` (FastAPI), `backend/tests/` (test_api.py, test_agent_integration.py)
- Frontend: New `book/src/components/ChatWidget/` (index.jsx, ChatInput.jsx, ChatMessages.jsx, SourceCitation.jsx, chatWidget.css), `book/theme/DocPage/` (swizzled layout)
- All additions, no modifications to existing structure

**Agent Context Updated**:
- Added FastAPI, React, Docusaurus to CLAUDE.md technology stack
- Configuration preserved for manual additions

## Outcome

- ✅ Impact: Complete architectural design with all research resolved, contracts defined, and implementation path clear
- 🧪 Tests: Testing strategy defined (pytest backend, manual frontend), quickstart includes validation scenarios
- 📁 Files: 5 new files created (plan.md, research.md, data-model.md, quickstart.md, chat-api.yaml)
- 🔁 Next prompts: /sp.tasks to generate implementation tasks, /sp.implement to execute implementation
- 🧠 Reflection: Plan addresses all spec requirements with minimal complexity. Constitution principles fully satisfied. No ADRs needed at this stage - all decisions documented in research.md. Ready for task generation and implementation.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Constitution check PASSED (all 5 principles), design quality validated against spec
- Prompt variant: N/A (first generation)
- Next experiment (smallest change to try): Consider ADR creation during implementation if significant architectural decisions emerge (e.g., API versioning, database-backed sessions, streaming alternatives)
