# Implementation Plan: Backend–Frontend Integration for RAG Chatbot

**Branch**: `001-rag-chatbot-integration` | **Date**: 2026-01-01 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot-integration/spec.md`

## Summary

This plan defines the technical architecture for integrating a RAG-powered chatbot into the existing Docusaurus book. The solution adds a FastAPI backend server exposing a `/chat` endpoint and embeds a React chat widget into the book pages using Docusaurus swizzling. The backend integrates the existing RAG agent from `backend/agent.py` to generate grounded responses from retrieved book content.

## Technical Context

**Language/Version**: Python 3.11+ (backend), JavaScript/React (frontend)
**Primary Dependencies**: FastAPI (backend), OpenAI Agents SDK, Qdrant Client, Cohere API, React (frontend), Docusaurus
**Storage**: In-memory (sessions), Qdrant Cloud (vectors)
**Testing**: pytest for backend, Docusaurus dev server for frontend integration
**Target Platform**: Local development (localhost), web browser
**Project Type**: Web application with backend API and Docusaurus static site frontend
**Performance Goals**: <5 second response time for 95% of requests, support 10+ concurrent requests
**Constraints**: No authentication, single backend API file, no changes to existing book pages
**Scale/Scope**: Single book project, embedded chat component on all pages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle I: Accuracy Through Verification
✅ **PASS**: All responses grounded in retrieved book content with source citations. No hallucinations accepted.

### Principle II: Clarity for Developer Audience
✅ **PASS**: Chat interface and documentation target developers. Technical jargon explained in context.

### Principle III: Spec-Driven Consistency
✅ **PASS**: This plan follows spec.md requirements. Implementation will use spec-driven workflow.

### Principle IV: Reproducibility
✅ **PASS**: Quickstart guide provides reproducible local setup. Code examples are runnable and tested.

### Principle V: AI-Assisted Rigor with Human Validation
✅ **PASS**: AI assistant created this plan. Implementation will be human-validated before deployment.

**All gates passed. Proceeding with design.**

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot-integration/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
│   └── chat-api.yaml    # OpenAPI specification
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created yet)
```

### Source Code (repository root)

```text
backend/
├── api.py                   # NEW - FastAPI application with /chat endpoint
├── agent.py                 # EXISTING - RAGAgent from Spec-3
├── retrieve.py              # EXISTING - Retrieval validation
├── src/
│   ├── config.py            # EXISTING - Configuration loading
│   ├── chunker.py          # EXISTING - Text chunking
│   ├── embedder.py         # EXISTING - Embedding generation
│   └── qdrant_store.py    # EXISTING - Vector storage
└── tests/
    ├── test_api.py          # NEW - API endpoint tests
    └── test_agent_integration.py  # NEW - Agent integration tests

book/
├── src/
│   └── components/
│       └── ChatWidget/      # NEW - Chat UI component
│           ├── index.jsx    # Main chat widget component
│           ├── ChatInput.jsx     # Input field component
│           ├── ChatMessages.jsx  # Message display component
│           ├── SourceCitation.jsx  # Source citation component
│           └── chatWidget.css  # Component styles
├── theme/
│   └── DocPage/            # NEW - Swizzled DocPage
│       └── index.jsx    # Inject ChatWidget into all pages
└── package.json              # EXISTING
```

**Structure Decision**: Web application structure selected because:
1. Feature involves both backend API and frontend UI integration
2. Backend adds FastAPI server alongside existing Python scripts
3. Frontend adds React components to existing Docusaurus project
4. Minimal changes to existing structure - only additions
5. No new top-level directories needed (reuse `backend/` and `book/`)

## Phase 0 Research Summary

Research completed and documented in `research.md`. Key decisions:

| Research Question | Decision | Rationale |
|------------------|----------|-----------|
| Backend API design | Single `/chat` endpoint with optional streaming via query param | FastAPI native SSE support, simpler than WebSocket |
| Frontend integration | Docusaurus swizzling of DocPage to inject ChatWidget | No content file modifications, Docusaurus-native |
| Text selection context | Browser `window.getSelection()` with context menu | Native API, no dependencies needed |
| Session management | sessionStorage with in-memory backend sessions | No auth requirement, simple implementation |
| Error handling | Toast notifications with exponential backoff retry | Non-intrusive UX, standard pattern |
| Agent integration | Direct import from agent.py, singleton via FastAPI lifespan | Efficient reuse, minimal refactoring |
| Frontend UI framework | Plain React with inline styles | Minimal dependencies, Docusaurus compatible |
| CORS configuration | Wildcard for dev, specific origins for production | Flexible dev, secure production |

**All research questions resolved. No NEEDS CLARIFICATION markers.**

## Phase 1 Design

### API Contract

See `contracts/chat-api.yaml` for complete OpenAPI specification. Key endpoints:

- `POST /chat` - Submit chat query (JSON or streaming response)
- `GET /health` - Health check endpoint

### Data Models

See `data-model.md` for complete entity definitions. Key entities:

- **ChatRequest** - User query from frontend
- **ChatResponse** - Backend answer with sources
- **SourceCitation** - Referenced book section
- **ChatSession** - In-memory session tracking
- **ErrorMessage** - Error responses

### Backend Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     FastAPI Application                    │
│                                                           │
│  ┌──────────────┐         ┌──────────────┐            │
│  │ Pydantic     │         │              │            │
│  │ Models        │<────────│  /chat       │            │
│  │ (Validation) │         │  Endpoint    │            │
│  └──────────────┘         └──────┬───────┘            │
│                                   │                       │
│                          ┌────────▼────────┐            │
│                          │  Request         │            │
│                          │  Handler        │            │
│                          └────────┬────────┘            │
│                                   │                       │
│                          ┌────────▼────────┐            │
│                          │  RAGAgent       │            │
│                          │  (Singleton)     │            │
│                          └────────┬────────┘            │
│                                   │                       │
│                          ┌────────▼────────┐            │
│                          │  Retrieval       │            │
│                          │  Tool           │            │
│                          └────────┬────────┘            │
│                                   │                       │
│                          ┌────────▼────────┐            │
│                          │  Qdrant Client  │            │
│                          │  + Cohere       │            │
│                          └────────┬────────┘            │
│                                   │                       │
└──────────────────────────┬───────────┘                    │
                           │                                │
                  ┌────────▼────────┐                      │
                  │  Vector DB      │                      │
                  │  (Qdrant)       │                      │
                  └─────────────────┘                      │
```

### Frontend Architecture

```
┌─────────────────────────────────────────────────────────────┐
│              Docusaurus Application                       │
│                                                           │
│  ┌──────────────┐         ┌──────────────┐            │
│  │ Swizzled     │         │              │            │
│  │ DocPage      │────────>│  ChatWidget   │            │
│  │ (Layout)     │ Inject  │  (Component)  │            │
│  └──────────────┘         └──────┬───────┘            │
│                                   │                       │
│                          ┌────────▼────────┐            │
│                          │  React State   │            │
│                          │  (messages,    │            │
│                          │   loading,     │            │
│                          │   error)       │            │
│                          └────────┬────────┘            │
│                                   │                       │
│                          ┌────────▼────────┐            │
│                          │  HTTP Client   │            │
│                          │  (fetch API)   │            │
│                          └────────┬────────┘            │
│                                   │                       │
└──────────────────────────┬───────────┘                    │
                           │                                │
                  ┌────────▼────────┐                      │
                  │  FastAPI       │                      │
                  │  /chat         │                      │
                  │  Endpoint      │                      │
                  └─────────────────┘                      │
```

### Component Design

**ChatWidget Component Hierarchy**:

```
ChatWidget (main container)
├── ChatMessages (message list)
│   ├── UserMessage (user query)
│   └── AssistantMessage (agent response)
│       └── SourceCitations (list of sources)
│           └── SourceCitation (single source)
├── ChatInput (input field + send button)
└── ToastNotification (error messages)
```

**State Management**:

```javascript
const [messages, setMessages] = useState([]);        // Chat history
const [loading, setLoading] = useState(false);       // Loading state
const [error, setError] = useState(null);           // Error state
const [sessionId, setSessionId] = useState(null);  // Session ID
```

### Request Flow

```
User types in chat widget
       ↓
ChatInput component captures input
       ↓
Frontend validates (non-empty, length)
       ↓
Fetch POST to /chat with:
  - query_text
  - session_id
  - selected_text (optional)
  - source_url (optional)
       ↓
FastAPI validates request
       ↓
FastAPI calls RAGAgent.chat()
       ↓
RAGAgent invokes retrieve_context tool
       ↓
retrieve_context searches Qdrant
       ↓
RAGAgent generates response via LLM
       ↓
FastAPI returns ChatResponse
       ↓
Frontend displays response with citations
```

### Error Handling Strategy

**Frontend Error Handling**:

1. **Network Errors** (fetch fails):
   - Show toast notification with error message
   - Display retry button
   - Exponential backoff for automatic retries

2. **API Errors** (4xx/5xx responses):
   - Parse error from ErrorMessage response
   - Show user-friendly error message
   - Provide actionable guidance (e.g., "Try rephrasing your question")

3. **Timeouts** (request >5 seconds):
   - Show timeout message
   - Allow retry
   - Maintain conversation state

**Backend Error Handling**:

1. **Validation Errors** (422):
   - Return detailed validation error messages
   - Include field-level validation details

2. **Retrieval Errors** (Qdrant failures):
   - Log error with context
   - Return generic error to user
   - Allow retry

3. **Agent Errors** (LLM failures):
   - Log error with conversation context
   - Return error message
   - Allow retry

### Session Management

**Frontend (sessionStorage)**:

```javascript
// On page load
const sessionId = sessionStorage.getItem('chat_session_id')
  || uuidv4();
sessionStorage.setItem('chat_session_id', sessionId);
```

**Backend (in-memory)**:

```python
# Global sessions dictionary
sessions: dict[str, ChatSession] = {}

# On first request with session_id
if session_id not in sessions:
    sessions[session_id] = ChatSession(session_id=session_id)
```

### Performance Considerations

**Backend Optimizations**:

1. **Connection Pooling**:
   - Reuse Qdrant client across requests
   - Reuse Cohere client across requests
   - Configure connection pool sizes

2. **Caching**:
   - Cache query embeddings (same query = same embedding)
   - Cache frequent retrieval results (optional)

3. **Async Operations**:
   - Use async/await for all I/O
   - Parallelize independent operations where possible

**Frontend Optimizations**:

1. **Lazy Loading**:
   - Load chat widget code only when needed
   - Use React.lazy() for component splitting

2. **Debouncing**:
   - Debounce user input (prevent rapid requests)

3. **Optimistic Updates**:
   - Show user message immediately
   - Update after response received

### Security Considerations

**Input Validation**:

1. **Backend**:
   - Pydantic models for all inputs
   - Maximum length constraints
   - UUID validation for session IDs
   - URL validation for source URLs

2. **Frontend**:
   - Validate before sending
   - Sanitize displayed content
   - Escape HTML in user inputs

**CORS Configuration**:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Development
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Rate Limiting** (future enhancement):

- Track request rate per session
- Return 429 if rate exceeded
- Implement token bucket algorithm

### Testing Strategy

**Backend Testing** (pytest):

```python
# test_api.py
- test_health_endpoint
- test_chat_endpoint_json_response
- test_chat_endpoint_streaming_response
- test_validation_errors
- test_retrieval_errors
- test_concurrent_requests

# test_agent_integration.py
- test_agent_generates_response
- test_agent_maintains_context
- test_agent_handles_no_retrieval_results
```

**Frontend Testing** (manual + automated):

1. **Manual Testing**:
   - Verify chat widget appears on all pages
   - Test basic query flow
   - Test follow-up questions
   - Test text selection context
   - Test error handling

2. **Automated Testing** (future):
   - Jest unit tests for components
   - React Testing Library for integration tests
   - Cypress for end-to-end tests

### Deployment Considerations

**Local Development**:

- Backend runs on localhost:8000
- Frontend runs on localhost:3000
- CORS configured for localhost

**Production Deployment** (future):

1. **Backend**:
   - Deploy to cloud (AWS Lambda, GCP Cloud Run)
   - Configure CORS for production domain
   - Set up monitoring and logging
   - Implement rate limiting

2. **Frontend**:
   - Build with `npm run build`
   - Deploy to static hosting (GitHub Pages, Vercel)
   - Configure environment variable for API URL
   - Set up CDN for static assets

### Monitoring & Observability

**Backend Logging**:

```python
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Log key events
logger.info(f"Chat request: session_id={session_id}, query_length={len(query)}")
logger.info(f"Response generated: latency_ms={latency}, sources_count={len(sources)}")
logger.error(f"Retrieval failed: {error}", exc_info=True)
```

**Frontend Logging**:

```javascript
console.log(`Chat request: ${sessionId}, query: ${query}`);
console.error(`Chat error: ${error}`);
```

**Metrics to Track**:

- Request count per session
- Average response time
- Error rate
- Source citation count
- Confidence score distribution

## Implementation Phases

### Phase 2: Tasks Generation (Next Step)

Execute `/sp.tasks` to generate actionable, dependency-ordered tasks for implementation.

### Phase 3: Implementation (Future)

Execute `/sp.implement` to process and execute all tasks defined in tasks.md.

### Phase 4: Validation (Future)

- Run backend tests: `pytest backend/tests/`
- Run frontend integration tests
- Manual testing following quickstart guide
- Verify all success criteria from spec

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations. Constitution check passed with no issues requiring justification.**

## Architecture Decision Records

No ADRs generated at this stage. All technical decisions are documented in research.md and this plan.

If significant decisions arise during implementation, consider creating ADRs for:

1. API versioning strategy (if v2 needed)
2. Streaming vs polling for updates (if streaming proves insufficient)
3. Database-backed sessions (if in-memory insufficient)
4. Frontend state management library (if React state insufficient)

## Risks & Mitigations

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|-----------|
| RAGAgent integration breaks with existing code | Medium | High | Test agent independently before API integration |
| Docusaurus swizzling conflicts with future Docusaurus updates | Medium | Medium | Document swizzling approach, test with Docusaurus updates |
| Frontend bundle size increases significantly | Low | Medium | Use code splitting, lazy loading for chat widget |
| API rate limits exceeded in production | Low | High | Implement rate limiting, caching strategies |
| Session context lost on backend restart | High | Low | Document limitation, acceptable for MVP |

## Success Criteria Validation

From spec.md, verify these after implementation:

- **SC-001**: Developers can successfully ask questions through the chat interface and receive answers in under 5 seconds.
  - **Verification**: Test with stopwatch, measure average response time
  - **Acceptance**: 95th percentile <5 seconds

- **SC-002**: 95% of responses include at least one relevant source citation from the book content.
  - **Verification**: Manual testing with diverse questions
  - **Acceptance**: Count sources in responses, target 95%

- **SC-003**: Follow-up questions maintain context accuracy across at least 3 consecutive turns.
  - **Verification**: Test multi-turn conversation
  - **Acceptance**: Agent correctly references previous context

- **SC-004**: Local development setup works end-to-end with both backend and frontend running on the same machine.
  - **Verification**: Follow quickstart guide on fresh machine
  - **Acceptance**: All steps work as documented

- **SC-005**: Questions based on selected text return answers that reference the specific selection 90% of the time.
  - **Verification**: Test text selection queries
  - **Acceptance**: Manual verification of selection references

- **SC-006**: The system handles at least 10 concurrent requests without response time degradation beyond 5 seconds.
  - **Verification**: Load testing with concurrent requests
  - **Acceptance**: 10 concurrent, all <5 seconds

- **SC-007**: Frontend gracefully handles backend unavailability with clear error messages to users.
  - **Verification**: Stop backend, test chat interactions
  - **Acceptance**: Error toast appears, no crashes

- **SC-008**: The chat interface works on all book pages without modifications to existing book content or structure.
  - **Verification**: Navigate to multiple pages
  - **Acceptance**: Chat widget appears on all pages, no content changes

## Next Steps

1. ✅ **Complete**: Spec creation
2. ✅ **Complete**: Research (Phase 0)
3. ✅ **Complete**: Design & contracts (Phase 1)
4. ⏭️ **Next**: Execute `/sp.tasks` to generate implementation tasks (Phase 2)
5. ⏭️ **Future**: Execute `/sp.implement` to build the feature (Phase 3)
6. ⏭️ **Future**: Run validation and testing (Phase 4)

**Status**: Planning complete. Ready for task generation.
