# Research: Backend–Frontend Integration for RAG Chatbot

**Created**: 2026-01-01
**Purpose**: Technical research and decision rationale for implementation plan

## Technical Context Assessment

Based on the spec requirements and existing codebase analysis, the following technical context is established:

**Language/Version**: Python 3.11+ (backend), JavaScript (Docusaurus frontend)
**Primary Dependencies**: FastAPI (backend), OpenAI Agents SDK, Qdrant Client, Cohere API
**Storage**: Qdrant Cloud (vector database)
**Testing**: pytest for backend, Docusaurus dev server for frontend integration testing
**Target Platform**: Local development (localhost), web browser
**Project Type**: Web application with backend API and Docusaurus static site frontend
**Performance Goals**: <5 second response time for 95% of requests, support 10+ concurrent requests
**Constraints**: No authentication, single backend API file, no changes to existing book pages
**Scale/Scope**: Single book project, embedded chat component on all pages

## Research Questions & Decisions

### 1. Backend API Design

**Question**: How should the FastAPI `/chat` endpoint be structured to support both JSON and streaming responses?

**Decision**: Implement a single `/chat` endpoint with an optional query parameter to control response format. Use FastAPI's native streaming response support (`StreamingResponse`) for Server-Sent Events (SSE) when requested, and standard JSON responses otherwise.

**Rationale**:
- FastAPI has built-in support for streaming responses via `StreamingResponse` with generators
- SSE is simpler than WebSocket for one-way streaming (server to client)
- Single endpoint reduces complexity while offering flexibility
- Query parameter approach is RESTful and doesn't require multiple endpoints

**Alternatives Considered**:
- WebSocket approach (rejected: overkill for one-way streaming, adds complexity)
- Separate `/chat` and `/chat/stream` endpoints (rejected: unnecessary endpoint duplication)

### 2. Frontend Integration Method

**Question**: How to embed a chat UI component into existing Docusaurus pages without modifying book content?

**Decision**: Use Docusaurus swizzling of the main DocPage component and inject the chat widget as a new component. Add the component to the `book/src/components/` directory without altering `.md` content files.

**Rationale**:
- Docusaurus swizzling allows customizing layout without modifying content files
- Injecting via a component ensures all book pages get the chat interface
- No changes to `.md` files preserves content integrity
- Component-based approach aligns with Docusaurus best practices

**Alternatives Considered**:
- Modify every `.md` file to include component (rejected: violates constraint of no content changes)
- Use React Portal injected via client script (rejected: harder to maintain, less Docusaurus-native)

### 3. Text Selection Context

**Question**: How should the frontend capture user-selected text and send it with the query?

**Decision**: Use native browser `window.getSelection()` API to capture selected text. Add a custom context menu or floating action button that appears when text is selected, allowing users to ask questions about the selection.

**Rationale**:
- Native browser API is lightweight and works across all modern browsers
- Context menu approach is intuitive for users
- No external dependencies required for text selection handling
- Maintains clean separation between book content and chat functionality

**Alternatives Considered**:
- Highlighting text with custom library (rejected: adds complexity, conflicts with existing UI)
- Keyboard shortcut to trigger context (rejected: less discoverable)

### 4. Session Management

**Question**: How should conversation sessions be maintained without authentication?

**Decision**: Generate unique session IDs using `uuid4` on the client side and store them in `sessionStorage`. Maintain session context in-memory on the backend within the RAG agent.

**Rationale**:
- `sessionStorage` persists for the tab/session, providing adequate context lifetime
- No authentication requirement means sessions don't need to be persisted across browser refreshes
- In-memory storage on backend is sufficient for short-term context
- Simple implementation without database complexity

**Alternatives Considered**:
- LocalStorage for persistence (rejected: sessions should reset when tab closes)
- Database-backed sessions (rejected: unnecessary complexity, no authentication requirement)

### 5. Error Handling

**Question**: How should the frontend handle backend unavailability gracefully?

**Decision**: Implement exponential backoff retry logic for failed requests. Display user-friendly error messages using toast notifications. Add a fallback mode that shows "Chat unavailable" message with retry button.

**Rationale**:
- Exponential backoff reduces server load during outages
- Toast notifications are non-intrusive and follow modern UX patterns
- Fallback mode provides clear user feedback
- Retry button allows users to recover without page refresh

**Alternatives Considered**:
- Silent retry with no user feedback (rejected: poor UX, users unaware of issues)
- Alert-based error messages (rejected: intrusive, blocks user interaction)

### 6. Agent Integration

**Question**: How should the existing `backend/agent.py` be integrated with the FastAPI API?

**Decision**: Import the `RAGAgent` class from `agent.py` directly in `api.py`. Initialize a single shared agent instance at application startup using FastAPI's lifespan events. Reuse the existing `retrieve_context` tool without modification.

**Rationale**:
- Single shared instance reduces overhead and connection pool usage
- FastAPI lifespan events ensure proper initialization and cleanup
- Direct import maintains code organization without unnecessary refactoring
- Existing agent is well-tested and meets spec requirements

**Alternatives Considered**:
- Create new agent instance per request (rejected: inefficient, connection overhead)
- Refactor agent into a separate package (rejected: unnecessary for this scope)

### 7. Frontend UI Framework

**Question**: What UI framework should be used for the chat component?

**Decision**: Use plain React with inline styles or Tailwind CSS classes if available. Keep dependencies minimal to avoid bloating the Docusaurus bundle size.

**Rationale**:
- Docusaurus already includes React and styling
- Minimal dependencies reduces bundle size and complexity
- Plain React is sufficient for chat UI requirements
- Reuses existing Docusaurus patterns

**Alternatives Considered**:
- Material-UI or other component libraries (rejected: adds bundle weight)
- Headless UI libraries (rejected: overkill for simple chat interface)

### 8. CORS Configuration

**Question**: How should CORS be configured for local development?

**Decision**: Enable CORS with wildcard origins for local development. Restrict to specific origins in production configuration using FastAPI's CORSMiddleware.

**Rationale**:
- Wildcard origins simplify local development
- FastAPI's CORSMiddleware provides easy configuration
- Production restrictions prevent unauthorized access
- Follows security best practices

**Alternatives Considered**:
- Disable CORS entirely (rejected: breaks browser security model)
- Hardcode localhost origins (rejected: inflexible for different dev environments)

## Integration Patterns

### Backend-Frontend Communication

```
┌─────────────┐         HTTP POST /chat         ┌──────────────┐
│   Frontend  │ ──────────────────────────────> │   Backend    │
│ (Docusaurus)│                                 │   (FastAPI)  │
└─────────────┘ <───────────────────────────────┘──────────────┘
     JSON/SSE                                        JSON
     Response                                        Response
```

### Request Flow

1. User types question in chat UI
2. Frontend validates input (non-empty, length check)
3. Frontend sends POST to `/chat` with:
   - `query_text`: user question
   - `session_id`: unique session identifier
   - `selected_text`: optional selected content
   - `source_url`: optional current page URL
   - `response_format`: "json" or "stream"
4. Backend receives request
5. Backend calls RAGAgent.chat() with query
6. Agent retrieves context via retrieve_context tool
7. Agent generates response grounded in retrieved content
8. Backend returns JSON or streams response
9. Frontend displays response with source citations

### Data Flow

```
User Query → Frontend → FastAPI → RAGAgent → Retrieval → Qdrant → Sources
                                                       ↓
                                                   OpenAI LLM
                                                       ↓
                                              Response + Citations
                                                       ↓
                                                   FastAPI
                                                       ↓
                                                   Frontend
                                                       ↓
                                                   Display
```

## Best Practices Applied

### FastAPI
- Async/await for non-blocking I/O
- Pydantic models for request/response validation
- Automatic OpenAPI documentation via Swagger UI
- Lifespan events for resource management

### React/Docusaurus
- Functional components with hooks
- CSS-in-JS for component isolation
- Client-side only rendering (no SSR for chat)
- Error boundaries for graceful failures

### Error Handling
- Try/catch with specific error types
- User-friendly error messages
- Retry logic with exponential backoff
- Fallback UI for service unavailability

### Performance
- Connection pooling for database clients
- Caching where appropriate
- Lazy loading for chat UI
- Efficient bundling with tree shaking

## Open Questions Resolved

All questions resolved through this research. No NEEDS CLARIFICATION markers remain.

## Dependencies Verified

- **FastAPI**: Confirmed availability via Python package manager
- **OpenAI Agents SDK**: Already in backend from Spec-3
- **Qdrant Client**: Already in backend from Spec-1
- **Cohere API**: Already configured in backend
- **Docusaurus**: Confirmed as existing frontend framework
- **React**: Confirmed as Docusaurus's React version

## Constraints Confirmed

- No authentication: ✓ Verified, session management without auth is feasible
- Single backend API file: ✓ Verified, agent.py integration works directly
- No changes to existing book pages: ✓ Verified, Docusaurus swizzling enables this
- Local development support: ✓ Verified, CORS and localhost configuration straightforward
