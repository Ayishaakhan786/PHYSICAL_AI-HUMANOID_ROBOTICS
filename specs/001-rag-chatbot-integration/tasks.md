# Tasks: Backend–Frontend Integration for RAG Chatbot

**Input**: Design documents from `/specs/001-rag-chatbot-integration/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/chat-api.yaml

**Tests**: Tests are NOT included in this task list (not explicitly requested in spec). Quickstart.md provides manual test scenarios.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies on incomplete tasks)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/` for Python code, `backend/tests/` for backend tests
- **Frontend**: `book/src/` for React components
- Paths align with plan.md structure

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend/tests directory in backend/tests
- [X] T002 [P] Create book/src/components/ChatWidget directory in book/src/components/ChatWidget
- [X] T003 [P] Create book/theme/DocPage directory in book/theme/DocPage
- [X] T004 [P] Add FastAPI to backend/pyproject.toml dependencies
- [X] T005 [P] Add pytest and httpx to backend/pyproject.toml dependencies
- [X] T006 [P] Add uuid to backend/pyproject.toml dependencies

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T007 Create Pydantic models in backend/api.py (ChatRequest, ChatResponse, SourceCitation, ErrorMessage)
- [X] T008 Implement session management with in-memory dict in backend/api.py
- [X] T009 Configure CORS middleware in backend/api.py
- [X] T010 Implement health check endpoint GET /health in backend/api.py
- [X] T011 Implement ChatSession dataclass in backend/api.py
- [X] T012 Create base FastAPI app with error handlers in backend/api.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Ask Questions About Book Content (Priority: P1) 🎯 MVP

**Goal**: Chat interface on all pages that accepts queries and returns grounded answers

**Independent Test**: Navigate to book in browser, open chat, ask question, verify answer with citations appears within 5 seconds

### Implementation for User Story 1

- [X] T013 [P] [US1] Create ChatWidget main component in book/src/components/ChatWidget/index.jsx
- [X] T014 [P] [US1] Create ChatMessages component in book/src/components/ChatWidget/ChatMessages.jsx
- [X] T015 [P] [US1] Create ChatInput component in book/src/components/ChatWidget/ChatInput.jsx
- [X] T016 [P] [US1] Create SourceCitation component in book/src/components/ChatWidget/SourceCitation.jsx
- [X] T017 [US1] Create chat widget styles in book/src/components/ChatWidget/chatWidget.css
- [X] T018 [US1] Implement state management in ChatWidget (messages, loading, error, sessionId) in book/src/components/ChatWidget/index.jsx
- [X] T019 [US1] Implement sessionStorage session ID management in book/src/components/ChatWidget/index.jsx
- [X] T020 [US1] Implement frontend validation (non-empty, length) in book/src/components/ChatWidget/ChatInput.jsx
- [X] T021 [US1] Implement POST /chat endpoint handler in backend/api.py (depends on T007, T011)
- [X] T022 [US1] Integrate RAGAgent from backend/agent.py in backend/api.py
- [X] T023 [US1] Implement response streaming support (SSE) in backend/api.py
- [X] T024 [US1] Implement calculate_confidence function in backend/api.py
- [X] T025 [US1] Implement fetch call to /chat in book/src/components/ChatWidget/index.jsx
- [X] T026 [US1] Implement JSON response parsing in book/src/components/ChatWidget/index.jsx
- [X] T027 [US1] Implement streaming response parsing in book/src/components/ChatWidget/index.jsx
- [X] T028 [US1] Display user messages in ChatMessages component
- [X] T029 [US1] Display assistant responses with source citations in ChatMessages component
- [X] T030 [US1] Swizzle DocPage in book/theme/DocPage/index.jsx
- [X] T031 [US1] Inject ChatWidget into all pages in book/theme/DocPage/index.jsx

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Ask Context-Aware Follow-up Questions (Priority: P2)

**Goal**: Follow-up questions maintain conversation context across turns

**Independent Test**: Ask multi-part conversation, verify agent maintains context across 3+ turns

### Implementation for User Story 2

- [ ] T032 [P] [US2] Implement message_history tracking in ChatSession backend/api.py
- [ ] T033 [P] [US2] Implement context_summary tracking in ChatSession backend/api.py
- [ ] T034 [US2] Add session_id to ChatRequest handling in backend/api.py (depends on T021)
- [ ] T035 [US2] Pass message history to RAGAgent in backend/api.py (depends on T022, T032)
- [ ] T036 [US2] Update ChatSession.add_message method in backend/api.py
- [ ] T037 [US2] Send message history with requests in ChatWidget book/src/components/ChatWidget/index.jsx
- [ ] T038 [US2] Display conversation history in ChatMessages book/src/components/ChatWidget/ChatMessages.jsx

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Ask Questions Based on Selected Text (Priority: P2)

**Goal**: Users can select text and ask context-aware questions

**Independent Test**: Select text on page, ask question, verify answer references selection

### Implementation for User Story 3

- [ ] T039 [P] [US3] Implement text selection detection in ChatWidget book/src/components/ChatWidget/index.jsx
- [ ] T040 [P] [US3] Add selected_text field to ChatRequest Pydantic model backend/api.py
- [ ] T041 [P] [US3] Add source_url field to ChatRequest Pydantic model backend/api.py
- [ ] T042 [US3] Implement context menu or floating button for selected text in ChatWidget book/src/components/ChatWidget/index.jsx
- [ ] T043 [US3] Pass selected_text and source_url to API in ChatWidget book/src/components/ChatWidget/index.jsx
- [ ] T044 [US3] Incorporate selected_text into RAGAgent query in backend/api.py (depends on T022, T040)
- [ ] T045 [US3] Display selection context indicator in ChatWidget book/src/components/ChatWidget/index.jsx

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Local Development and Testing (Priority: P3)

**Goal**: Backend and frontend run locally, end-to-end flow works

**Independent Test**: Follow quickstart.md on fresh machine, verify complete flow works

### Implementation for User Story 4

- [ ] T046 [P] [US4] Add uvicorn startup script to backend/pyproject.toml
- [ ] T047 [P] [US4] Add backend startup instructions to backend/README.md
- [ ] T048 [P] [US4] Add frontend startup instructions to book/README.md
- [ ] T049 [US4] Create backend/.env.example with required environment variables
- [ ] T050 [US4] Create quickstart.md following plan.md specifications in specs/001-rag-chatbot-integration/quickstart.md
- [ ] T051 [US4] Test local backend startup with python api.py
- [ ] T052 [US4] Test local frontend startup with npm start
- [ ] T053 [US4] Verify end-to-end flow (frontend → backend → agent → response)

**Checkpoint**: Local development fully functional

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T054 [P] Implement error handling with toast notifications in ChatWidget book/src/components/ChatWidget/index.jsx
- [ ] T055 [P] Add loading state indicator in ChatWidget book/src/components/ChatWidget/index.jsx
- [ ] T056 [P] Add exponential backoff retry logic in ChatWidget book/src/components/ChatWidget/index.jsx
- [ ] T057 [P] Implement backend error logging in backend/api.py
- [ ] T058 [P] Add response timeout handling in ChatWidget book/src/components/ChatWidget/index.jsx
- [ ] T059 [P] Validate HTML escaping in displayed content ChatMessages book/src/components/ChatWidget/ChatMessages.jsx
- [ ] T060 [P] Run quickstart.md validation scenarios

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P2 → P3)
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1 but independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - Integrates with US1 but independently testable
- **User Story 4 (P3)**: Can start after Foundational (Phase 2) - Documentation/setup only, no code dependencies

### Within Each User Story

- Models before handlers
- Frontend components [P] can run in parallel
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks (T001-T006) can run in parallel
- All Foundational tasks (T007-T012) can run in parallel
- Frontend component creation tasks within stories (marked [P]) can run in parallel
- US2 tasks (T032-T038) can run in parallel after US1 mostly complete
- US3 tasks (T039-T045) can run in parallel after US1 mostly complete
- US4 tasks (T046-T053) can run in parallel
- Polish tasks (T054-T060) can run in parallel

---

## Parallel Example: User Story 1

```bash
# Launch all ChatWidget components together:
Task: "Create ChatMessages component in book/src/components/ChatWidget/ChatMessages.jsx"
Task: "Create ChatInput component in book/src/components/ChatWidget/ChatInput.jsx"
Task: "Create SourceCitation component in book/src/components/ChatWidget/SourceCitation.jsx"
Task: "Create chat widget styles in book/src/components/ChatWidget/chatWidget.css"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T006)
2. Complete Phase 2: Foundational (T007-T012) - CRITICAL
3. Complete Phase 3: User Story 1 (T013-T031)
4. **STOP AND VALIDATE**: Test User Story 1 independently with quickstart scenarios
5. Demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Demo (MVP!)
3. Add User Story 2 → Test independently → Demo
4. Add User Story 3 → Test independently → Demo
5. Add User Story 4 → Validate locally
6. Polish → Final deliverable

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (T013-T031)
   - Developer B: User Story 2 (T032-T038)
   - Developer C: User Story 3 (T039-T045)
3. Developer D: User Story 4 (T046-T053)
4. Team completes Polish together (T054-T060)

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- User Stories 1, 2, 3 build on US1 but should be independently testable
- Stop at any checkpoint to validate story independently
- User Story 4 is documentation/setup, can be done in parallel with others

**Total Tasks**: 60
**Tasks per User Story**: US1 (19 tasks), US2 (7 tasks), US3 (7 tasks), US4 (8 tasks)
**Setup Tasks**: 6
**Foundational Tasks**: 6
**Polish Tasks**: 7
**Parallel Opportunities Identified**: 35+ tasks marked [P]

**Suggested MVP Scope**: Complete User Story 1 (19 tasks) after Setup (6) and Foundational (6) = **31 tasks total**
