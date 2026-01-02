# Data Model: Backend–Frontend Integration for RAG Chatbot

**Created**: 2026-01-01
**Purpose**: Define entity structures, validation rules, and state transitions

## Entity Definitions

### ChatRequest

Represents a user query sent from frontend to backend.

**Type**: Pydantic Model (backend) / TypeScript Interface (frontend)

**Fields**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `query_text` | string | Yes | User's question or prompt | Non-empty, max 1000 chars |
| `session_id` | string | Yes | Unique identifier for conversation session | Valid UUID v4 |
| `selected_text` | string | No | Text selected by user on current page | Optional, max 5000 chars |
| `source_url` | string | No | URL of page where query was initiated | Valid URL format |
| `response_format` | enum | No | Preferred response format | Values: "json" or "stream", default "json" |

**Backend Example**:
```python
from pydantic import BaseModel, Field
from typing import Optional, Literal

class ChatRequest(BaseModel):
    query_text: str = Field(..., min_length=1, max_length=1000)
    session_id: str = Field(..., pattern=r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$')
    selected_text: Optional[str] = Field(None, max_length=5000)
    source_url: Optional[str] = Field(None)
    response_format: Literal["json", "stream"] = "json"
```

**Frontend Example**:
```typescript
interface ChatRequest {
  query_text: string;
  session_id: string;
  selected_text?: string;
  source_url?: string;
  response_format?: "json" | "stream";
}
```

---

### ChatResponse

Represents the backend's answer to a user query.

**Type**: Pydantic Model (backend) / TypeScript Interface (frontend)

**Fields**:

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| `answer_text` | string | Generated response text from agent | Non-empty |
| `sources` | array | List of retrieved book chunks with metadata | 1-10 items |
| `confidence` | enum | Indicator of answer confidence | Values: "high", "medium", "low" |
| `session_id` | string | Echoed from request for context tracking | Must match request session_id |
| `latency_ms` | number | Time taken to generate response | Positive number |

**Backend Example**:
```python
class SourceCitation(BaseModel):
    text: str = Field(..., max_length=500)
    url: str = Field(...)
    section_path: str = Field(...)
    relevance_score: float = Field(..., ge=0.0, le=1.0)

class ChatResponse(BaseModel):
    answer_text: str = Field(..., min_length=1)
    sources: list[SourceCitation] = Field(..., min_items=1, max_items=10)
    confidence: Literal["high", "medium", "low"] = Field(...)
    session_id: str = Field(...)
    latency_ms: int = Field(..., gt=0)
```

**Frontend Example**:
```typescript
interface SourceCitation {
  text: string;
  url: string;
  section_path: string;
  relevance_score: number;
}

interface ChatResponse {
  answer_text: string;
  sources: SourceCitation[];
  confidence: "high" | "medium" | "low";
  session_id: string;
  latency_ms: number;
}
```

---

### SourceCitation

Represents a referenced book section in the response.

**Type**: Pydantic Model (backend) / TypeScript Interface (frontend)

**Fields**:

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| `text` | string | Excerpt from the book content | Non-empty, max 500 chars |
| `url` | string | Link to the book page containing this content | Valid URL |
| `section_path` | string | Hierarchical path of the section | Non-empty |
| `relevance_score` | number | Similarity score from retrieval | Range: 0.0-1.0 |

---

### ChatSession

Represents a conversation between a user and the agent. Stored in-memory on backend for context tracking.

**Type**: Python Dataclass (backend only - frontend uses sessionStorage)

**Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `session_id` | string | Unique identifier for the session |
| `message_history` | list | Sequential list of user queries and agent responses |
| `context_summaries` | list | Optional summaries of key context for follow-up queries |
| `created_at` | datetime | Timestamp when the session began |
| `last_activity` | datetime | Timestamp of the most recent interaction |

**Backend Example**:
```python
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Dict, Any

@dataclass
class ConversationMessage:
    role: str  # "user" or "assistant"
    content: str
    sources: List[Dict[str, Any]] = field(default_factory=list)
    timestamp: str = ""

@dataclass
class ChatSession:
    session_id: str
    message_history: List[ConversationMessage] = field(default_factory=list)
    context_summaries: List[str] = field(default_factory=list)
    created_at: datetime = field(default_factory=datetime.now)
    last_activity: datetime = field(default_factory=datetime.now)

    def add_message(self, role: str, content: str, sources: List[Dict[str, Any]] = None):
        """Add a message to the conversation history."""
        self.message_history.append(
            ConversationMessage(
                role=role,
                content=content,
                sources=sources or [],
                timestamp=datetime.now().isoformat()
            )
        )
        self.last_activity = datetime.now()
```

---

### ErrorMessage

Represents an error response from the backend.

**Type**: Pydantic Model (backend) / TypeScript Interface (frontend)

**Fields**:

| Field | Type | Description |
|-------|------|-------------|
| `error` | string | Human-readable error message |
| `error_code` | string | Machine-readable error identifier |
| `session_id` | string | Session ID for error tracking |
| `retry_after` | number | Optional - seconds before retry |

**Backend Example**:
```python
class ErrorMessage(BaseModel):
    error: str = Field(...)
    error_code: str = Field(...)
    session_id: str = Field(...)
    retry_after: Optional[int] = Field(None, gt=0)
```

---

## State Transitions

### Chat Session Lifecycle

```
[User opens page]
       ↓
[Generate session_id] ──→ [Store in sessionStorage]
       ↓
[First message sent]
       ↓
[Backend creates ChatSession]
       ↓
[Chat exists] ──┬─→ [Follow-up message] ──→ [Update message_history]
                │                          ↓
                │                    [Update last_activity]
                │
                └─→ [Tab closed] ──→ [Session expires]
```

### Request-Response States (Frontend)

```
IDLE
  ↓ [User submits query]
LOADING (show spinner)
  ↓
  ├─→ SUCCESS (display response)
  │         ↓
  │      READY for next query
  │
  ├─→ ERROR (show error toast)
  │         ↓
  │      RETRY button available
  │         ↓
  │      IDLE
  │
  └─→ TIMEOUT (show timeout message)
            ↓
         RETRY button available
            ↓
         IDLE
```

---

## Validation Rules

### Input Validation (Frontend)

1. **Query Text**:
   - Required field
   - Must not be empty or whitespace only
   - Maximum length: 1000 characters
   - Trim whitespace before sending

2. **Session ID**:
   - Required field
   - Must be valid UUID v4 format
   - Generate on first interaction

3. **Selected Text**:
   - Optional field
   - Maximum length: 5000 characters
   - Validate UTF-8 encoding

4. **Source URL**:
   - Optional field
   - Must be valid URL format
   - Use `window.location.href` for current page

### Input Validation (Backend)

1. **Pydantic Models**:
   - All fields validated automatically by Pydantic
   - Return HTTP 422 for validation errors

2. **Query Text**:
   - Check non-empty
   - Check maximum length
   - Sanitize to prevent injection attacks

3. **Session ID**:
   - Validate UUID v4 format
   - Check if session exists in memory

4. **Response Format**:
   - Validate enum value ("json" or "stream")
   - Default to "json" if not provided

---

## Relationships

```
ChatSession (1) ──────── (*) ConversationMessage
    │                        │
    │                        ├─→ role: "user" | "assistant"
    │                        ├─→ content: string
    │                        └─→ sources: [SourceCitation]
    │
    └─→ session_id: string

ChatRequest (1) ──────── (1) ChatSession
      │                    (via session_id)
      ├─→ query_text
      ├─→ selected_text
      └─→ source_url

ChatResponse (1) ──────── (*) SourceCitation
      │                    (1-10 items)
      ├─→ answer_text
      ├─→ confidence
      └─→ sources
```

---

## Storage

### Backend Storage

| Entity | Storage | Persistence | Access Pattern |
|--------|---------|-------------|----------------|
| ChatSession | In-memory dict | Session lifetime | Fast lookup by session_id |
| RAGAgent | Singleton | Application lifetime | Shared across requests |

### Frontend Storage

| Entity | Storage | Persistence | Access Pattern |
|--------|---------|-------------|----------------|
| session_id | sessionStorage | Tab session | Read/write as needed |
| message_history | sessionStorage | Tab session | For chat UI display |

---

## Error Codes

| Error Code | Description | HTTP Status | Retry Recommended |
|------------|-------------|-------------|-------------------|
| `VALIDATION_ERROR` | Invalid request format | 422 | No |
| `SESSION_NOT_FOUND` | Session ID not found | 404 | No (new session) |
| `RETRIEVAL_FAILED` | No relevant content found | 404 | Maybe (rephrase query) |
| `AGENT_ERROR` | LLM generation failed | 500 | Yes |
| `TIMEOUT` | Request took too long | 504 | Yes |
| `SERVICE_UNAVAILABLE` | Backend not responding | 503 | Yes |
| `RATE_LIMITED` | Too many requests | 429 | Yes (after delay) |

---

## Confidence Calculation

Confidence levels are determined based on retrieval quality:

| Confidence | Relevance Score | Source Count | Criteria |
|------------|----------------|--------------|----------|
| **high** | > 0.75 | >= 2 | Top sources are highly relevant and consistent |
| **medium** | 0.50 - 0.75 | >= 1 | Sources are relevant but may be less comprehensive |
| **low** | < 0.50 | >= 1 | Low relevance or limited sources available |

**Calculation** (backend):
```python
def calculate_confidence(sources: List[SourceCitation]) -> str:
    if not sources:
        return "low"

    avg_score = sum(s.relevance_score for s in sources) / len(sources)
    top_score = sources[0].relevance_score

    if top_score > 0.75 and len(sources) >= 2:
        return "high"
    elif avg_score > 0.50:
        return "medium"
    else:
        return "low"
```
