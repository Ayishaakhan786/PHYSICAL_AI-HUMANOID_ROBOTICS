# Data Model: RAG Agent

**Feature**: `003-rag-agent` | **Date**: 2025-12-31

## Overview

This document describes the data structures used by the RAG agent for conversation management, retrieval, and response generation.

## Core Entities

### ConversationSession

Represents a single conversation thread between user and agent.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| session_id | string | Yes | Unique identifier for the session |
| messages | Message[] | Yes | List of messages in conversation |
| created_at | datetime | Yes | Session creation timestamp |
| updated_at | datetime | Yes | Last activity timestamp |
| metadata | dict | No | Optional session metadata |

### Message

Represents a single message in the conversation.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| role | enum | Yes | "user" or "assistant" |
| content | string | Yes | Message text content |
| sources | Source[] | No | Sources used for this message |
| timestamp | datetime | Yes | When message was created |

### Source

Represents a source citation from retrieved content.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| url | string | Yes | Source URL from documentation |
| heading | string | Yes | Page/section heading |
| section_path | string | No | Hierarchical path in docs |
| score | float | Yes | Relevance score (0-1) |
| chunk_text | string | Yes | Text content from chunk |

### RetrievalResult

Output from the retrieval pipeline.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| query | string | Yes | Original search query |
| chunks | RetrievedChunk[] | Yes | List of retrieved chunks |
| latency_ms | float | Yes | Retrieval time in milliseconds |
| timestamp | datetime | Yes | When retrieval occurred |

### RetrievedChunk

Individual chunk from the vector database.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | string | Yes | Unique point identifier |
| text | string | Yes | Chunk content text |
| source_url | string | Yes | Original documentation URL |
| section_path | string | No | Hierarchical path |
| heading | string | Yes | Page heading |
| chunk_index | int | No | Position in chunking sequence |
| score | float | Yes | Similarity score |

### AgentConfig

Configuration for the RAG agent.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| model | string | Yes | OpenAI model identifier |
| system_prompt | string | Yes | Agent instructions |
| max_results | int | No | Default max chunks (5) |
| threshold | float | No | Default relevance threshold (0.5) |
| temperature | float | No | Model temperature (0.0) |

### ChatRequest

API request format for chat endpoint.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| message | string | Yes | User's question |
| session_id | string | No | Existing session or new one |
| max_results | int | No | Override default max chunks |
| threshold | float | No | Override default threshold |

### ChatResponse

API response format from chat endpoint.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| answer | string | Yes | Agent's generated response |
| sources | Source[] | Yes | Citations used |
| session_id | string | Yes | Session identifier |
| latency_ms | float | Yes | Total response time |

## Validation Rules

- `session_id`: Must be a valid UUID or auto-generated
- `score`: Must be between 0.0 and 1.0
- `threshold`: Must be between 0.0 and 1.0
- `max_results`: Must be positive integer (1-20)
- `message.content`: Must not be empty
- `message.role`: Must be "user" or "assistant"

## Relationships

```
ConversationSession (1) ───> (many) Message
Message (1) ───> (many) Source
RetrievalResult (1) ───> (many) RetrievedChunk
ChatRequest (1) ───> (1) ChatResponse
```

## JSON Schema Reference

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "session_id": {"type": "string", "format": "uuid"},
    "message": {"type": "string", "minLength": 1},
    "sources": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "url": {"type": "string", "format": "uri"},
          "heading": {"type": "string"},
          "score": {"type": "number", "minimum": 0, "maximum": 1}
        },
        "required": ["url", "heading", "score"]
      }
    }
  },
  "required": ["session_id", "message"]
}
```
