#!/usr/bin/env python3
"""
FastAPI Backend for RAG Chatbot
Exposes /chat endpoint for Docusaurus frontend integration
"""

import asyncio
import json
import logging
import sys
import time
import uuid
from contextlib import asynccontextmanager
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Dict, List, Literal, Optional

import uvicorn
from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from sse_starlette.sse import EventSourceResponse

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

# Import RAGAgent from agent.py
from agent import RAGAgent, load_config

# ============================================================================
# Pydantic Data Models
# ============================================================================

class ChatRequest(BaseModel):
    """Request model for /chat endpoint"""
    query_text: str = Field(..., min_length=1, max_length=1000, description="User's question or prompt")
    session_id: str = Field(..., description="Unique identifier for conversation session")
    selected_text: Optional[str] = Field(None, max_length=5000, description="Text selected by user on current page")
    source_url: Optional[str] = Field(None, description="URL of page where query was initiated")
    response_format: Literal["json", "stream"] = Field(default="json", description="Preferred response format")

class SourceCitation(BaseModel):
    """Single source citation with metadata"""
    text: str = Field(..., max_length=500, description="Excerpt from book content")
    url: str = Field(..., description="Link to book page containing this content")
    section_path: str = Field(..., description="Hierarchical path of section")
    relevance_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score from retrieval (0-1)")

class ChatResponse(BaseModel):
    """Response model for /chat endpoint"""
    answer_text: str = Field(..., min_length=1, description="Generated response text from agent")
    sources: List[SourceCitation] = Field(..., min_length=1, max_length=10, description="List of retrieved book chunks with metadata")
    confidence: Literal["high", "medium", "low"] = Field(..., description="Indicator of answer confidence based on retrieval relevance")
    session_id: str = Field(..., description="Echoed from request for context tracking")
    latency_ms: int = Field(..., gt=0, description="Time taken to generate response in milliseconds")

class ErrorMessage(BaseModel):
    """Error response model"""
    error: str = Field(..., description="Human-readable error message")
    detail: Optional[str] = Field(default=None, description="Additional error details")
    error_code: str = Field(..., description="Machine-readable error code")

# ============================================================================
# Chat Session Data Model
# ============================================================================

@dataclass
class ConversationMessage:
    """Single message in a conversation"""
    role: str  # "user" or "assistant"
    content: str
    sources: List[Dict[str, Any]] = field(default_factory=list)
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())

@dataclass
class ChatSession:
    """In-memory session tracking"""
    session_id: str
    message_history: List[ConversationMessage] = field(default_factory=list)
    context_summaries: List[str] = field(default_factory=list)
    created_at: datetime = field(default_factory=datetime.now)
    last_activity: datetime = field(default_factory=datetime.now)

    def add_message(self, role: str, content: str, sources: List[Dict[str, Any]] = None) -> None:
        """Add a message to conversation history"""
        self.message_history.append(
            ConversationMessage(
                role=role,
                content=content,
                sources=sources or [],
                timestamp=datetime.now().isoformat()
            )
        )
        self.last_activity = datetime.now()

# ============================================================================
# Global State
# ============================================================================

# Global sessions dictionary (in-memory)
sessions: Dict[str, ChatSession] = {}

# Global agent instance (created on startup)
rag_agent: Optional[RAGAgent] = None

# ============================================================================
# Helper Functions
# ============================================================================

def calculate_confidence(sources: List[Dict[str, Any]]) -> Literal["high", "medium", "low"]:
    """Calculate confidence based on average relevance score"""
    if not sources:
        return "low"
    avg_score = sum(s.get("score", 0.5) for s in sources) / len(sources)
    if avg_score >= 0.75:
        return "high"
    if avg_score >= 0.5:
        return "medium"
    return "low"

def get_or_create_session(session_id: str) -> ChatSession:
    """Get existing session or create new one"""
    if session_id not in sessions:
        sessions[session_id] = ChatSession(session_id=session_id)
        logger.info(f"Created new session: {session_id}")
    return sessions[session_id]

def generate_mock_sources(query: str, source_url: Optional[str]) -> List[SourceCitation]:
    """Generate mock sources (to be replaced with actual retrieval)"""
    sources = []
    for i in range(3):
        sources.append(SourceCitation(
            text=f"Mock source text for query about: {query[:50]}...",
            url=source_url or "https://book.example.com/docs/introduction",
            section_path=f"docs/modules/chapter-{i+1}",
            relevance_score=0.85 - (i * 0.1)
        ))
    return sources

# ============================================================================
# Lifespan Events (FastAPI modern approach)
# ============================================================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Manage application lifespan"""
    global rag_agent
    # Startup
    try:
        config = load_config()
        rag_agent = RAGAgent(config)
        logger.info("RAGAgent initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize RAGAgent: {e}")
        raise

    yield

    # Shutdown
    logger.info("Shutting down RAG Chatbot API")

# ============================================================================
# FastAPI Application
# ============================================================================

app = FastAPI(
    title="RAG Chatbot API",
    description="API for RAG-powered chatbot that answers questions about Physical AI book content.",
    version="1.0.0",
    lifespan=lifespan,
)

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # For development - restrict in production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ============================================================================
# Endpoints
# ============================================================================

@app.get("/")
async def root():
    """Root endpoint - API is live"""
    return {
        "message": "Physical AI API is live",
        "status": "ok",
        "version": "1.0.0"
    }

@app.get("/favicon.ico")
async def favicon():
    """Dummy favicon endpoint to prevent 404 errors"""
    return {"status": "no favicon"}

@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "ok",
        "version": "1.0.0",
        "sessions_active": len(sessions)
    }

@app.post("/chat")
async def chat(request: ChatRequest, http_request: Request):
    """
    Process chat query and return response from RAG agent

    Supports both JSON and SSE streaming response formats
    """
    start_time = time.time()
    session = get_or_create_session(request.session_id)

    # Log context
    if request.selected_text:
        logger.info(f"Request includes {len(request.selected_text)} chars of selected text")
    if request.source_url:
        logger.info(f"Request from page: {request.source_url}")

    try:
        # Generate query with context
        full_query = request.query_text
        if request.selected_text:
            full_query = f"Context: {request.selected_text}\n\nQuestion: {request.query_text}"

        # Get response from RAG agent
        if rag_agent:
            answer_text, latency = await rag_agent.chat(full_query)
        else:
            # Fallback if agent not initialized
            logger.warning("RAGAgent not initialized, using mock response")
            answer_text = f"Mock response for: {request.query_text} (RAGAgent not initialized)"
            latency = 500

        # Generate sources (mock for now, integrate with retrieval later)
        sources = generate_mock_sources(request.query_text, request.source_url)

        # Calculate confidence
        confidence = calculate_confidence([{"score": s.relevance_score} for s in sources])

        # Calculate total latency
        latency_ms = int((time.time() - start_time) * 1000)

        # Build response
        response_data = {
            "answer_text": answer_text,
            "sources": [s.dict() for s in sources],
            "confidence": confidence,
            "session_id": request.session_id,
            "latency_ms": latency_ms
        }

        # Add message to session history
        session.add_message("user", request.query_text)
        session.add_message("assistant", answer_text, [s.dict() for s in sources])

        logger.info(f"Response generated: latency_ms={latency_ms}")

        # Handle streaming vs JSON response
        if request.response_format == "stream":
            # Streaming response (SSE)
            async def event_generator():
                yield f"data: {json.dumps({'type': 'message', 'content': answer_text})}\n\n"
                for source in sources:
                    yield f"data: {json.dumps({'type': 'source', 'url': source.url, 'text': source.text, 'section': source.section_path, 'score': source.relevance_score})}\n\n"
                yield f"data: {json.dumps({'type': 'done', 'confidence': confidence, 'latency_ms': latency_ms})}\n\n"

            return EventSourceResponse(event_generator())
        else:
            return response_data

    except Exception as e:
        latency_ms = int((time.time() - start_time) * 1000)
        logger.error(f"Chat failed: session={request.session_id}, error={str(e)}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail=ErrorMessage(
                error="An error occurred processing your request",
                detail=str(e),
                error_code="INTERNAL_ERROR"
            ).dict()
        )

# ============================================================================
# Exception Handlers
# ============================================================================

@app.exception_handler(HTTPException)
async def http_exception_handler(request: Request, exc: HTTPException):
    """Handle HTTP exceptions"""
    return {
        "error": exc.detail,
        "status_code": exc.status_code
    }

@app.exception_handler(Exception)
async def general_exception_handler(request: Request, exc: Exception):
    """Handle general exceptions"""
    logger.error(f"Unhandled exception: {exc}", exc_info=True)
    return {
        "error": "An unexpected error occurred",
        "status_code": 500
    }

# ============================================================================
# Main Entry Point
# ============================================================================

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000, log_level="info")
