#!/usr/bin/env python3
"""
RAG Agent - Fixed Version for OpenRouter & Mistral
"""

import argparse
import asyncio
import json
import logging
import sys
import time
import uuid
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional
from agents import OpenAIChatCompletionsModel, Agent, Runner, function_tool
from openai import AsyncOpenAI

# 1. OpenRouter Configuration
ROUTER_API_KEY = "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"

client = AsyncOpenAI(
    api_key=ROUTER_API_KEY,
    base_url="https://openrouter.ai/api/v1"
)

# Global model instance using a FREE model
shared_model = OpenAIChatCompletionsModel(
    openai_client=client,
    model="mistralai/mistral-7b-instruct:free"
)

# Enable UTF-8 for Windows
if sys.platform == "win32":
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
logger = logging.getLogger(__name__)

# ============================================================================
# Configuration & Models
# ============================================================================

def load_config() -> Dict[str, str]:
    from dotenv import load_dotenv
    import os
    load_dotenv(override=True)
    
    config = {
        "QDRANT_URL": os.getenv("QDRANT_URL", ""),
        "QDRANT_API_KEY": os.getenv("QDRANT_API_KEY", ""),
        "COHERE_API_KEY": os.getenv("COHERE_API_KEY", ""),
        "COLLECTION_NAME": "book-rag-embeddings",
    }
    
    if not all([config["QDRANT_URL"], config["COHERE_API_KEY"]]):
        raise ValueError("Missing QDRANT or COHERE environment variables in .env")
    return config

@dataclass
class ConversationMessage:
    role: str
    content: str
    sources: List[Dict[str, Any]] = field(default_factory=list)
    timestamp: str = ""

# ============================================================================
# Retrieval Logic
# ============================================================================

class RetrievalWrapper:
    def __init__(self, config: Dict[str, str]):
        from qdrant_client import QdrantClient
        import cohere
        self.config = config
        self.qdrant_client = QdrantClient(url=config["QDRANT_URL"], api_key=config["QDRANT_API_KEY"])
        self.cohere_client = cohere.Client(api_key=config["COHERE_API_KEY"])

    def search(self, query: str):
        emb = self.cohere_client.embed(texts=[query], model="embed-english-v3.0", input_type="search_query")
        results = self.qdrant_client.query_points(
            collection_name=self.config["COLLECTION_NAME"],
            query=emb.embeddings[0],
            limit=5
        )
        return results.points

    def format_results(self, points):
        formatted = []
        for i, p in enumerate(points):
            formatted.append(f"[Source {i+1}] {p.payload.get('source_url')}\nContent: {p.payload.get('text')}")
        return "\n\n".join(formatted)

# ============================================================================
# Agent Setup
# ============================================================================

SYSTEM_PROMPT = """You are an expert on Physical AI. 
Answer questions based ONLY on the provided chunks. Use [Source N] for citations."""

class RAGAgent:
    def __init__(self, config: Dict[str, str]):
        self.config = config
        self.retriever = RetrievalWrapper(config)
        self.agent = self._create_agent()
        self.conversation_history = []

    def _create_agent(self) -> Agent:
        @function_tool
        async def retrieve_context(query: str) -> str:
            """Find information in the book."""
            points = self.retriever.search(query)
            return self.retriever.format_results(points) if points else "No info found."

        return Agent(
            name="PhysicalAI_Bot",
            instructions=SYSTEM_PROMPT,
            tools=[retrieve_context],
            model=shared_model  # Using the global OpenRouter model
        )

    async def chat(self, message: str):
        start = time.time()
        # Runner.run must be used with the agent instance
        result = await Runner.run(starting_agent=self.agent, input=message)
        latency = (time.time() - start) * 1000
        return result.final_output, latency

# ============================================================================
# Execution
# ============================================================================

async def run_query(query):
    config = load_config()
    bot = RAGAgent(config)
    print(f"\nThinking about: {query}...")
    answer, latency = await bot.chat(query)
    print(f"\nAgent Response:\n{answer}")
    print(f"\n[Time: {latency:.0f}ms]")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("query", type=str, help="Ask a question")
    args = parser.parse_args()
    
    if args.query:
        asyncio.run(run_query(args.query))