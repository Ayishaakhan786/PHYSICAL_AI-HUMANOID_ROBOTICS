# Quickstart: RAG Agent

**Feature**: `003-rag-agent` | **Date**: 2025-12-31

## Prerequisites

- Python 3.11+
- OpenAI API key
- Existing `.env` with Qdrant and Cohere credentials

## Installation

```bash
cd backend
pip install openai-agents
```

## Configuration

Ensure your `.env` file has:

```env
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your-api-key
COHERE_API_KEY=your-cohere-key
OPENAI_API_KEY=your-openai-key
COLLECTION_NAME=book-rag-embeddings
```

## Basic Usage

### Single Query

```bash
python agent.py "Explain embodied intelligence"
```

### Interactive Mode

```bash
python agent.py --interactive
# Then type your questions...
```

### With Custom Parameters

```bash
python agent.py "What is VLA?" --max-results 10 --threshold 0.6
```

## API Integration

### Python Client

```python
import requests

response = requests.post(
    "http://localhost:8000/chat",
    json={
        "message": "Explain humanoid robot kinematics",
        "max_results": 5,
        "threshold": 0.5
    }
)

data = response.json()
print(data["answer"])
print(f"Sources: {data['sources']}")
```

### cURL

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"message": "How does ROS 2 work?"}'
```

## Example Conversations

### Q1: Basic Question

```
User: "What is embodied intelligence?"
Agent: "Based on the Physical AI book, embodied intelligence refers to..."
Sources:
- https://.../embodied-intelligence (score: 0.71)
- https://.../physical-ai-fundamentals (score: 0.60)
```

### Q2: Follow-up

```
User: "How is it different from traditional AI?"
Agent: "Unlike traditional AI approaches, embodied intelligence..."
Sources:
- https://.../what-is-physical-ai (score: 0.55)
```

### Q3: Out of Scope

```
User: "What's the weather like?"
Agent: "I can only answer questions about the Physical AI and Humanoid Robotics book."
```

## Testing

```bash
# Run validation script
python agent.py --test

# Run specific test queries
python agent.py "Explain embodied intelligence"
python agent.py "What is VLA?"
python agent.py "How does ROS 2 integrate with Isaac Sim?"
```

## Troubleshooting

**No sources found**: Check that Qdrant collection has data
**Low confidence responses**: Lower the threshold with `--threshold 0.3`
**API errors**: Verify all API keys in `.env`
