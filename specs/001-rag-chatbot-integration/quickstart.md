# Quickstart: Backend–Frontend Integration for RAG Chatbot

**Created**: 2026-01-01
**Purpose**: Step-by-step guide for local development and end-to-end testing

## Prerequisites

- Python 3.11+ installed
- Node.js 18+ installed
- Valid API keys for:
  - Qdrant Cloud (from Spec-1)
  - Cohere (from Spec-1)
  - OpenRouter or OpenAI (from Spec-3)
- Git repository cloned locally

## Setup Steps

### 1. Backend Setup

Navigate to the backend directory and install dependencies:

```bash
cd backend
uv sync
```

Configure environment variables:

```bash
# Copy example environment file (if exists) or create .env
cp .env.example .env

# Edit .env with your API keys
nano .env
```

Required environment variables in `backend/.env`:

```bash
QDRANT_URL=https://your-qdrant-instance.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
COHERE_API_KEY=your-cohere-api-key
COLLECTION_NAME=book-rag-embeddings
```

### 2. Start Backend Server

Run the FastAPI server:

```bash
# Start the API server
python api.py

# Or with auto-reload for development
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

Verify the server is running:

```bash
# Check health endpoint
curl http://localhost:8000/health

# Should return: {"status":"ok","version":"1.0.0"}
```

The API documentation will be available at: http://localhost:8000/docs

### 3. Frontend Setup

Navigate to the book directory and install dependencies:

```bash
cd ../book
npm install
```

### 4. Start Frontend Development Server

Start the Docusaurus development server:

```bash
npm start
```

The book will be available at: http://localhost:3000

### 5. Test the Integration

Open your browser and navigate to: http://localhost:3000

1. Navigate to any book page (e.g., "Why Humanoid Robotics Matters")
2. You should see a chat widget in the bottom-right corner
3. Type a question like "What is embodied AI?"
4. Wait for the response (should arrive within 5 seconds)
5. Verify the response includes source citations with clickable links

### 6. Test Text Selection Context

1. On any book page, select a paragraph of text
2. A context menu or floating button should appear
3. Click "Ask about this" (or similar)
4. The chat widget should pre-populate with your selected text context
5. Type a question and submit
6. Verify the answer references your selected text

## Testing Scenarios

### Scenario 1: Basic Chat Query

**Test**: Ask a question about book content

1. Open chat widget
2. Type: "What is humanoid robot kinematics?"
3. Click send
4. Expected: Answer with source citations within 5 seconds

### Scenario 2: Follow-up Questions

**Test**: Ask follow-up questions that reference context

1. Ask: "What are VLA models?"
2. Wait for response
3. Ask: "How do they integrate with ROS 2?"
4. Expected: Agent understands "they" refers to VLA models from previous question

### Scenario 3: Text Selection Context

**Test**: Ask about selected text

1. Select a paragraph on the page
2. Click context menu action
3. Type question about the selection
4. Expected: Answer references the selected text specifically

### Scenario 4: Error Handling

**Test**: Backend unavailable

1. Stop the backend server (Ctrl+C)
2. Try to send a chat message
3. Expected: Error toast appears with retry button
4. Click retry
5. Expected: Fails gracefully (no crash)

### Scenario 5: No Relevant Content

**Test**: Question outside book scope

1. Ask: "What is the capital of France?"
2. Expected: Response indicates cannot answer based on available documentation

## Troubleshooting

### Backend Won't Start

**Problem**: Server fails to start with import errors

**Solutions**:
- Ensure `uv sync` completed successfully
- Check that Python version is 3.11+
- Verify `.env` file exists with correct keys
- Check if port 8000 is already in use

### Frontend Won't Start

**Problem**: Docusaurus server fails

**Solutions**:
- Run `npm install` to ensure dependencies are installed
- Check Node.js version (must be 18+)
- Clear cache: `rm -rf node_modules .docusaurus`
- Check if port 3000 is already in use

### Chat Requests Fail

**Problem**: Frontend cannot connect to backend

**Solutions**:
- Verify backend server is running on port 8000
- Check browser console for CORS errors
- Verify FastAPI CORS middleware is configured
- Check network tab in browser dev tools for request status

### No Responses from Agent

**Problem**: API returns errors or timeouts

**Solutions**:
- Check API keys in `.env` are valid
- Verify Qdrant collection exists and contains vectors
- Check Cohere API quota and rate limits
- Review backend logs for specific error messages
- Test retrieval with `python retrieve.py --query "test query"`

### Text Selection Not Working

**Problem**: Context menu doesn't appear when selecting text

**Solutions**:
- Check browser console for JavaScript errors
- Verify chat component is properly loaded
- Ensure you're selecting actual content text (not navigation elements)
- Try refreshing the page

## Development Workflow

### Making Backend Changes

1. Edit code in `backend/api.py` or other backend files
2. Server auto-reloads (if using `--reload` flag)
3. Refresh browser to test changes
4. Check backend logs for errors

### Making Frontend Changes

1. Edit chat component files in `book/src/components/`
2. Docusaurus hot-reloads automatically
3. Refresh browser if changes don't appear
4. Check browser console for errors

### Debugging Tips

**Backend Debugging**:
- Use FastAPI's built-in logging: `import logging; logging.basicConfig(level=logging.DEBUG)`
- Check Swagger docs at http://localhost:8000/docs
- Test endpoints manually with curl or Postman
- Use Python debugger: `import pdb; pdb.set_trace()`

**Frontend Debugging**:
- Use browser dev tools (F12)
- Check Console tab for JavaScript errors
- Check Network tab for API requests/responses
- Use React DevTools extension for component inspection

## API Testing

### Test JSON Response

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What is embodied AI?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "response_format": "json"
  }'
```

### Test Streaming Response

```bash
curl -X POST "http://localhost:8000/chat?stream=true" \
  -H "Content-Type: application/json" \
  -d '{
    "query_text": "What is embodied AI?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "response_format": "stream"
  }'
```

## Performance Benchmarks

Expected performance for local development:

| Metric | Target |
|--------|--------|
| API response time | <5 seconds (95th percentile) |
| Chat message display | <100ms after response received |
| Page load with chat widget | <2 seconds |
| Text selection context | <50ms to appear |

## Next Steps

Once the quickstart is working:

1. Customize the chat UI design to match book branding
2. Add additional error handling for edge cases
3. Implement analytics for chat usage (optional)
4. Set up production deployment (separate guide)
5. Write integration tests for critical paths

## Resources

- Backend API Docs: http://localhost:8000/docs
- Docusaurus Docs: https://docusaurus.io/docs
- FastAPI Docs: https://fastapi.tiangolo.com/
- React Docs: https://react.dev/

## Support

If you encounter issues:

1. Check this quickstart's Troubleshooting section
2. Review backend logs for detailed error messages
3. Check browser console for frontend errors
4. Ensure all prerequisites are met
5. Verify API keys are valid and quotas are not exceeded
