# Quickstart Guide: OpenAI Agents SDK RAG Agent

**Branch**: `3-openai-agent` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)

## Overview

This guide walks you through setting up and using the RAG agent powered by OpenAI Agents SDK for answering questions about humanoid robotics. The agent retrieves relevant content from the book, provides cited responses, and supports multi-turn conversations.

**What You'll Build**:
- AI agent with retrieval-augmented generation (RAG)
- Dual-mode operation: full-book search vs. context-constrained explanations
- Multi-turn conversational interface with session management
- Citation transparency with source URLs

**Time to Complete**: ~30 minutes (assuming Feature 2 is implemented)

---

## Prerequisites

### Required Software
- **Python**: 3.13+ (3.9+ may work but not tested)
- **pip**: Latest version (`python -m pip install --upgrade pip`)
- **Git**: For branch checkout
- **Code Editor**: VS Code, PyCharm, or similar

### Required Accounts
- **OpenAI Account**: Sign up at https://platform.openai.com
- **OpenAI API Key**: Generate at https://platform.openai.com/api-keys
  - Requires payment method on file
  - Estimated cost: $2-10/day depending on usage

### Feature Dependencies
- **Feature 1 (Vector Ingestion)**: Must be complete (embeddings in Qdrant)
- **Feature 2 (RAG Retrieval)**: Must be complete (POST /search endpoint working)
  - If Feature 2 not ready, see "Development Without Feature 2" section below

---

## Step 1: Environment Setup

### 1.1 Check Out Feature Branch

```bash
cd humanoid-robotics
git checkout 3-openai-agent
```

**Verify you're on the correct branch**:
```bash
git branch
# Should show: * 3-openai-agent
```

### 1.2 Install Dependencies

Add new packages to `backend/requirements.txt`:

```bash
# Existing dependencies (already installed from Feature 1-2)
fastapi>=0.104.0
uvicorn[standard]>=0.27.0
qdrant-client>=1.8.0
cohere>=5.5.0
python-dotenv>=1.0.0

# NEW: Feature 3 dependencies
openai-agents>=0.6.3
httpx>=0.26.0
```

**Install packages**:
```bash
cd backend
pip install openai-agents httpx
# Or install all from requirements.txt
pip install -r requirements.txt
```

**Verify installation**:
```bash
python -c "import agents; import httpx; print('✓ Dependencies installed')"
# Should print: ✓ Dependencies installed
```

### 1.3 Configure OpenAI API Key

**Get your API key** from https://platform.openai.com/api-keys

**Add to `.env` file** (backend/.env):
```bash
# Existing keys (from Feature 1-2)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
COHERE_API_KEY=your-cohere-key

# NEW: OpenAI API key
OPENAI_API_KEY=sk-proj-...your-key-here...
```

**Important**: Never commit `.env` file to git. Add to `.gitignore`:
```bash
echo ".env" >> .gitignore
```

**Verify API key**:
```bash
python -c "import os; from dotenv import load_dotenv; load_dotenv(); print('✓ API key loaded' if os.getenv('OPENAI_API_KEY') else '✗ API key missing')"
```

### 1.4 Create Session Database Directory

```bash
mkdir -p backend/data
```

The SQLite session database (`conversations.db`) will be created automatically on first use.

---

## Step 2: Verify Feature 2 Dependency

The agent requires the `/search` endpoint from Feature 2 to be working.

**Test /search endpoint**:
```bash
# Start backend server (if not already running)
cd backend
uvicorn main:app --reload --port 8000
```

**In another terminal, test /search**:
```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "zero-moment point",
    "top_k": 3
  }'
```

**Expected response**:
```json
{
  "results": [
    {
      "text": "The zero-moment point (ZMP) is...",
      "score": 0.892,
      "metadata": {
        "url": "https://example.com/chapter3",
        "source": "Humanoid Robotics Book",
        "chunk_index": 42
      }
    }
  ],
  "query": "zero-moment point",
  "total_results": 3
}
```

**If /search returns 404**: Feature 2 not implemented. See "Development Without Feature 2" below.

---

## Step 3: Run the Agent (First Test)

### 3.1 Start FastAPI Server

```bash
cd backend
uvicorn main:app --reload --port 8000
```

**Expected output**:
```
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

### 3.2 Open API Documentation

Navigate to **http://localhost:8000/docs** in your browser.

You should see:
- Existing endpoints: `/ingest`, `/search` (from Features 1-2)
- **NEW endpoints**:
  - POST `/agent/chat`
  - POST `/agent/chat/structured`
  - GET `/agent/session/{session_id}`
  - DELETE `/agent/session/{session_id}`
  - GET `/health`

### 3.3 Test Agent via Swagger UI

**Click on POST `/agent/chat`** → **Try it out**

**Request body**:
```json
{
  "message": "What is the zero-moment point in humanoid robotics?",
  "session_id": "quickstart_test_1",
  "context_text": null
}
```

**Click "Execute"**

**Expected response** (takes 2-4 seconds):
```json
{
  "response": "The zero-moment point (ZMP) is a fundamental concept in humanoid robot balance control [Source 1]. It represents the point on the ground where the sum of all inertial and gravitational moments equals zero [Source 2]. The ZMP is used to ensure dynamic stability during walking and other movements [Source 1].\n\nSources:\n- [Source 1] https://example.com/humanoid-robotics/chapter3\n- [Source 2] https://example.com/humanoid-robotics/chapter5",
  "session_id": "quickstart_test_1",
  "status": "success"
}
```

**Verify**:
- ✅ Response includes inline citations `[Source 1]`, `[Source 2]`
- ✅ Sources section lists URLs at the end
- ✅ Response is grounded in book content (no hallucination)
- ✅ Status is "success"

---

## Step 4: Test Multi-Turn Conversation

Multi-turn conversations use the **same session_id** across requests.

### 4.1 First Turn

**Request**:
```json
{
  "message": "What is the zero-moment point?",
  "session_id": "conversation_123",
  "context_text": null
}
```

**Response**: (Agent explains ZMP with citations)

### 4.2 Second Turn (Follow-Up Question)

**Request** (same session_id):
```json
{
  "message": "How is it used for balance control?",
  "session_id": "conversation_123",
  "context_text": null
}
```

**Response**: Agent remembers the previous context about ZMP and answers the follow-up question.

**Verify**:
- ✅ Agent understands "it" refers to ZMP from previous turn
- ✅ Response builds on prior conversation
- ✅ Still includes citations

### 4.3 View Conversation History

**GET /agent/session/conversation_123**

**Response**:
```json
{
  "session_id": "conversation_123",
  "messages": [
    {
      "role": "user",
      "content": "What is the zero-moment point?",
      "timestamp": "2025-12-15T10:30:00Z",
      "tool_call_id": null
    },
    {
      "role": "tool",
      "content": "[Source 1] URL: https://...",
      "timestamp": "2025-12-15T10:30:02Z",
      "tool_call_id": "call_abc123"
    },
    {
      "role": "assistant",
      "content": "The zero-moment point (ZMP) is...",
      "timestamp": "2025-12-15T10:30:05Z",
      "tool_call_id": null
    },
    {
      "role": "user",
      "content": "How is it used for balance control?",
      "timestamp": "2025-12-15T10:31:00Z",
      "tool_call_id": null
    }
  ],
  "total_count": 4
}
```

---

## Step 5: Test Dual-Mode Operation

### 5.1 Full-Book Mode (with Retrieval)

**Request** (context_text = null):
```json
{
  "message": "What is inverse kinematics?",
  "session_id": "test_full_book",
  "context_text": null
}
```

**Behavior**:
- Agent calls `retrieve_book_content` tool
- Retrieves relevant chunks from /search
- Responds with citations

### 5.2 Context-Constrained Mode (without Retrieval)

**Request** (context_text provided):
```json
{
  "message": "Explain this concept in simple terms",
  "session_id": "test_context_mode",
  "context_text": "Inverse kinematics (IK) is the mathematical process of calculating the joint angles needed to place a robot's end-effector (like a hand or foot) at a desired position and orientation. Unlike forward kinematics, which computes where the end-effector will be given joint angles, IK works backwards to determine what angles are needed to reach a specific target."
}
```

**Response**:
```json
{
  "response": "Based on the provided text, inverse kinematics (IK) is like solving a puzzle backwards. Imagine you want to touch a specific spot with your hand—IK figures out how to bend your elbow and shoulder to reach it. It's the opposite of forward kinematics, which would tell you where your hand ends up if you bend your joints a certain way. In robotics, this helps the robot plan how to move its joints to reach a target position.",
  "session_id": "test_context_mode",
  "status": "success"
}
```

**Verify**:
- ✅ Response does NOT include citations (no retrieval performed)
- ✅ Explanation is based solely on provided context_text
- ✅ Simpler language (as requested)

---

## Step 6: Test Error Handling

### 6.1 Off-Topic Question (No Relevant Content)

**Request**:
```json
{
  "message": "What is quantum entanglement?",
  "session_id": "test_off_topic",
  "context_text": null
}
```

**Expected Response**:
```json
{
  "response": "I couldn't find relevant information in the book to answer your question about quantum entanglement. This topic may not be covered in the humanoid robotics textbook. Could you ask about a robotics-related topic instead?",
  "session_id": "test_off_topic",
  "status": "success"
}
```

**Verify**:
- ✅ Agent refuses to answer (no hallucination)
- ✅ Provides helpful guidance to ask on-topic questions
- ✅ No citations (no relevant content retrieved)

### 6.2 Invalid Request (Validation Error)

**Request** (empty message):
```json
{
  "message": "",
  "session_id": "test_validation",
  "context_text": null
}
```

**Expected Response** (400 Bad Request):
```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "message must be at least 1 character",
    "details": "Field validation failed for 'message'"
  },
  "session_id": null
}
```

---

## Step 7: Clear Session (Cleanup)

After testing, clear conversation history:

**DELETE /agent/session/conversation_123**

**Expected Response** (204 No Content):
- Empty response body
- Session deleted from database

**Verify deletion**:

GET `/agent/session/conversation_123` should return 404 Not Found.

---

## Usage Examples

### Example 1: Basic Question

**cURL**:
```bash
curl -X POST http://localhost:8000/agent/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is forward kinematics?",
    "session_id": "user_42_conv_1",
    "context_text": null
  }'
```

**Python**:
```python
import requests

response = requests.post(
    "http://localhost:8000/agent/chat",
    json={
        "message": "What is forward kinematics?",
        "session_id": "user_42_conv_1",
        "context_text": None
    }
)

print(response.json()["response"])
```

**JavaScript**:
```javascript
const response = await fetch('http://localhost:8000/agent/chat', {
  method: 'POST',
  headers: { 'Content-Type': 'application/json' },
  body: JSON.stringify({
    message: "What is forward kinematics?",
    session_id: "user_42_conv_1",
    context_text: null
  })
});

const data = await response.json();
console.log(data.response);
```

### Example 2: Multi-Turn Conversation

**Python**:
```python
import requests

session_id = "user_42_conv_2"
base_url = "http://localhost:8000/agent/chat"

# Turn 1
response1 = requests.post(base_url, json={
    "message": "What is the zero-moment point?",
    "session_id": session_id,
    "context_text": None
})
print("Turn 1:", response1.json()["response"])

# Turn 2 (follow-up)
response2 = requests.post(base_url, json={
    "message": "How is it calculated?",
    "session_id": session_id,
    "context_text": None
})
print("Turn 2:", response2.json()["response"])

# Get full history
history = requests.get(f"http://localhost:8000/agent/session/{session_id}")
print(f"Total messages: {history.json()['total_count']}")
```

### Example 3: Context-Constrained Explanation

**Python**:
```python
import requests

selected_text = """
The Denavit-Hartenberg (DH) convention is a systematic method for
describing the kinematic structure of a robot manipulator. It uses
four parameters (a, α, d, θ) to represent the transformation between
adjacent joint coordinate frames.
"""

response = requests.post("http://localhost:8000/agent/chat", json={
    "message": "Explain what this means for a beginner",
    "session_id": "user_42_conv_3",
    "context_text": selected_text
})

print(response.json()["response"])
# Expected: Simplified explanation without retrieval
```

---

## Development Without Feature 2

If Feature 2 (RAG retrieval) is not yet implemented, you can develop Feature 3 using mock responses.

### Option A: Mock /search Endpoint

Add temporary mock endpoint to `backend/main.py`:

```python
@app.post("/search")  # Temporary mock for development
async def search_mock(request: dict):
    """Mock /search endpoint for Feature 3 development."""
    return {
        "results": [
            {
                "text": "The zero-moment point (ZMP) is a fundamental concept in humanoid robot balance control. It represents the point on the ground where the sum of all inertial and gravitational moments equals zero.",
                "score": 0.892,
                "metadata": {
                    "url": "https://example.com/humanoid-robotics/chapter3",
                    "source": "Humanoid Robotics Book",
                    "chunk_index": 42
                }
            },
            {
                "text": "The ZMP is used to ensure dynamic stability during walking and other movements. By controlling the robot's posture to keep the ZMP within the support polygon, the robot can avoid falling.",
                "score": 0.854,
                "metadata": {
                    "url": "https://example.com/humanoid-robotics/chapter5",
                    "source": "Humanoid Robotics Book",
                    "chunk_index": 78
                }
            }
        ],
        "query": request.get("query", ""),
        "total_results": 2
    }
```

**Remove mock after Feature 2 is implemented.**

### Option B: Mock httpx Responses in Tests

Use `pytest` with `respx` library for mocking:

```python
import pytest
import respx
import httpx

@pytest.mark.asyncio
@respx.mock
async def test_agent_with_mock_search():
    # Mock /search endpoint response
    respx.post("http://localhost:8000/search").mock(
        return_value=httpx.Response(200, json={
            "results": [{"text": "...", "score": 0.9, "metadata": {...}}]
        })
    )

    # Test agent
    result = await Runner.run(agent, "What is ZMP?")
    assert "[Source 1]" in result.final_output
```

---

## Troubleshooting

### Issue 1: "Module 'agents' not found"

**Cause**: `openai-agents` package not installed.

**Solution**:
```bash
pip install openai-agents
```

### Issue 2: "OpenAI API key not found"

**Cause**: `OPENAI_API_KEY` not in environment.

**Solution**:
1. Check `.env` file exists and has key
2. Restart FastAPI server to reload environment
3. Verify with: `python -c "import os; from dotenv import load_dotenv; load_dotenv(); print(os.getenv('OPENAI_API_KEY'))"`

### Issue 3: "Connection refused to http://localhost:8000/search"

**Cause**: FastAPI server not running or Feature 2 not implemented.

**Solution**:
1. Start server: `uvicorn main:app --reload --port 8000`
2. Verify /search endpoint exists: `curl http://localhost:8000/docs`
3. Use mock /search if Feature 2 not ready (see "Development Without Feature 2")

### Issue 4: Agent responses have no citations

**Cause**: Grounding instructions not enforced or retrieval failed.

**Solution**:
1. Check agent initialization includes `GROUNDING_INSTRUCTIONS`
2. Verify retrieval tool is returning formatted results with `[Source N]` markers
3. Check agent logs for retrieval errors
4. Enable debug logging: `DEBUG=true` in `.env`

### Issue 5: "Session database locked"

**Cause**: Multiple processes accessing SQLite database simultaneously.

**Solution**:
1. Stop all FastAPI server instances
2. Delete `backend/data/conversations.db`
3. Restart single server instance

### Issue 6: High latency (>10 seconds per query)

**Cause**: /search endpoint slow, OpenAI API slow, or network issues.

**Solution**:
1. Test /search latency independently: `time curl -X POST http://localhost:8000/search ...`
2. Check OpenAI API status: https://status.openai.com
3. Enable verbose logging to identify bottleneck
4. Consider using GPT-3.5-turbo for faster (but lower quality) responses

---

## Next Steps

### For Users
1. **Frontend Integration**: Build chat UI that calls `/agent/chat`
2. **Session Management**: Implement session ID generation strategy
3. **Citation UI**: Display clickable source links in responses

### For Developers
1. **Testing**: Run comprehensive test suite (grounding, citation, error tests)
2. **Monitoring**: Add logging and metrics for production
3. **Optimization**: Implement caching, streaming responses
4. **Guardrails**: Add input topic filtering (optional)

---

## API Reference

### POST /agent/chat

**Purpose**: Chat with RAG agent (dual-mode support).

**Request**:
```json
{
  "message": "string (1-2000 chars, required)",
  "session_id": "string (alphanumeric + _-, required)",
  "context_text": "string (optional, max 10000 chars) | null"
}
```

**Response** (200 OK):
```json
{
  "response": "string (agent answer with citations)",
  "session_id": "string (echo from request)",
  "status": "success | guardrail_triggered | error"
}
```

**Errors**:
- 400: Validation error (invalid request)
- 422: Guardrail triggered (off-topic question)
- 500: Internal error (retrieval failure, OpenAI error)

### GET /agent/session/{session_id}

**Purpose**: Retrieve conversation history.

**Response** (200 OK):
```json
{
  "session_id": "string",
  "messages": [
    {
      "role": "user | assistant | tool",
      "content": "string",
      "timestamp": "ISO 8601 datetime",
      "tool_call_id": "string | null"
    }
  ],
  "total_count": "integer"
}
```

**Errors**:
- 404: Session not found

### DELETE /agent/session/{session_id}

**Purpose**: Clear conversation history.

**Response**: 204 No Content (success)

**Errors**:
- 404: Session not found

### GET /health

**Purpose**: Check service health.

**Response** (200 OK if healthy):
```json
{
  "status": "healthy | unhealthy",
  "timestamp": "ISO 8601 datetime",
  "checks": {
    "openai_api": "healthy | unhealthy",
    "search_endpoint": "healthy | unhealthy",
    "session_database": "healthy | unhealthy"
  },
  "version": "string"
}
```

**Response** (503 Service Unavailable if unhealthy):
- Same schema, but `status: "unhealthy"` and one or more checks failed

---

## Configuration Reference

### Environment Variables

| Variable | Required | Description | Example |
|----------|----------|-------------|---------|
| `OPENAI_API_KEY` | Yes | OpenAI API key | `sk-proj-...` |
| `QDRANT_URL` | Yes (Feature 1) | Qdrant Cloud URL | `https://cluster.qdrant.io` |
| `QDRANT_API_KEY` | Yes (Feature 1) | Qdrant API key | `your-key` |
| `COHERE_API_KEY` | Yes (Feature 1) | Cohere API key | `your-key` |
| `DEBUG` | No | Enable verbose logging | `true` or `false` (default) |

### Agent Configuration

Default configuration in `backend/agent_config.py`:

```python
# Model selection
MODEL = "gpt-4o"  # Options: gpt-4o, gpt-4-turbo, gpt-3.5-turbo

# Max turns (prevent infinite loops)
MAX_TURNS = 10

# Retrieval settings (in tool)
DEFAULT_TOP_K = 5  # Number of chunks to retrieve
RETRIEVAL_TIMEOUT = 10.0  # Seconds

# Session settings
SESSION_DB_PATH = "data/conversations.db"
MAX_HISTORY_LENGTH = 50  # Messages to keep in session
```

---

## Cost Estimation

### OpenAI API Costs (GPT-4o)

**Pricing** (as of Dec 2025):
- Input: $2.50 per 1M tokens
- Output: $10.00 per 1M tokens

**Typical Query Breakdown**:
- System instructions: 800 tokens (input)
- Tool schema: 200 tokens (input)
- User message: 100 tokens (input)
- Retrieval results: 2500 tokens (input)
- Agent response: 500 tokens (output)
- **Total**: 3600 input + 500 output = ~$0.014 per query

**Monthly Estimates**:
- 100 queries/day: ~$42/month
- 500 queries/day: ~$210/month
- 1000 queries/day: ~$420/month

**Cost Reduction Strategies**:
1. Use GPT-3.5-turbo for simple queries (~70% cheaper)
2. Limit conversation history length
3. Reduce retrieval top_k (fewer chunks = less input tokens)
4. Implement caching for repeated queries

---

## Support

**Issues**: https://github.com/your-org/humanoid-robotics/issues
**Documentation**: `/specs/3-openai-agent/` directory
**API Docs**: http://localhost:8000/docs (when server running)

---

**Guide Version**: 1.0.0
**Last Updated**: 2025-12-15
**Next**: Proceed to `/sp.tasks` for task breakdown and implementation
