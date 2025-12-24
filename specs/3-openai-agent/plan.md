# Implementation Plan: OpenAI Agents SDK RAG Agent

**Branch**: `3-openai-agent` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/3-openai-agent/spec.md`

## Summary

Implement an AI agent using the OpenAI Agents SDK with retrieval-augmented generation (RAG) to answer user questions about humanoid robotics. The agent enforces strict response grounding, provides transparent citations, supports multi-turn conversations, and operates in dual modes: full-book search (with retrieval) and context-constrained explanations (without retrieval).

**Core Functionality**:
- Initialize OpenAI Agents SDK agent with grounding system instructions
- Define retrieval tool that calls backend `/search` endpoint for semantic search
- Implement FastAPI endpoint (`/agent/chat`) for agent interactions
- Manage conversation history with SQLiteSession for multi-turn context
- Enforce citation requirements and hallucination prevention policies
- Support dual-mode operation (full-book vs. selected-text-only)

**Technical Approach** (from research):
- Use `@function_tool` decorator for retrieval function with httpx async HTTP calls
- Agent instance reuse at module level for performance
- Detailed static system instructions for grounding enforcement
- Session-based conversation management (no manual history tracking)
- GPT-4o model with 128k context window (no truncation needed)

**Dependencies on Feature 2**:
- **CRITICAL**: `/search` endpoint from Feature 2 (rag-retrieval) must be implemented first
- Agent retrieval tool makes HTTP POST to `http://localhost:8000/search`
- If Feature 2 not complete, use mock /search responses for development

## Technical Context

**Language/Version**: Python 3.13+
**Primary Dependencies**:
- Existing: FastAPI 0.104.0+, qdrant-client 1.8.0+, cohere 5.5.0+, python-dotenv 1.0.0+
- New: `openai-agents>=0.6.3`, `httpx>=0.26.0`

**External APIs**:
- OpenAI API (GPT-4o model for agent execution)
- Backend /search endpoint (Feature 2 - semantic retrieval)

**Storage**:
- SQLite database (`conversations.db`) for session management
- No additional Qdrant or Cohere dependencies (delegated to /search endpoint)

**Testing**:
- Manual testing via FastAPI `/docs` interactive UI
- Automated test suite in `/sp.tasks` phase (agent behavior validation)

**Target Platform**: Linux/Windows server with Python 3.13+
**Project Type**: Monolithic FastAPI application (extend `backend/main.py`)
**Performance Goals**:
- <4s end-to-end latency per query (retrieval + LLM generation)
- Support 50+ turn conversations without truncation
- <2% guardrail false positive rate

**Constraints**:
- OpenAI API costs (~$0.01-0.03 per query with GPT-4o)
- Token limit: 128k (not a concern for typical sessions)
- /search endpoint latency affects total response time

**Scale/Scope**:
- 4 new endpoints (`/agent/chat`, `/agent/chat/structured`, `/agent/session/{id}`, `/health`)
- ~800 LOC addition (agent setup, tools, endpoints, error handling)
- 1 new dependency installation (`openai-agents`, `httpx`)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle V: AI-Native Authoring Workflow ✅

- ✅ Development follows Spec-Driven Development using SpecKit-Plus
- ✅ Planning preceded generation (spec → research → design → plan)
- ✅ All artifacts are human-auditable and editable (Markdown, Python, YAML)
- ✅ PHR will be created for this planning session

### Principle VI: Code & Simulation Standards ✅

- ✅ Python as supported language
- ✅ Type hints with Pydantic models for all request/response payloads
- ✅ Async/await throughout (FastAPI, httpx, OpenAI SDK)
- ✅ Error handling with specific exception types
- ✅ Logging for debugging and observability

### Principle II: Grounding and Verification ✅

- ✅ **STRICT GROUNDING POLICY**: Agent responses grounded strictly in retrieved content
- ✅ **CITATION TRANSPARENCY**: Inline citations with source URLs mandatory
- ✅ **HALLUCINATION PREVENTION**: Refusal to answer when no relevant content found
- ✅ **VERIFIABLE SOURCES**: All claims traceable to book URLs
- ✅ **ERROR BOUNDARIES**: Graceful degradation on retrieval failures

### Principle VII: Dependency, Modularity, and Extensibility ✅

- ✅ Minimal new dependencies (`openai-agents`, `httpx`)
- ✅ Reuse existing FastAPI, Qdrant, Cohere infrastructure
- ✅ Modular design: tools, models, endpoints separately defined
- ✅ Extensible: guardrails, structured outputs, streaming as future enhancements

---

## Scope and Dependencies

### In Scope

1. **Agent Initialization**
   - Install `openai-agents` and `httpx` packages
   - Configure OpenAI API key in environment
   - Create agent instance with grounding system instructions
   - Define retrieval tool with `@function_tool` decorator

2. **Retrieval Tool Implementation**
   - HTTP POST to `/search` endpoint with query and top_k
   - Format retrieval results with source markers `[Source 1]`, `[Source 2]`, etc.
   - Error handling for retrieval failures (timeout, HTTP errors)
   - Custom error messages for graceful degradation

3. **FastAPI Endpoints**
   - POST `/agent/chat`: Main chat endpoint with dual-mode support
   - POST `/agent/chat/structured`: Structured response with explicit citations (optional)
   - GET `/agent/session/{session_id}`: Retrieve conversation history
   - DELETE `/agent/session/{session_id}`: Clear session history
   - GET `/health`: Service health check with dependency status

4. **Session Management**
   - SQLiteSession for conversation persistence
   - Automatic history retrieval and storage
   - Session ID validation and compound ID strategy (`user_{id}_conv_{id}`)
   - Session cleanup for testing/privacy

5. **Request/Response Models**
   - Pydantic models for type safety and validation
   - `ChatRequest`, `ChatRequestWithContext`, `ChatResponse`, `GroundedResponse`
   - OpenAPI schema generation for `/docs`

6. **Error Handling**
   - Custom `failure_error_function` for retrieval tool
   - FastAPI exception handlers for SDK exceptions
   - User-friendly error messages for all failure scenarios
   - Logging for debugging and monitoring

7. **Dual-Mode Operation**
   - Full-book mode: `context_text=None` → retrieval-first answering
   - Context-constrained mode: `context_text="..."` → no retrieval, explain provided text

8. **Citation Enforcement**
   - System instructions require inline citations
   - Structured tool output with source metadata
   - Sources section at end of responses
   - Optional output guardrail for citation validation

### Out of Scope

1. **Frontend Integration**: No UI implementation (API-only)
2. **Streaming Responses**: Deferred to Phase 2 (optional enhancement)
3. **Advanced Guardrails**: Input topic filtering deferred (optional)
4. **Retrieval Caching**: Query result caching deferred (performance optimization)
5. **User Authentication**: API key auth deferred to production deployment
6. **Rate Limiting**: OpenAI/backend rate limiting handled externally
7. **Analytics Dashboard**: Usage metrics and conversation analytics deferred
8. **Multi-Language Support**: English-only for initial release

### External Dependencies

1. **OpenAI API** (external service)
   - **Ownership**: OpenAI
   - **Purpose**: GPT-4o model for agent execution
   - **SLA**: 99.9% uptime (OpenAI commitment)
   - **Dependency Type**: Critical (agent cannot function without it)
   - **Fallback**: None (service degradation on OpenAI outage)
   - **Configuration**: `OPENAI_API_KEY` environment variable

2. **Backend /search Endpoint** (Feature 2 - internal dependency)
   - **Ownership**: Our team (Feature 2 implementation)
   - **Purpose**: Semantic retrieval of book content
   - **Availability**: Requires Feature 2 completion
   - **Dependency Type**: Critical for full-book mode
   - **Fallback**: Mock responses for development, graceful error in production
   - **Configuration**: `http://localhost:8000/search` (hardcoded initially)

3. **SQLite Database** (local storage)
   - **Ownership**: Built-in Python library
   - **Purpose**: Conversation history persistence
   - **Dependency Type**: Non-critical (can fall back to stateless mode)
   - **Fallback**: In-memory session or no history
   - **Configuration**: `conversations.db` file path

---

## Key Architecture Decisions

### AD-001: Extend Existing FastAPI Application

**Decision**: Extend `backend/main.py` with agent endpoints rather than creating separate microservice.

**Options Considered**:
1. **Monolithic Extension** (CHOSEN): Add agent routes to existing FastAPI app
2. **Separate Microservice**: Create new FastAPI app in `backend/agent/main.py`
3. **Serverless Functions**: Deploy agent as AWS Lambda/Cloud Function

**Rationale**:
- Simpler deployment (single service)
- Reuse existing FastAPI infrastructure, middleware, CORS configuration
- Easier development and testing (one server to run)
- /search endpoint already in same app (no cross-service HTTP)
- Future: Can extract to microservice if needed for scaling

**Trade-offs**:
- (+) Simplicity, shared configuration, faster development
- (-) Less isolated, agent failures could affect ingestion endpoints
- (-) Harder to scale independently

**Implementation**:
```python
# backend/main.py
from fastapi import FastAPI
from agents import Agent, Runner, SQLiteSession

app = FastAPI()  # Existing app

# ... existing ingestion endpoints ...

# NEW: Agent endpoints
@app.post("/agent/chat")
async def chat(request: ChatRequest):
    ...
```

---

### AD-002: Use @function_tool Decorator for Retrieval

**Decision**: Define retrieval tool using `@function_tool` decorator with automatic schema generation.

**Options Considered**:
1. **@function_tool Decorator** (CHOSEN): Automatic schema from type annotations
2. **Manual FunctionTool**: Manually define JSON schema with `FunctionTool(...)`
3. **Agent-as-Tool**: Create separate retrieval agent and use `agent.as_tool()`

**Rationale**:
- Type annotations + docstrings = zero boilerplate
- Automatic validation via Pydantic
- Simpler to maintain and test
- Standard pattern in OpenAI Agents SDK documentation

**Trade-offs**:
- (+) Less code, automatic schema generation, better developer experience
- (-) Less control over schema (acceptable for our use case)

**Implementation**:
```python
from agents import function_tool
import httpx

@function_tool
async def retrieve_book_content(
    query: Annotated[str, "User's question about humanoid robotics"],
    top_k: Annotated[int, "Number of chunks to retrieve"] = 5
) -> str:
    """Retrieve relevant content from humanoid robotics book.

    Args:
        query: The user's question
        top_k: Number of results (1-10)

    Returns:
        Formatted chunks with source citations
    """
    # Call /search endpoint
    async with httpx.AsyncClient(timeout=10.0) as client:
        response = await client.post(
            "http://localhost:8000/search",
            json={"query": query, "top_k": top_k}
        )
        data = response.json()

    # Format with [Source N] markers
    return format_retrieval_results(data["results"])
```

---

### AD-003: Static System Instructions for Grounding

**Decision**: Use detailed static system instructions rather than dynamic function.

**Options Considered**:
1. **Static Instructions String** (CHOSEN): Predefined comprehensive grounding policy
2. **Dynamic Instructions Function**: Runtime-generated instructions based on context
3. **Minimal Instructions**: Short system prompt, rely on retrieval tool design

**Rationale**:
- Grounding policy is fixed (not context-dependent)
- Easier to review, audit, and version control
- Better performance (no function call overhead per request)
- Explicit policy reduces hallucination risk

**Trade-offs**:
- (+) Clarity, auditability, performance, easier testing
- (-) Less flexible (acceptable - grounding rules shouldn't change per request)

**Implementation**:
```python
GROUNDING_INSTRUCTIONS = """You are a Humanoid Robotics Expert Assistant with strict grounding requirements.

**MANDATORY RETRIEVAL POLICY**:
1. For EVERY user question, you MUST call the retrieve_book_content tool FIRST
2. NEVER answer questions without retrieving relevant content
3. If no relevant content found, respond: "I couldn't find relevant information..."

**RESPONSE GROUNDING RULES**:
1. Base answers STRICTLY on retrieved content - NO external knowledge
2. If retrieved content is insufficient, acknowledge the limitation
3. NEVER speculate or fill gaps with general knowledge

**CITATION REQUIREMENTS**:
1. Include inline citations: [Source 1], [Source 2]
2. List all sources with URLs at the end

... (full instructions in research.md RQ3)
"""

agent = Agent(
    name="Humanoid Robotics Assistant",
    instructions=GROUNDING_INSTRUCTIONS,
    tools=[retrieve_book_content],
    model="gpt-4o"
)
```

---

### AD-004: SQLiteSession for Conversation Management

**Decision**: Use `SQLiteSession` with file-based persistence for conversation history.

**Options Considered**:
1. **SQLiteSession (file-based)** (CHOSEN): `conversations.db` file
2. **RedisSession**: External Redis server for distributed deployments
3. **OpenAIConversationsSession**: OpenAI-hosted conversation management
4. **Manual History Management**: `to_input_list()` pattern from SDK docs

**Rationale**:
- Simple setup (no external service required)
- Sufficient for single-server deployment
- Automatic history retrieval and storage
- Built-in CRUD operations (`get_items`, `clear_session`, etc.)
- Can migrate to Redis later if scaling needed

**Trade-offs**:
- (+) Zero config, no external dependencies, persistent storage
- (-) Single server only (no distributed scaling)
- (-) File-based storage (acceptable for <1000 sessions/day)

**Implementation**:
```python
from agents import SQLiteSession

@app.post("/agent/chat")
async def chat(request: ChatRequest):
    session = SQLiteSession(
        session_id=request.session_id,
        db_path="data/conversations.db"
    )

    result = await Runner.run(
        agent,
        request.message,
        session=session,
        max_turns=10
    )

    return ChatResponse(
        response=result.final_output,
        session_id=request.session_id,
        status="success"
    )
```

---

### AD-005: Dual-Mode via Single Endpoint with Optional Field

**Decision**: Implement dual-mode (full-book vs. context-constrained) using single `/agent/chat` endpoint with optional `context_text` field.

**Options Considered**:
1. **Single Endpoint with Optional Field** (CHOSEN): `ChatRequestWithContext` model
2. **Separate Endpoints**: `/agent/chat` (full-book) and `/agent/explain` (context)
3. **Mode Parameter**: `mode` field with enum ["full_book", "context_constrained"]

**Rationale**:
- Simpler API surface (one endpoint instead of two)
- Natural request structure (`context_text` presence = mode switch)
- Single agent configuration (instructions handle both modes)
- Frontend logic simpler (same endpoint, different payload)

**Trade-offs**:
- (+) Simplicity, fewer endpoints, clearer semantics
- (-) Slightly larger request model (acceptable with Pydantic validation)

**Implementation**:
```python
class ChatRequestWithContext(BaseModel):
    message: str
    session_id: str
    context_text: Optional[str] = None  # None = full-book, string = context mode

@app.post("/agent/chat")
async def chat(request: ChatRequestWithContext):
    if request.context_text:
        # Context-constrained mode: prepend context to message
        enhanced_message = (
            f"[CONTEXT-CONSTRAINED MODE]\n"
            f"Given text: {request.context_text}\n"
            f"User question: {request.message}"
        )
    else:
        # Full-book mode: normal message
        enhanced_message = request.message

    result = await Runner.run(agent, enhanced_message, session=session)
    return ChatResponse(...)
```

---

### AD-006: httpx for Async HTTP in Retrieval Tool

**Decision**: Use `httpx.AsyncClient` for calling `/search` endpoint in retrieval tool.

**Options Considered**:
1. **httpx (async)** (CHOSEN): Native async HTTP client
2. **requests (sync)**: Standard synchronous HTTP library
3. **aiohttp**: Alternative async HTTP client

**Rationale**:
- Native async/await support (no event loop blocking)
- Modern API (similar to `requests`)
- Connection pooling support
- Timeout configuration built-in
- Widely adopted in async Python ecosystem

**Trade-offs**:
- (+) Performance, non-blocking, good developer experience
- (-) New dependency (but lightweight)

**Implementation**:
```python
import httpx

# Shared client with connection pooling
HTTP_CLIENT = httpx.AsyncClient(
    timeout=10.0,
    limits=httpx.Limits(max_keepalive_connections=5)
)

@function_tool
async def retrieve_book_content(query: str, top_k: int = 5) -> str:
    response = await HTTP_CLIENT.post(
        "http://localhost:8000/search",
        json={"query": query, "top_k": top_k}
    )
    response.raise_for_status()
    return format_results(response.json())
```

---

### AD-007: Agent Instance Reuse at Module Level

**Decision**: Create agent instance once at module level and reuse across requests.

**Options Considered**:
1. **Module-Level Instance** (CHOSEN): Create agent outside endpoint functions
2. **Per-Request Instance**: Create new agent for each request
3. **Singleton Pattern**: Lazy initialization with global lock

**Rationale**:
- Agent initialization is expensive (tool schema generation)
- Instructions and tools are static (no per-request variation)
- Better performance (no repeated initialization)
- Standard pattern in FastAPI applications

**Trade-offs**:
- (+) Performance, simplicity, resource efficiency
- (-) Less flexible for dynamic agent configuration (not needed)

**Implementation**:
```python
# backend/main.py

# Agent initialized once at module level
agent = Agent(
    name="Humanoid Robotics Assistant",
    instructions=GROUNDING_INSTRUCTIONS,
    tools=[retrieve_book_content],
    model="gpt-4o"
)

# Reused across all requests
@app.post("/agent/chat")
async def chat(request: ChatRequest):
    result = await Runner.run(agent, ...)  # Reuse instance
    ...
```

---

### AD-008: GPT-4o Model Selection

**Decision**: Use `gpt-4o` model with 128k context window.

**Options Considered**:
1. **gpt-4o** (CHOSEN): Latest GPT-4 optimized, 128k context, best quality
2. **gpt-4-turbo**: 128k context, slightly older
3. **gpt-3.5-turbo**: 16k context, cheaper, lower quality

**Rationale**:
- 128k context window (sufficient for 50+ turn conversations)
- Best grounding and citation accuracy
- Strong instruction following (critical for grounding policy)
- Structured output support (for `GroundedResponse`)
- Acceptable cost (~$0.01-0.03 per query)

**Trade-offs**:
- (+) Quality, context window, instruction following
- (-) Higher cost than GPT-3.5 (acceptable for quality requirements)

**Implementation**:
```python
agent = Agent(
    name="Humanoid Robotics Assistant",
    instructions=GROUNDING_INSTRUCTIONS,
    tools=[retrieve_book_content],
    model="gpt-4o"  # Explicit model selection
)
```

---

### AD-009: max_turns Limit for Safety

**Decision**: Set `max_turns=10` in `Runner.run()` to prevent infinite tool call loops.

**Options Considered**:
1. **max_turns=10** (CHOSEN): Allow complex reasoning, prevent runaway costs
2. **max_turns=5**: Stricter limit, lower cost
3. **No limit**: Rely on OpenAI timeout
4. **max_turns=20**: Higher complexity support

**Rationale**:
- Typical queries: 1-2 turns (retrieve → respond)
- Complex queries: 2-4 turns
- Edge cases: 5-10 turns (multiple retrieval attempts)
- Prevents infinite loops from buggy tool logic

**Trade-offs**:
- (+) Safety, cost control, reasonable for 99% of queries
- (-) May truncate very complex multi-step reasoning (acceptable)

**Implementation**:
```python
result = await Runner.run(
    agent,
    request.message,
    session=session,
    max_turns=10  # Safety limit
)
```

---

### AD-010: Custom Error Handler for Retrieval Tool

**Decision**: Implement custom `failure_error_function` for graceful retrieval failures.

**Options Considered**:
1. **Custom Error Handler** (CHOSEN): User-friendly messages for specific failures
2. **Default SDK Behavior**: Generic "Tool call failed" message
3. **Exception Propagation**: Let errors bubble to FastAPI exception handler

**Rationale**:
- User-friendly error messages for common failures (timeout, 404, 500)
- Prevents exposing internal error details to users
- Allows agent to continue conversation despite tool failure
- Better debugging with specific error logging

**Trade-offs**:
- (+) Better UX, security, debugging
- (-) Additional code (minimal)

**Implementation**:
```python
def handle_retrieval_error(ctx: RunContextWrapper[Any], error: Exception) -> str:
    """Graceful error messages for retrieval failures."""
    print(f"[ERROR] Retrieval failed: {error}")

    if isinstance(error, httpx.TimeoutException):
        return "The search is taking longer than expected. Please try again."
    elif isinstance(error, httpx.HTTPStatusError):
        if error.response.status_code >= 500:
            return "The search service is currently unavailable. Please try again later."

    return "I encountered an error while searching. Please rephrase your question."

@function_tool(failure_error_function=handle_retrieval_error)
async def retrieve_book_content(...):
    ...
```

---

## Implementation Strategy

### Phase 0: Dependency Setup
**Objective**: Install required packages and configure environment.

**Tasks**:
1. Add `openai-agents>=0.6.3` and `httpx>=0.26.0` to requirements.txt
2. Install packages: `pip install openai-agents httpx`
3. Add `OPENAI_API_KEY=your-key` to `.env`
4. Verify OpenAI API connectivity with test script
5. Create `backend/data/conversations.db` directory

**Acceptance Criteria**:
- ✅ Packages installed without conflicts
- ✅ `import agents` and `import httpx` succeed
- ✅ OpenAI API key validated with test call
- ✅ Conversations database directory exists

---

### Phase 1: Retrieval Tool Implementation
**Objective**: Create working retrieval tool that calls `/search` endpoint.

**Tasks**:
1. Create `backend/tools.py` module for agent tools
2. Implement `retrieve_book_content` function with `@function_tool` decorator
3. Add httpx POST call to `http://localhost:8000/search`
4. Implement result formatting with `[Source N]` markers
5. Add custom error handler with `failure_error_function`
6. Write unit tests for tool with mock /search responses

**Acceptance Criteria**:
- ✅ Tool callable by agent with `query` and `top_k` parameters
- ✅ Formats results with source citations
- ✅ Handles retrieval errors gracefully
- ✅ Unit tests pass with mock responses

**Example Output**:
```
Retrieved content from the book:

[Source 1]
URL: https://example.com/chapter3
Relevance: 0.892
Content: The zero-moment point (ZMP) is a fundamental concept...

---

[Source 2]
URL: https://example.com/chapter5
Relevance: 0.854
Content: ZMP is used for balance control during walking...

Instructions: Reference sources using [Source 1], [Source 2] in your response.
```

---

### Phase 2: Agent Initialization
**Objective**: Create agent instance with grounding instructions and tools.

**Tasks**:
1. Create `backend/agent_config.py` for agent configuration
2. Define `GROUNDING_INSTRUCTIONS` string with full policy (from research RQ3)
3. Initialize agent with instructions, tools, and model
4. Test agent locally with mock Runner.run() calls
5. Validate agent follows grounding policy and citation requirements

**Acceptance Criteria**:
- ✅ Agent initialized without errors
- ✅ Tool schema generated correctly
- ✅ Agent calls retrieval tool for questions
- ✅ Agent includes citations in responses
- ✅ Agent refuses to answer when no content found

**Test Queries**:
```python
# Test 1: Normal query (should retrieve and cite)
result = await Runner.run(agent, "What is the zero-moment point?")
assert "[Source" in result.final_output
assert "Sources:" in result.final_output

# Test 2: Off-topic query (should refuse)
result = await Runner.run(agent, "What is quantum entanglement?")
assert "couldn't find relevant information" in result.final_output.lower()
```

---

### Phase 3: Pydantic Models
**Objective**: Define request/response models for type safety and validation.

**Tasks**:
1. Create `backend/models.py` for all Pydantic models
2. Implement `ChatRequest`, `ChatRequestWithContext`, `ChatResponse`
3. Implement `Citation`, `GroundedResponse` (optional structured output)
4. Implement `ConversationMessage`, `ErrorDetail`, `ErrorResponse`
5. Add field validators (e.g., session_id pattern, context_text non-empty)
6. Test model validation with valid/invalid inputs

**Acceptance Criteria**:
- ✅ All models defined with proper type hints
- ✅ Field validation works as expected
- ✅ OpenAPI schema generated correctly (check with `/docs`)
- ✅ Example values provided for documentation

---

### Phase 4: FastAPI Endpoints
**Objective**: Implement main `/agent/chat` endpoint with session management.

**Tasks**:
1. Import agent, models, and session classes in `backend/main.py`
2. Implement POST `/agent/chat` endpoint
3. Add SQLiteSession initialization with request.session_id
4. Implement dual-mode logic (context_text check)
5. Add error handling for guardrails and SDK exceptions
6. Test endpoint via FastAPI `/docs` interface

**Acceptance Criteria**:
- ✅ Endpoint accepts `ChatRequestWithContext` and returns `ChatResponse`
- ✅ Full-book mode triggers retrieval tool
- ✅ Context-constrained mode skips retrieval
- ✅ Multi-turn conversations maintain context
- ✅ Errors return user-friendly messages

**Test Cases**:
```json
// Test 1: Full-book mode
POST /agent/chat
{
  "message": "What is the zero-moment point?",
  "session_id": "test_session_1",
  "context_text": null
}
// Expected: Response with citations from retrieved chunks

// Test 2: Context-constrained mode
POST /agent/chat
{
  "message": "Explain this in simple terms",
  "session_id": "test_session_2",
  "context_text": "The ZMP is the point where forces balance..."
}
// Expected: Response without retrieval, based on provided text

// Test 3: Multi-turn conversation
POST /agent/chat
{"message": "What is ZMP?", "session_id": "test_session_3", "context_text": null}
POST /agent/chat  // Same session_id
{"message": "What about balance control?", "session_id": "test_session_3", "context_text": null}
// Expected: Second response uses context from first question
```

---

### Phase 5: Session Management Endpoints
**Objective**: Add endpoints for session history retrieval and clearing.

**Tasks**:
1. Implement GET `/agent/session/{session_id}` endpoint
2. Retrieve session items with `session.get_items(limit=50)`
3. Format as `ConversationMessage` list with timestamps
4. Implement DELETE `/agent/session/{session_id}` endpoint
5. Call `session.clear_session()` for deletion
6. Add error handling for non-existent sessions

**Acceptance Criteria**:
- ✅ GET returns full conversation history
- ✅ DELETE clears session successfully
- ✅ 404 returned for non-existent sessions
- ✅ Timestamps included for all messages

---

### Phase 6: Health Check Endpoint
**Objective**: Add service health monitoring.

**Tasks**:
1. Implement GET `/health` endpoint
2. Check OpenAI API connectivity (test API call)
3. Check /search endpoint reachability (HTTP GET /health or HEAD /search)
4. Check SQLite database access (test query)
5. Return `HealthResponse` with individual check statuses

**Acceptance Criteria**:
- ✅ Returns 200 when all dependencies healthy
- ✅ Returns 503 when any dependency unhealthy
- ✅ Individual check statuses visible in response
- ✅ Useful for load balancer health checks

---

### Phase 7: Optional Structured Output (Enhancement)
**Objective**: Add `/agent/chat/structured` endpoint with explicit citations.

**Tasks**:
1. Create separate agent instance with `output_type=GroundedResponse`
2. Implement POST `/agent/chat/structured` endpoint
3. Extract structured output with `result.final_output_as(GroundedResponse)`
4. Return structured response with `sources` array
5. Test with frontend mock to validate citation extraction

**Acceptance Criteria**:
- ✅ Returns `GroundedResponse` with explicit citation list
- ✅ `sources` array contains all cited sources
- ✅ `confidence` field reflects retrieval scores
- ✅ `retrieval_mode` indicates full_book vs. context_constrained

**Note**: This is optional and can be deferred if basic endpoint works well.

---

### Phase 8: Testing and Validation
**Objective**: Comprehensive testing of agent behavior and edge cases.

**Tasks**:
1. Create test suite in `backend/tests/test_agent.py`
2. Test grounding policy (no hallucination on unknown topics)
3. Test citation requirements (all responses have sources)
4. Test dual-mode behavior (with/without context_text)
5. Test error handling (retrieval failures, timeouts, invalid inputs)
6. Test multi-turn conversations (follow-up questions, context retention)
7. Performance testing (latency, token usage)

**Test Categories**:
- **Grounding Tests**: Verify refusal when no relevant content
- **Citation Tests**: Verify all factual claims have inline citations
- **Dual-Mode Tests**: Verify correct mode selection based on context_text
- **Error Tests**: Verify graceful handling of all failure scenarios
- **Conversation Tests**: Verify multi-turn context maintenance
- **Performance Tests**: Verify latency <4s, token usage within budget

**Acceptance Criteria**:
- ✅ All unit tests pass
- ✅ No hallucination detected in 20+ test queries
- ✅ 100% citation coverage in responses
- ✅ Error messages user-friendly for all failures
- ✅ Average latency <3s for typical queries

---

## Data Flow Diagrams

### Full-Book Mode (with Retrieval)

```
┌─────────┐     1. POST /agent/chat          ┌──────────────┐
│ Frontend│────────────────────────────────>│ FastAPI      │
│         │     ChatRequestWithContext       │ /agent/chat  │
└─────────┘     context_text=null            └──────┬───────┘
                                                     │
                                                     │ 2. Get/Create Session
                                                     ▼
                                              ┌──────────────┐
                                              │ SQLiteSession│
                                              │ (history DB) │
                                              └──────┬───────┘
                                                     │
                                                     │ 3. Runner.run(agent, message, session)
                                                     ▼
                                              ┌──────────────┐
                                              │ OpenAI Agent │
                                              │  (GPT-4o)    │
                                              └──────┬───────┘
                                                     │
                                                     │ 4. Calls retrieve_book_content(query, top_k=5)
                                                     ▼
                                              ┌──────────────┐
                                              │ Retrieval    │
                                              │ Tool         │
                                              └──────┬───────┘
                                                     │
                                                     │ 5. HTTP POST /search
                                                     ▼
                                              ┌──────────────┐
                                              │ /search      │
                                              │ (Feature 2)  │
                                              └──────┬───────┘
                                                     │
                                                     │ 6. Returns chunks with scores
                                                     ▼
                                              ┌──────────────┐
                                              │ Format with  │
                                              │ [Source N]   │
                                              └──────┬───────┘
                                                     │
                                                     │ 7. Tool returns formatted string
                                                     ▼
                                              ┌──────────────┐
                                              │ OpenAI Agent │
                                              │ Generates    │
                                              │ Response     │
                                              └──────┬───────┘
                                                     │
                                                     │ 8. Session saves history
                                                     ▼
                                              ┌──────────────┐
                                              │ SQLiteSession│
                                              │ (save)       │
                                              └──────┬───────┘
                                                     │
                                                     │ 9. Return ChatResponse
                                                     ▼
┌─────────┐                                   ┌──────────────┐
│ Frontend│<──────────────────────────────────│ FastAPI      │
│         │     ChatResponse                  │ /agent/chat  │
└─────────┘     (with citations)              └──────────────┘
```

### Context-Constrained Mode (without Retrieval)

```
┌─────────┐     1. POST /agent/chat          ┌──────────────┐
│ Frontend│────────────────────────────────>│ FastAPI      │
│         │     context_text="The ZMP is..." │ /agent/chat  │
└─────────┘                                   └──────┬───────┘
                                                     │
                                                     │ 2. Prepend context to message
                                                     │    "[CONTEXT-CONSTRAINED MODE]
                                                     │     Given text: {context_text}
                                                     │     User question: {message}"
                                                     ▼
                                              ┌──────────────┐
                                              │ OpenAI Agent │
                                              │  (GPT-4o)    │
                                              └──────┬───────┘
                                                     │
                                                     │ 3. NO retrieval tool call
                                                     │    (context already in prompt)
                                                     ▼
                                              ┌──────────────┐
                                              │ Generates    │
                                              │ Explanation  │
                                              └──────┬───────┘
                                                     │
                                                     │ 4. Return response
                                                     ▼
┌─────────┐                                   ┌──────────────┐
│ Frontend│<──────────────────────────────────│ FastAPI      │
│         │     ChatResponse                  │ /agent/chat  │
└─────────┘     (explanation, no citations)   └──────────────┘
```

---

## Error Handling Strategy

### Error Categories and Responses

| Error Type | HTTP Status | User Message | Log Level |
|-----------|-------------|--------------|-----------|
| Invalid request (validation) | 400 | "message must be at least 1 character" | WARNING |
| Session not found | 404 | "No session found with ID: {id}" | INFO |
| Input guardrail triggered | 422 | "Your question is outside the scope..." | INFO |
| Retrieval timeout | 500 (fallback) | "Search taking longer than expected..." | ERROR |
| Retrieval 500 error | 500 (fallback) | "Search service unavailable..." | ERROR |
| OpenAI API error | 500 | "An error occurred processing your request..." | ERROR |
| Output guardrail triggered | 422 | "Couldn't generate properly cited response..." | WARNING |
| Unexpected error | 500 | "An unexpected error occurred..." | ERROR |

### Error Handling Flow

```python
@app.post("/agent/chat")
async def chat(request: ChatRequestWithContext):
    try:
        # 1. Session management
        session = SQLiteSession(request.session_id, "data/conversations.db")

        # 2. Dual-mode logic
        if request.context_text:
            enhanced_message = f"[CONTEXT-CONSTRAINED MODE]..."
        else:
            enhanced_message = request.message

        # 3. Run agent
        result = await Runner.run(agent, enhanced_message, session=session, max_turns=10)

        # 4. Return success
        return ChatResponse(
            response=result.final_output,
            session_id=request.session_id,
            status="success"
        )

    except InputGuardrailTripwireTriggered:
        # Input blocked (off-topic, etc.)
        return ChatResponse(
            response="Your question is outside the scope of this assistant.",
            session_id=request.session_id,
            status="guardrail_triggered"
        )

    except OutputGuardrailTripwireTriggered:
        # Output validation failed (missing citations, etc.)
        return ChatResponse(
            response="I couldn't generate a properly cited response. Please rephrase.",
            session_id=request.session_id,
            status="citation_validation_failed"
        )

    except Exception as e:
        # Unexpected error
        logger.error(f"Agent execution failed: {e}", exc_info=True)
        raise HTTPException(
            status_code=500,
            detail="An error occurred while processing your request."
        )
```

---

## Non-Functional Requirements

### Performance

**Latency Targets**:
- Single query: <4 seconds (p95)
- Concurrent queries (10): <6 seconds avg
- Session history retrieval: <200ms

**Optimization Strategies**:
- Agent instance reuse (avoid repeated initialization)
- httpx connection pooling (reduce HTTP overhead)
- SQLiteSession file-based storage (fast local access)

**Token Budget** (per query):
- System instructions: 800 tokens
- Tool schema: 200 tokens
- Conversation history: 3000 tokens (max)
- Retrieval results: 2500 tokens
- Agent response: 500 tokens
- **Total**: ~7000 tokens (~5% of 128k limit)

### Reliability

**Error Recovery**:
- Retrieval failures: Graceful error messages, no agent crash
- OpenAI API errors: Retry logic (SDK built-in)
- Session database corruption: Fall back to stateless mode

**SLO**:
- Availability: 99% (dependent on OpenAI API SLA)
- Error rate: <1% (excluding user validation errors)

### Security

**Data Handling**:
- No PII in conversation logs (except session_id)
- API keys stored in environment variables (never hardcoded)
- Session IDs validated with regex (prevent injection)

**AuthN/AuthZ**:
- Deferred to production (API key auth via X-API-Key header)
- Development: Open access (localhost only)

### Observability

**Logging**:
- INFO: Successful requests, session operations
- WARNING: Guardrail triggers, validation errors
- ERROR: Retrieval failures, OpenAI API errors, unexpected exceptions

**Metrics** (future):
- Request count, latency distribution
- Tool call frequency (retrieval tool usage)
- Token usage per query
- Error rate by type

---

## Risks and Mitigation

### Risk 1: Feature 2 Dependency (HIGH)

**Description**: Agent requires `/search` endpoint from Feature 2, which may not be implemented yet.

**Impact**: Blocking - agent cannot retrieve content without it.

**Mitigation**:
1. **Mock /search Endpoint**: Create temporary mock endpoint returning sample chunks for development
2. **Feature 2 Priority**: Implement Feature 2 tasks before Feature 3
3. **Parallel Development**: Use mock responses initially, integrate real /search when ready

**Mock Implementation**:
```python
@app.post("/search")  # Temporary mock
async def search_mock(request: dict):
    return {
        "results": [
            {
                "text": "The zero-moment point (ZMP) is...",
                "score": 0.892,
                "metadata": {"url": "https://example.com/chapter3"}
            }
        ],
        "query": request["query"],
        "total_results": 1
    }
```

### Risk 2: OpenAI API Costs (MEDIUM)

**Description**: GPT-4o costs ~$0.01-0.03 per query, may exceed budget with high usage.

**Impact**: Financial - unexpected costs if traffic spikes.

**Mitigation**:
1. **Rate Limiting**: Implement request throttling (e.g., 100 queries/user/day)
2. **Cost Monitoring**: Track OpenAI API usage via dashboard
3. **Model Fallback**: Option to use GPT-3.5-turbo for non-critical queries
4. **Token Optimization**: Minimize system instructions length, limit conversation history

**Budget Estimate**:
- Expected: 100 queries/day × $0.02 = $2/day = $60/month
- Contingency: 500 queries/day × $0.02 = $10/day = $300/month

### Risk 3: Hallucination Despite Grounding (MEDIUM)

**Description**: Agent may still hallucinate or provide uncited claims despite grounding instructions.

**Impact**: Quality - loss of user trust, incorrect information.

**Mitigation**:
1. **Output Guardrail**: Optional guardrail to validate citation presence
2. **Testing**: Comprehensive grounding tests with off-topic queries
3. **System Instructions**: Very explicit grounding policy with examples
4. **Structured Output**: Use `GroundedResponse` to force citation extraction
5. **User Feedback**: Add thumbs up/down for quality monitoring

**Guardrail Example**:
```python
@output_guardrail
async def citation_guardrail(ctx, agent, output: str) -> GuardrailFunctionOutput:
    has_citations = "[Source" in output and "Sources:" in output
    return GuardrailFunctionOutput(
        output_info={"has_citations": has_citations},
        tripwire_triggered=not has_citations
    )
```

### Risk 4: Session Database Growth (LOW)

**Description**: SQLite database may grow large with many long conversations.

**Impact**: Performance - slower session retrieval, disk space usage.

**Mitigation**:
1. **Session Expiration**: Delete sessions older than 30 days
2. **History Truncation**: Limit conversation history to last 50 messages
3. **Database Vacuuming**: Periodic SQLite VACUUM to reclaim space
4. **Migration Path**: Redis fallback if scaling needed

**Cleanup Script**:
```python
# Cron job: daily session cleanup
async def cleanup_old_sessions():
    # Delete sessions older than 30 days
    cutoff = datetime.now() - timedelta(days=30)
    # Implementation: query SQLite for old sessions and delete
```

### Risk 5: Retrieval Latency Bottleneck (LOW)

**Description**: /search endpoint latency may dominate total response time.

**Impact**: Performance - slow user experience if retrieval >2s.

**Mitigation**:
1. **Optimize /search**: Ensure Feature 2 implements caching, fast embedding
2. **Timeout Configuration**: Set httpx timeout=10s, fail fast if slow
3. **Parallel Retrieval**: Future enhancement to call /search for multiple queries in parallel
4. **User Feedback**: Loading indicator in frontend for transparency

---

## Testing Strategy

### Unit Tests

**Scope**: Individual functions and tools in isolation.

**Coverage**:
- `retrieve_book_content` tool with mock httpx responses
- Pydantic model validation (valid/invalid inputs)
- Error handler functions (retrieval_error_handler)
- Result formatting functions (format_retrieval_results)

**Example**:
```python
@pytest.mark.asyncio
async def test_retrieve_book_content_success(mock_httpx):
    mock_httpx.post.return_value.json.return_value = {
        "results": [
            {"text": "Sample text", "score": 0.9, "metadata": {"url": "http://example.com"}}
        ]
    }

    result = await retrieve_book_content("What is ZMP?", top_k=5)

    assert "[Source 1]" in result
    assert "http://example.com" in result
    assert "Sample text" in result
```

### Integration Tests

**Scope**: End-to-end API tests with real agent execution.

**Coverage**:
- POST /agent/chat with full-book mode
- POST /agent/chat with context-constrained mode
- Multi-turn conversations (session continuity)
- Error scenarios (invalid requests, retrieval failures)
- GET /agent/session/{id} and DELETE /agent/session/{id}
- GET /health endpoint

**Example**:
```python
async def test_chat_full_book_mode(client):
    response = await client.post("/agent/chat", json={
        "message": "What is the zero-moment point?",
        "session_id": "test_session_1",
        "context_text": None
    })

    assert response.status_code == 200
    data = response.json()
    assert "[Source" in data["response"]
    assert "Sources:" in data["response"]
    assert data["status"] == "success"
```

### Behavior Tests

**Scope**: Validate agent grounding policy and quality.

**Test Cases**:
1. **Grounding Test**: Ask off-topic question, verify refusal
2. **Citation Test**: Verify all factual claims have inline citations
3. **Hallucination Test**: Ask unanswerable question, verify acknowledgment
4. **Context Test**: Follow-up question uses previous answer
5. **Dual-Mode Test**: Same question with/without context_text, verify different responses

**Example**:
```python
async def test_grounding_policy(client):
    # Off-topic question
    response = await client.post("/agent/chat", json={
        "message": "What is quantum entanglement?",
        "session_id": "test_grounding",
        "context_text": None
    })

    data = response.json()
    assert "couldn't find relevant information" in data["response"].lower()
    assert "[Source" not in data["response"]  # No citations for refusal
```

### Performance Tests

**Scope**: Validate latency and token usage.

**Metrics**:
- Latency (p50, p95, p99)
- Token usage per query
- Concurrent request handling (10 simultaneous requests)

**Example**:
```python
async def test_latency():
    start = time.time()
    response = await client.post("/agent/chat", json={...})
    latency = time.time() - start

    assert latency < 4.0  # p95 target
    assert response.status_code == 200
```

---

## Rollback and Migration

**Rollback Strategy**:
- Feature 3 is additive (new endpoints only)
- Rollback: Remove agent endpoints, uninstall openai-agents package
- No database migrations required (SQLite session DB is isolated)

**Migration to Feature 4**:
- Feature 3 provides API foundation for frontend integration
- Frontend can consume /agent/chat endpoint directly
- Session management already in place for chat UI

---

## Documentation

**Deliverables**:
1. **quickstart.md**: Setup guide, environment configuration, example requests
2. **API Documentation**: Auto-generated via FastAPI `/docs` (OpenAPI schema)
3. **Testing Guide**: How to run tests, interpret results
4. **PHR**: Prompt History Record for this planning session

---

## Success Criteria

Feature 3 is considered complete when:

- ✅ All dependencies installed and configured
- ✅ Retrieval tool successfully calls /search endpoint
- ✅ Agent follows grounding policy (no hallucination)
- ✅ All responses include inline citations and Sources section
- ✅ POST /agent/chat endpoint works for both modes (full-book, context-constrained)
- ✅ Multi-turn conversations maintain context automatically
- ✅ Session management endpoints (GET/DELETE /agent/session/{id}) functional
- ✅ Health check endpoint returns dependency status
- ✅ Error handling graceful for all failure scenarios
- ✅ Latency <4s for typical queries (p95)
- ✅ Comprehensive test suite passes (grounding, citation, error, performance tests)
- ✅ Documentation complete (quickstart.md, API docs, PHR)

---

**Plan Completed**: 2025-12-15
**Next Step**: Create quickstart.md with setup and usage guide
