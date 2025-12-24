# Research: OpenAI Agents SDK RAG Agent

**Branch**: `3-openai-agent` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)

## Overview

This document captures technical research findings for implementing an AI agent using the OpenAI Agents SDK with retrieval-augmented answering. Research focused on understanding SDK capabilities, integration patterns with FastAPI, retrieval tool design, response grounding strategies, and conversation management.

## Research Questions

### RQ1: OpenAI Agents SDK Capabilities and FastAPI Integration

**Question**: How does the OpenAI Agents SDK work, and how can it be integrated with our existing FastAPI backend?

**Findings**:

#### SDK Architecture
- **Package**: `openai-agents` (Python SDK, version 0.6.3+)
- **Installation**: `pip install openai-agents`
- **Purpose**: Lightweight framework for building multi-agent workflows with tool calling, sessions, and tracing
- **Key Components**:
  - `Agent`: Core building block configured with instructions, tools, and model
  - `Runner`: Execution engine for orchestrating agent runs
  - `@function_tool`: Decorator for converting Python functions into callable tools
  - `Session`: Conversation history management (SQLite, Redis, OpenAI Conversations API)
  - `Guardrails`: Input/output validation with tripwire mechanism

#### Basic Agent Pattern
```python
from agents import Agent, Runner

agent = Agent(
    name="Assistant",
    instructions="You are a helpful assistant.",
    tools=[get_weather]
)

async def main():
    result = await Runner.run(agent, "What's the weather in Tokyo?")
    print(result.final_output)
```

#### FastAPI Integration Strategy

**Option 1: Async Endpoint with Direct Runner Execution** (RECOMMENDED)
```python
from fastapi import FastAPI
from agents import Agent, Runner, SQLiteSession

app = FastAPI()

# Initialize agent at module level (reuse across requests)
agent = Agent(
    name="RAG Assistant",
    instructions="...",
    tools=[retrieval_tool]
)

@app.post("/agent/chat")
async def chat(request: ChatRequest):
    # Create or retrieve session for conversation continuity
    session = SQLiteSession(request.session_id, "conversations.db")

    # Run agent asynchronously
    result = await Runner.run(
        agent,
        request.message,
        session=session
    )

    return {"response": result.final_output}
```

**Advantages**:
- Native async support (FastAPI and OpenAI SDK both async)
- No blocking calls in event loop
- Session management handles conversation history automatically
- Simple error handling with try/except

**Option 2: Sync Endpoint with Runner.run_sync()** (Alternative)
```python
@app.post("/agent/chat")
def chat_sync(request: ChatRequest):
    result = Runner.run_sync(agent, request.message)
    return {"response": result.final_output}
```

**Disadvantages**:
- Blocks event loop during agent execution
- May impact FastAPI performance under load
- Less idiomatic for async FastAPI applications

#### Dependency Management
- **Environment Variable**: `OPENAI_API_KEY` (required for SDK)
- **Existing Dependencies**: FastAPI, Cohere, Qdrant already installed
- **New Dependencies**: `openai-agents` only
- **No Conflicts**: OpenAI SDK and Cohere SDK can coexist

**Recommendation**: Use Option 1 (async endpoint) with `SQLiteSession` for conversation management and agent instance reuse at module level.

---

### RQ2: Retrieval Tool Interface Connected to /search Endpoint

**Question**: How should we define the retrieval tool that calls the Feature 2 `/search` endpoint to fetch relevant book chunks?

**Findings**:

#### Function Tool Decorator Pattern
The OpenAI Agents SDK uses the `@function_tool` decorator to automatically generate tool schemas from Python functions with type annotations and docstrings.

**Basic Retrieval Tool Structure**:
```python
from agents import function_tool
from typing import Annotated
import httpx

@function_tool
async def retrieve_book_content(
    query: Annotated[str, "The user's question about humanoid robotics"],
    top_k: Annotated[int, "Number of chunks to retrieve (1-10)"] = 5
) -> str:
    """Retrieve relevant content from the humanoid robotics book.

    This tool searches the embedded book content using semantic similarity
    and returns the most relevant passages to answer the user's question.

    Args:
        query: The user's question about humanoid robotics
        top_k: Number of relevant chunks to retrieve (default 5)

    Returns:
        Formatted string with retrieved chunks and metadata
    """
    # Call backend /search endpoint
    async with httpx.AsyncClient() as client:
        response = await client.post(
            "http://localhost:8000/search",
            json={"query": query, "top_k": top_k}
        )
        data = response.json()

    # Format results for agent consumption
    if not data["results"]:
        return "No relevant content found for this query."

    formatted_results = []
    for i, chunk in enumerate(data["results"], 1):
        formatted_results.append(
            f"[Source {i}] (Score: {chunk['score']:.3f}, URL: {chunk['metadata']['url']})\n"
            f"{chunk['text']}\n"
        )

    return "\n".join(formatted_results)
```

#### Tool Schema Generation
The SDK automatically generates JSON schema from:
- **Function name**: `retrieve_book_content` → tool name
- **Docstring**: First line → tool description
- **Args section**: Parameter descriptions
- **Type annotations**: Parameter types and constraints
- **Annotated types**: Additional parameter documentation

**Generated Schema** (automatic):
```json
{
  "type": "function",
  "function": {
    "name": "retrieve_book_content",
    "description": "Retrieve relevant content from the humanoid robotics book...",
    "parameters": {
      "type": "object",
      "properties": {
        "query": {"type": "string", "description": "The user's question..."},
        "top_k": {"type": "integer", "description": "Number of chunks...", "default": 5}
      },
      "required": ["query"]
    }
  }
}
```

#### Error Handling in Tools
The SDK supports custom error handlers for graceful failure:

```python
from agents import function_tool, RunContextWrapper
from typing import Any

def retrieval_error_handler(context: RunContextWrapper[Any], error: Exception) -> str:
    """Custom error message when retrieval fails."""
    print(f"Retrieval tool failed: {error}")
    return "I encountered an error while searching the book content. Please try rephrasing your question."

@function_tool(failure_error_function=retrieval_error_handler)
async def retrieve_book_content(query: str, top_k: int = 5) -> str:
    # Implementation with potential failures
    ...
```

#### Alternative: Using httpx vs requests
- **httpx**: Async-native HTTP client (RECOMMENDED)
  - Advantages: Native async/await, no blocking
  - Installation: `pip install httpx`

- **requests**: Synchronous HTTP client
  - Disadvantages: Blocks event loop, requires run_in_executor workaround

**Recommendation**: Use `@function_tool` decorator with `httpx.AsyncClient` for async /search endpoint calls. Include custom error handler for retrieval failures.

---

### RQ3: Agent System Instructions for Response Grounding

**Question**: How should we design system instructions to enforce retrieval-first answering and prevent hallucination?

**Findings**:

#### System Instructions (Developer Message)
The `instructions` parameter (also called "system prompt" or "developer message") guides agent behavior. Can be static string or dynamic function.

**Static Instructions Pattern** (RECOMMENDED for clarity):
```python
GROUNDING_INSTRUCTIONS = """You are a Humanoid Robotics Expert Assistant with strict grounding requirements.

**YOUR ROLE**:
- Answer questions about humanoid robotics using ONLY the book content retrieved via the retrieve_book_content tool
- Provide accurate, well-cited responses grounded strictly in retrieved passages

**MANDATORY RETRIEVAL POLICY**:
1. For EVERY user question, you MUST call the retrieve_book_content tool FIRST
2. NEVER answer questions without retrieving relevant content
3. If the retrieve_book_content tool returns "No relevant content found", respond with:
   "I couldn't find relevant information in the book to answer your question. Could you rephrase or ask about a different topic covered in the humanoid robotics textbook?"

**RESPONSE GROUNDING RULES**:
1. Base your answers STRICTLY on the retrieved content - do NOT use external knowledge
2. If retrieved content is insufficient or contradictory, acknowledge the limitation
3. NEVER make up information, speculate, or fill gaps with general knowledge
4. When uncertain, say "The book content doesn't provide enough detail on this specific aspect"

**CITATION REQUIREMENTS**:
1. Include inline citations for all factual claims: [Source 1], [Source 2], etc.
2. The retrieve_book_content tool returns numbered sources - reference them explicitly
3. At the end of your response, list all sources with URLs under a "Sources:" section

**EXAMPLE RESPONSE FORMAT**:
"According to the retrieved content, humanoid robots use zero-moment point (ZMP) for balance control [Source 1]. The ZMP represents the point where the total inertia force equals zero [Source 2].

Sources:
- [Source 1] https://example.com/chapter3
- [Source 2] https://example.com/chapter5"

**CONTEXT-CONSTRAINED MODE**:
- If the user provides selected text with their question (indicated by context in their message), answer based ONLY on that text without calling retrieve_book_content
- Still apply grounding rules and cite the provided context

Remember: Accuracy and honesty about limitations are more important than appearing knowledgeable. When in doubt, acknowledge what the book content does and doesn't cover."""

agent = Agent(
    name="Humanoid Robotics Assistant",
    instructions=GROUNDING_INSTRUCTIONS,
    tools=[retrieve_book_content]
)
```

#### Dynamic Instructions Pattern (Alternative)
For runtime context injection:
```python
def get_dynamic_instructions(agent: Agent, ctx: RunContextWrapper) -> str:
    base_instructions = GROUNDING_INSTRUCTIONS

    # Add dynamic context (e.g., user preferences, current date)
    user_info = ctx.context.get("user_info", {})
    if user_info.get("expertise_level") == "beginner":
        base_instructions += "\n\nUSER LEVEL: Explain concepts in simple terms, avoiding jargon."

    return base_instructions

agent = Agent(
    name="Humanoid Robotics Assistant",
    instructions=get_dynamic_instructions,
    tools=[retrieve_book_content]
)
```

#### Guardrails for Additional Safety
The SDK supports input/output guardrails with tripwire mechanism:

```python
from agents import input_guardrail, GuardrailFunctionOutput
from pydantic import BaseModel

class RelevanceCheck(BaseModel):
    is_robotics_question: bool
    reasoning: str

guardrail_agent = Agent(
    name="Relevance Checker",
    instructions="Determine if the question is about humanoid robotics.",
    output_type=RelevanceCheck
)

@input_guardrail
async def robotics_topic_guardrail(ctx, agent, input_data):
    result = await Runner.run(guardrail_agent, input_data, context=ctx.context)
    output = result.final_output_as(RelevanceCheck)

    return GuardrailFunctionOutput(
        output_info=output,
        tripwire_triggered=not output.is_robotics_question
    )

# Apply to main agent
agent = Agent(
    name="Humanoid Robotics Assistant",
    instructions=GROUNDING_INSTRUCTIONS,
    tools=[retrieve_book_content],
    input_guardrails=[robotics_topic_guardrail]  # Optional: filter off-topic questions
)
```

**Recommendation**: Use detailed static instructions with explicit grounding rules, mandatory retrieval policy, and citation requirements. Optional guardrails for topic filtering if off-topic queries are a concern.

---

### RQ4: Conversation History and Session Management

**Question**: How should we manage multi-turn conversations and maintain context across user queries?

**Findings**:

#### Session Management Options

**Option 1: SQLiteSession** (RECOMMENDED for single-server deployment)
```python
from agents import SQLiteSession

# File-based persistence
session = SQLiteSession(
    session_id="user_123_conversation_456",
    db_path="conversations.db"
)

# In-memory (testing only)
session = SQLiteSession(session_id="test_session")
```

**Advantages**:
- Simple file-based storage
- No external dependencies
- Automatic history retrieval and storage
- Works with `async def` FastAPI endpoints
- Built-in CRUD operations: `get_items()`, `add_items()`, `pop_item()`, `clear_session()`

**Usage Pattern**:
```python
@app.post("/agent/chat")
async def chat(request: ChatRequest):
    # Each user session has unique ID
    session = SQLiteSession(
        session_id=request.session_id,
        db_path="conversations.db"
    )

    # Runner.run automatically:
    # 1. Retrieves conversation history from session
    # 2. Appends new user message
    # 3. Executes agent with full context
    # 4. Stores agent response back to session
    result = await Runner.run(agent, request.message, session=session)

    return {"response": result.final_output}
```

**Option 2: RedisSession** (for distributed/scaled deployments)
```python
from agents.extensions.memory import RedisSession

session = RedisSession.from_url(
    session_id="user_123",
    url="redis://localhost:6379/0"
)
```

**Advantages**:
- Distributed storage for multi-server setups
- Better performance at scale
- Shared session state across API instances

**Disadvantages**:
- Requires Redis server
- Additional infrastructure complexity

**Option 3: OpenAIConversationsSession** (OpenAI hosted)
```python
from agents import OpenAIConversationsSession

# New conversation
session = OpenAIConversationsSession()

# Resume existing conversation
session = OpenAIConversationsSession(conversation_id="conv_123")
```

**Advantages**:
- Hosted by OpenAI
- No local storage management

**Disadvantages**:
- Requires OpenAI API dependency for storage
- Less control over data retention

#### Session Operations

**Retrieve History**:
```python
# Get all messages in session
history = await session.get_items()

# Get limited recent messages
recent = await session.get_items(limit=10)

# Get specific range with offset
page = await session.get_items(offset=20, limit=10)
```

**Manual History Manipulation** (if needed):
```python
# Add messages manually
await session.add_items([
    {"role": "user", "content": "Hello"},
    {"role": "assistant", "content": "Hi there!"}
])

# Remove last message
last_msg = await session.pop_item()

# Clear entire session
await session.clear_session()
```

#### Advanced Session Features (AdvancedSQLiteSession)

For analytics and debugging:
```python
from agents.extensions.memory import AdvancedSQLiteSession

session = AdvancedSQLiteSession(
    session_id="conversation_123",
    db_path="conversations.db",
    create_tables=True
)

# After each run, store usage metrics
result = await Runner.run(agent, "...", session=session)
await session.store_run_usage(result)

# Query analytics
tool_usage = await session.get_tool_usage()
for tool_name, count, turn in tool_usage:
    print(f"{tool_name}: used {count} times in turn {turn}")

# Search conversation
matching_turns = await session.find_turns_by_content("robotics")
```

#### Session ID Strategy

**Recommendation**: Use compound session IDs for multi-user scenarios:
```python
# Pattern: {user_id}_{conversation_id}
session_id = f"user_{request.user_id}_conv_{request.conversation_id}"
session = SQLiteSession(session_id, "conversations.db")
```

**Benefits**:
- Isolates users from each other
- Allows multiple conversations per user
- Easy to query by user or conversation

**Recommendation**: Use `SQLiteSession` with file-based persistence (`conversations.db`) for initial deployment. Use compound session IDs (`user_{id}_conv_{id}`). Consider `AdvancedSQLiteSession` if usage analytics are needed.

---

### RQ5: Citation Extraction and Source Transparency

**Question**: How should we extract citations from retrieved content and format them in agent responses?

**Findings**:

#### Two-Part Strategy: Tool Output + System Instructions

**Part 1: Structured Tool Output with Source Metadata**

The retrieval tool should return content in a citation-ready format:

```python
@function_tool
async def retrieve_book_content(query: str, top_k: int = 5) -> str:
    """Retrieve relevant content with source citations."""

    # Call /search endpoint
    response_data = await call_search_endpoint(query, top_k)

    if not response_data["results"]:
        return "No relevant content found for this query."

    # Format with explicit source markers
    formatted_chunks = []
    for i, chunk in enumerate(response_data["results"], 1):
        source_id = f"Source {i}"
        score = chunk["score"]
        url = chunk["metadata"]["url"]
        text = chunk["text"]

        # Include source metadata in output
        formatted_chunks.append(
            f"[{source_id}]\n"
            f"URL: {url}\n"
            f"Relevance: {score:.3f}\n"
            f"Content: {text}\n"
        )

    # Prepend source index for easy reference
    output = "Retrieved content from the book:\n\n"
    output += "\n---\n".join(formatted_chunks)
    output += "\n\nInstructions: Reference sources using [Source 1], [Source 2], etc. in your response."

    return output
```

**Example Tool Output**:
```
Retrieved content from the book:

[Source 1]
URL: https://example.com/chapter3
Relevance: 0.892
Content: Zero-moment point (ZMP) is a key concept in humanoid robot balance...

---

[Source 2]
URL: https://example.com/chapter5
Relevance: 0.854
Content: The ZMP represents the point where the sum of all moments equals zero...

Instructions: Reference sources using [Source 1], [Source 2], etc. in your response.
```

**Part 2: Enforce Citation in System Instructions**

Include explicit citation requirements in agent instructions:

```python
CITATION_INSTRUCTIONS = """
**CITATION REQUIREMENTS**:
1. The retrieve_book_content tool returns numbered sources ([Source 1], [Source 2], etc.)
2. When you use information from a source, cite it inline: [Source 1]
3. You can cite multiple sources for a single claim: [Source 1, 2]
4. At the end of your response, include a "Sources:" section listing all URLs

**Example**:
"Humanoid robots use ZMP for balance [Source 1]. The ZMP calculation requires real-time sensor data [Source 2].

Sources:
- [Source 1] https://example.com/chapter3
- [Source 2] https://example.com/chapter5"

**IMPORTANT**: Every factual claim MUST have an inline citation.
"""
```

#### Structured Output Alternative (Optional)

For programmatic citation extraction, use Pydantic structured outputs:

```python
from pydantic import BaseModel, Field
from typing import List

class Citation(BaseModel):
    source_number: int
    url: str

class GroundedResponse(BaseModel):
    answer: str = Field(description="The main answer with inline [Source N] citations")
    sources: List[Citation] = Field(description="List of all sources referenced")
    confidence: str = Field(description="High/Medium/Low based on retrieval scores")

agent = Agent(
    name="Humanoid Robotics Assistant",
    instructions=GROUNDING_INSTRUCTIONS,
    tools=[retrieve_book_content],
    output_type=GroundedResponse  # Force structured output
)

# In FastAPI endpoint
result = await Runner.run(agent, request.message, session=session)
response = result.final_output_as(GroundedResponse)

return {
    "answer": response.answer,
    "sources": [{"source_number": c.source_number, "url": c.url} for c in response.sources],
    "confidence": response.confidence
}
```

**Advantages**:
- Programmatically parseable citations
- Guaranteed source list presence
- Enables frontend citation UI (e.g., clickable source links)

**Disadvantages**:
- More rigid output format
- May reduce natural language quality
- Requires model support for structured outputs (GPT-4+)

#### Verification Strategy

**Post-Processing Check** (optional guardrail):
```python
@output_guardrail
async def citation_guardrail(ctx, agent, output: str) -> GuardrailFunctionOutput:
    """Verify that response includes citations."""
    has_inline_citations = "[Source" in output
    has_sources_section = "Sources:" in output

    is_valid = has_inline_citations and has_sources_section

    return GuardrailFunctionOutput(
        output_info={"has_citations": is_valid},
        tripwire_triggered=not is_valid  # Reject uncited responses
    )
```

**Recommendation**: Use structured tool output with explicit source markers `[Source N]` and enforce citation requirements via system instructions. Optionally use `GroundedResponse` Pydantic model for programmatic citation extraction if frontend needs structured sources.

---

### RQ6: Error Handling and Failure Scenarios

**Question**: What error handling patterns should we implement for retrieval failures, API errors, and edge cases?

**Findings**:

#### Tool-Level Error Handling

**Custom Error Handler for Retrieval Tool**:
```python
from agents import function_tool, RunContextWrapper
from typing import Any
import httpx

def handle_retrieval_error(context: RunContextWrapper[Any], error: Exception) -> str:
    """Graceful error message when retrieval fails."""
    print(f"[ERROR] Retrieval tool failed: {type(error).__name__}: {error}")

    if isinstance(error, httpx.TimeoutException):
        return "The search is taking longer than expected. Please try again in a moment."
    elif isinstance(error, httpx.HTTPStatusError):
        if error.response.status_code == 404:
            return "The search service is currently unavailable. Please try again later."
        elif error.response.status_code >= 500:
            return "The search service encountered an internal error. Please try again later."

    # Generic fallback
    return "I encountered an error while searching the book. Please try rephrasing your question or contact support if this persists."

@function_tool(failure_error_function=handle_retrieval_error)
async def retrieve_book_content(query: str, top_k: int = 5) -> str:
    """Retrieve relevant content with error handling."""
    try:
        async with httpx.AsyncClient(timeout=10.0) as client:
            response = await client.post(
                "http://localhost:8000/search",
                json={"query": query, "top_k": top_k}
            )
            response.raise_for_status()
            data = response.json()

            # Handle empty results
            if not data["results"]:
                return "No relevant content found for this query. The book may not cover this specific topic."

            # Format and return
            return format_retrieval_results(data["results"])

    except httpx.TimeoutException as e:
        raise  # Will be caught by failure_error_function
    except httpx.HTTPStatusError as e:
        raise
    except Exception as e:
        print(f"Unexpected error in retrieval tool: {e}")
        raise
```

#### Runner-Level Error Handling

**FastAPI Endpoint Error Handling**:
```python
from fastapi import HTTPException
from agents import Runner, SQLiteSession
from agents.exceptions import InputGuardrailTripwireTriggered, OutputGuardrailTripwireTriggered

@app.post("/agent/chat")
async def chat(request: ChatRequest):
    try:
        session = SQLiteSession(request.session_id, "conversations.db")

        result = await Runner.run(
            agent,
            request.message,
            session=session,
            max_turns=10  # Prevent infinite loops
        )

        return {
            "response": result.final_output,
            "session_id": request.session_id,
            "status": "success"
        }

    except InputGuardrailTripwireTriggered as e:
        # User input blocked by guardrail
        return {
            "response": "Your question appears to be outside the scope of this assistant. Please ask about humanoid robotics topics covered in the textbook.",
            "session_id": request.session_id,
            "status": "guardrail_triggered"
        }

    except OutputGuardrailTripwireTriggered as e:
        # Agent output blocked by guardrail (e.g., missing citations)
        return {
            "response": "I apologize, but I couldn't generate a properly cited response. Please try rephrasing your question.",
            "session_id": request.session_id,
            "status": "citation_validation_failed"
        }

    except Exception as e:
        print(f"[ERROR] Agent execution failed: {e}")
        raise HTTPException(
            status_code=500,
            detail="An error occurred while processing your request. Please try again."
        )
```

#### SDK-Specific Exceptions

The OpenAI Agents SDK provides specific exception types:
- `InputGuardrailTripwireTriggered`: Input validation failed
- `OutputGuardrailTripwireTriggered`: Output validation failed
- `RealtimeError`: Realtime agent errors (not applicable for our use case)

#### Logging and Debugging

**Enable Verbose Logging**:
```python
from agents import enable_verbose_stdout_logging

# Enable for development/debugging
if os.getenv("DEBUG", "false").lower() == "true":
    enable_verbose_stdout_logging()
```

**Structured Logging**:
```python
import logging

logger = logging.getLogger(__name__)

@function_tool
async def retrieve_book_content(query: str, top_k: int = 5) -> str:
    logger.info(f"Retrieval tool called: query='{query}', top_k={top_k}")

    try:
        results = await fetch_from_search(query, top_k)
        logger.info(f"Retrieved {len(results)} chunks, scores: {[r['score'] for r in results]}")
        return format_results(results)
    except Exception as e:
        logger.error(f"Retrieval failed: {type(e).__name__}: {e}", exc_info=True)
        raise
```

#### Edge Cases to Handle

1. **Empty Retrieval Results**: Return explicit message instead of letting agent hallucinate
2. **Low Relevance Scores**: Check if all scores < threshold (e.g., 0.4) and warn user
3. **Session Corruption**: Wrap session operations in try/except and fall back to stateless mode
4. **Token Limits**: Set `max_turns` in Runner.run() to prevent infinite tool call loops
5. **API Rate Limits**: Implement exponential backoff in retrieval tool

**Recommendation**: Use custom `failure_error_function` for retrieval tool, handle SDK exceptions in FastAPI endpoint, enable verbose logging in development, and implement edge case checks for empty results and low relevance scores.

---

### RQ7: Token Limits, Context Windows, and Performance

**Question**: How should we manage token limits, context window constraints, and optimize agent performance?

**Findings**:

#### Model and Context Window Defaults

**OpenAI Agents SDK Model Selection**:
```python
agent = Agent(
    name="RAG Assistant",
    instructions="...",
    tools=[retrieve_book_content],
    model="gpt-4-turbo",  # Default if not specified
)

# Alternative models
# model="gpt-4o"           # 128k context window (RECOMMENDED for RAG)
# model="gpt-4-turbo"      # 128k context window
# model="gpt-3.5-turbo"    # 16k context window (cheaper, less capable)
```

**Context Window Sizes**:
- **GPT-4o**: 128,000 tokens (~96,000 words)
- **GPT-4-turbo**: 128,000 tokens
- **GPT-3.5-turbo**: 16,385 tokens

**Typical Token Consumption**:
- System instructions: ~500-800 tokens (our detailed grounding instructions)
- Tool schema: ~100-200 tokens per tool
- User message: ~50-500 tokens (average query)
- Retrieval tool output: ~1000-3000 tokens (5 chunks × 200-600 tokens each)
- Agent response: ~200-800 tokens
- Conversation history: ~100-500 tokens per turn

**Example Budget**:
```
System instructions:      800 tokens
Tool schema:              200 tokens
Conversation history:   3,000 tokens (6 turns)
Current user query:       100 tokens
Retrieval results:      2,500 tokens (5 chunks)
Agent response:           400 tokens
------------------------
Total:                  7,000 tokens (~5% of 128k window)
```

**Conclusion**: With GPT-4o's 128k window, token limits are NOT a concern for typical RAG use cases with <20 turn conversations.

#### Session History Management

**Automatic Truncation** (if needed for long conversations):
```python
# Get recent history only
session = SQLiteSession(session_id, "conversations.db")
history = await session.get_items(limit=20)  # Last 20 messages only

# Manual truncation for very long sessions
if len(history) > 50:
    # Keep only last 40 messages
    recent_history = history[-40:]
    await session.clear_session()
    await session.add_items(recent_history)
```

**Session Input Callback** (advanced):
```python
from agents import Session

def truncate_history(session_history: list, new_input: list) -> list:
    """Custom function to manage context window."""
    # Keep only last 30 messages + new input
    recent = session_history[-30:] if len(session_history) > 30 else session_history
    return recent + new_input

session = Session(
    session_id="...",
    session_input_callback=truncate_history
)
```

#### Controlling Agent Execution

**Max Turns Limit** (prevent infinite loops):
```python
result = await Runner.run(
    agent,
    request.message,
    session=session,
    max_turns=10  # Maximum tool call iterations
)
```

**Typical Turn Counts**:
- Simple query: 1-2 turns (retrieve → respond)
- Complex query: 2-4 turns (retrieve → analyze → clarify → respond)
- Edge case: 5-10 turns (multiple retrieval attempts, refinements)

**Recommendation**: Set `max_turns=10` to prevent runaway costs while allowing complex multi-step reasoning.

#### Performance Optimization Strategies

**1. Agent Instance Reuse**:
```python
# GOOD: Create agent once at module level
agent = Agent(name="RAG Assistant", instructions="...", tools=[...])

@app.post("/agent/chat")
async def chat(request: ChatRequest):
    result = await Runner.run(agent, ...)  # Reuse agent

# BAD: Create agent per request
@app.post("/agent/chat")
async def chat(request: ChatRequest):
    agent = Agent(...)  # Wasteful recreation
    result = await Runner.run(agent, ...)
```

**2. Connection Pooling for httpx**:
```python
# Shared httpx client with connection pooling
HTTP_CLIENT = httpx.AsyncClient(
    timeout=10.0,
    limits=httpx.Limits(max_keepalive_connections=5, max_connections=10)
)

@function_tool
async def retrieve_book_content(query: str, top_k: int = 5) -> str:
    response = await HTTP_CLIENT.post(...)  # Reuse client
    ...
```

**3. SQLiteSession File Location**:
```python
# Use absolute path with proper directory
import os
DB_PATH = os.path.join(os.path.dirname(__file__), "data", "conversations.db")
session = SQLiteSession(session_id, DB_PATH)
```

**4. Retrieval Result Caching** (optional):
```python
from functools import lru_cache
import hashlib

# Simple in-memory cache for identical queries
RETRIEVAL_CACHE = {}

async def retrieve_with_cache(query: str, top_k: int) -> str:
    cache_key = hashlib.md5(f"{query}:{top_k}".encode()).hexdigest()

    if cache_key in RETRIEVAL_CACHE:
        print(f"[CACHE HIT] {query[:50]}...")
        return RETRIEVAL_CACHE[cache_key]

    results = await fetch_from_search(query, top_k)
    formatted = format_results(results)

    RETRIEVAL_CACHE[cache_key] = formatted
    return formatted
```

**Note**: Caching may reduce accuracy for dynamic content but improves performance for repeated queries.

#### Latency Expectations

**Typical Response Times**:
- Retrieval tool execution: 300-800ms (Qdrant search + Cohere embedding)
- OpenAI API call: 1-3 seconds (GPT-4o response generation)
- Total end-to-end: 1.5-4 seconds per user query

**Optimization Priority**:
1. Use async operations throughout (FastAPI, httpx, OpenAI SDK all async)
2. Minimize retrieval tool latency (optimize /search endpoint)
3. Consider streaming responses for better UX (see RQ8)

**Recommendation**: Use GPT-4o model (128k context window), set `max_turns=10`, reuse agent instance at module level, use connection pooling for httpx client. Token limits are not a concern for typical RAG scenarios.

---

### RQ8: Streaming Responses (Optional Enhancement)

**Question**: Should we implement streaming responses for better user experience?

**Findings**:

#### Streaming Support in OpenAI Agents SDK

The SDK supports streaming via `RunResultStreaming`:

```python
from agents import Agent, Runner

result = await Runner.run(agent, "Tell me about humanoid robots", stream=True)

# Iterate through streaming events
async for event in result.stream_events():
    if event.type == "text":
        print(event.data, end="", flush=True)  # Stream text chunks
    elif event.type == "tool_call":
        print(f"\n[Calling {event.tool_name}...]")
    elif event.type == "tool_result":
        print(f"[Tool completed]")
```

#### FastAPI SSE Integration

```python
from fastapi.responses import StreamingResponse
from agents import Runner

@app.post("/agent/chat/stream")
async def chat_stream(request: ChatRequest):
    async def event_generator():
        session = SQLiteSession(request.session_id, "conversations.db")
        result = await Runner.run(agent, request.message, session=session, stream=True)

        async for event in result.stream_events():
            if event.type == "text":
                # Send text chunks as SSE
                yield f"data: {json.dumps({'type': 'chunk', 'content': event.data})}\n\n"
            elif event.type == "tool_call":
                yield f"data: {json.dumps({'type': 'tool_start', 'tool': event.tool_name})}\n\n"

        yield f"data: {json.dumps({'type': 'done'})}\n\n"

    return StreamingResponse(event_generator(), media_type="text/event-stream")
```

**Advantages**:
- Better UX (user sees response as it's generated)
- Perceived lower latency
- Real-time visibility into tool calls

**Disadvantages**:
- More complex frontend implementation (SSE handling)
- Harder to debug
- Session management requires buffering complete response

**Recommendation**: Implement non-streaming endpoint first (simpler, easier to test). Add streaming as optional enhancement in Phase 2 if user feedback indicates latency concerns.

---

## Technical Decisions Summary

Based on research findings, the following technical decisions are recommended for implementation:

| Decision Area | Recommendation | Rationale |
|--------------|----------------|-----------|
| **FastAPI Integration** | Async POST endpoint with Runner.run() | Native async support, no blocking |
| **Session Management** | SQLiteSession with file persistence | Simple, no external dependencies |
| **Retrieval Tool** | @function_tool with httpx.AsyncClient | Async HTTP, automatic schema generation |
| **System Instructions** | Static detailed instructions with grounding rules | Clear policy enforcement, citation requirements |
| **Citation Strategy** | Structured tool output + instruction enforcement | Programmatically parseable, LLM-enforced |
| **Error Handling** | Custom failure_error_function + FastAPI exception handlers | Graceful degradation, user-friendly messages |
| **Model Selection** | GPT-4o (128k context) | Sufficient context window, best quality |
| **Token Management** | max_turns=10, no truncation needed | 128k window accommodates typical sessions |
| **Performance** | Agent instance reuse, httpx connection pooling | Minimize overhead, maximize throughput |
| **Streaming** | Defer to Phase 2 (optional) | Prioritize correctness over latency initially |

---

## Dependencies and Environment Setup

### Required Python Packages
```bash
# Existing dependencies (already installed)
fastapi>=0.104.0
uvicorn[standard]>=0.27.0
qdrant-client>=1.8.0
cohere>=5.5.0
python-dotenv>=1.0.0

# New dependency for OpenAI Agents SDK
openai-agents>=0.6.3

# New dependency for async HTTP in retrieval tool
httpx>=0.26.0
```

### Environment Variables
```bash
# Existing (from Feature 1)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-key
COHERE_API_KEY=your-cohere-key

# New for OpenAI Agents SDK
OPENAI_API_KEY=your-openai-key

# Optional
DEBUG=false  # Set to true for verbose logging
```

### Database Files
```
backend/
├── main.py
├── data/
│   └── conversations.db  # SQLiteSession storage (created automatically)
└── .env
```

---

## Implementation Roadmap

Based on research findings, the implementation should proceed in this order:

### Phase 1: Core Agent Setup
1. Install `openai-agents` and `httpx` dependencies
2. Create retrieval tool function with `@function_tool` decorator
3. Define grounding system instructions
4. Initialize agent with tools and instructions
5. Test agent locally with mock /search responses

### Phase 2: FastAPI Integration
6. Create `/agent/chat` POST endpoint
7. Integrate SQLiteSession for conversation management
8. Implement request/response Pydantic models
9. Add error handling for guardrails and API failures
10. Test multi-turn conversations via FastAPI `/docs`

### Phase 3: Validation and Testing
11. Create test suite for agent behavior (grounding, citations, refusals)
12. Validate retrieval tool integration with live /search endpoint
13. Performance testing (latency, token usage)
14. Edge case testing (empty results, low relevance, errors)

### Phase 4: Optional Enhancements
15. Add structured output with GroundedResponse Pydantic model
16. Implement streaming endpoint (optional)
17. Add input guardrails for topic filtering (optional)
18. Implement retrieval caching (optional)

---

## Open Questions for Design Phase

1. **Dual-Mode Implementation**: How should we detect "selected text" context vs. "full book" mode?
   - Option A: Separate `/agent/chat` and `/agent/explain` endpoints
   - Option B: Single endpoint with optional `context_text` field in request
   - **Recommendation**: Option B (simpler, single agent configuration)

2. **Citation Format**: Should we use inline `[Source 1]` or footnote-style `[1]`?
   - **Recommendation**: `[Source 1]` for clarity and LLM understanding

3. **Low Relevance Threshold**: What score threshold should trigger "no relevant content" response?
   - Research shows typical high-quality matches have scores >0.7
   - **Recommendation**: If all scores <0.4, trigger refusal to answer

4. **Conversation Retention**: How long should sessions persist in SQLite?
   - **Recommendation**: Indefinite persistence, add cleanup endpoint for testing

5. **Model Selection**: GPT-4o vs. GPT-4-turbo vs. GPT-3.5-turbo?
   - **Recommendation**: GPT-4o for best grounding and citation accuracy

These questions will be resolved during the design and planning phases.

---

## References

- OpenAI Agents SDK Documentation: https://openai.github.io/openai-agents-python/
- OpenAI Agents SDK GitHub: https://github.com/openai/openai-agents-python
- OpenAI Agents SDK PyPI: https://pypi.org/project/openai-agents/
- FastAPI Documentation: https://fastapi.tiangolo.com/
- httpx Documentation: https://www.python-httpx.org/

---

**Research Completed**: 2025-12-15
**Next Step**: Create data model definitions (data-model.md) and API contracts (contracts/api-contract.yaml)
