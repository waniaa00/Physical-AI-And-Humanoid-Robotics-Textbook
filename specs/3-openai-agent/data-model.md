# Data Model: OpenAI Agents SDK RAG Agent

**Branch**: `3-openai-agent` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)

## Overview

This document defines all Pydantic data models for the RAG agent API endpoints. Models provide type safety, validation, serialization, and OpenAPI documentation for both request and response payloads.

**Design Principles**:
- **Type Safety**: All fields use precise Python type annotations
- **Validation**: Pydantic validators for business rules (length limits, enum values, etc.)
- **Documentation**: Field descriptions for OpenAPI schema generation
- **Immutability**: Use frozen=True for value objects where applicable
- **Explicitness**: No implicit conversions or defaults that hide errors

---

## Request Models

### ChatRequest

**Purpose**: Basic chat request for full-book question answering mode.

**Usage**: POST /agent/chat

```python
from pydantic import BaseModel, Field

class ChatRequest(BaseModel):
    """Request model for agent chat endpoint.

    Represents a user question about humanoid robotics content,
    with session tracking for multi-turn conversation support.
    """

    message: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's question about humanoid robotics",
        examples=["What is the zero-moment point in humanoid robotics?"]
    )

    session_id: str = Field(
        ...,
        min_length=1,
        max_length=200,
        pattern=r"^[a-zA-Z0-9_\-]+$",
        description="Unique session identifier for conversation continuity",
        examples=["user_123_conv_456", "session_abc123"]
    )

    class Config:
        json_schema_extra = {
            "example": {
                "message": "What is the zero-moment point in humanoid robotics?",
                "session_id": "user_123_conv_456"
            }
        }
```

**Validation Rules**:
- `message`: Required, 1-2000 characters (prevents abuse, handles long queries)
- `session_id`: Required, alphanumeric with underscores/hyphens only (safe for file paths)

**Examples**:
```json
{
  "message": "What is the zero-moment point?",
  "session_id": "user_123_conv_456"
}
```

---

### ChatRequestWithContext

**Purpose**: Extended chat request supporting context-constrained mode (selected text explanations).

**Usage**: POST /agent/chat (same endpoint, context_text triggers dual-mode behavior)

```python
from pydantic import BaseModel, Field, field_validator
from typing import Optional

class ChatRequestWithContext(BaseModel):
    """Extended request model supporting both full-book and context-constrained modes.

    When context_text is provided, agent operates in context-constrained mode
    and does NOT call retrieval tool. Otherwise, operates in full-book mode
    with retrieval-first answering.
    """

    message: str = Field(
        ...,
        min_length=1,
        max_length=2000,
        description="User's question or request for explanation",
        examples=[
            "Explain this concept in simple terms",
            "What does ZMP mean in this context?"
        ]
    )

    session_id: str = Field(
        ...,
        min_length=1,
        max_length=200,
        pattern=r"^[a-zA-Z0-9_\-]+$",
        description="Unique session identifier for conversation continuity",
        examples=["user_123_conv_456"]
    )

    context_text: Optional[str] = Field(
        default=None,
        max_length=10000,
        description="Optional selected text from book for context-constrained answering. "
                    "When provided, agent will NOT perform retrieval and will answer "
                    "based solely on this text.",
        examples=[
            "The zero-moment point (ZMP) is the point on the ground where...",
            None
        ]
    )

    @field_validator('context_text')
    @classmethod
    def context_text_not_empty_string(cls, v: Optional[str]) -> Optional[str]:
        """Ensure context_text is None or non-empty."""
        if v is not None and v.strip() == "":
            raise ValueError("context_text must be None or contain non-whitespace content")
        return v

    class Config:
        json_schema_extra = {
            "examples": [
                {
                    "message": "What is the zero-moment point?",
                    "session_id": "user_123_conv_456",
                    "context_text": null
                },
                {
                    "message": "Explain this concept in simple terms",
                    "session_id": "user_123_conv_456",
                    "context_text": "The zero-moment point (ZMP) is the point on the ground where the total sum of inertial and gravitational forces equals zero..."
                }
            ]
        }
```

**Validation Rules**:
- `message`: Same as ChatRequest
- `session_id`: Same as ChatRequest
- `context_text`: Optional, max 10,000 characters (supports long book sections), no empty strings

**Dual-Mode Behavior**:
- `context_text=None`: Full-book mode → Agent calls `retrieve_book_content` tool
- `context_text="..."`: Context-constrained mode → Agent uses provided text only

**Examples**:

*Full-Book Mode*:
```json
{
  "message": "What is the zero-moment point?",
  "session_id": "user_123_conv_456",
  "context_text": null
}
```

*Context-Constrained Mode*:
```json
{
  "message": "Explain this concept in simple terms",
  "session_id": "user_123_conv_456",
  "context_text": "The zero-moment point (ZMP) is the point on the ground where the total sum of inertial and gravitational forces equals zero. It is used in humanoid robot control to maintain balance..."
}
```

---

## Response Models

### Citation

**Purpose**: Represents a single source citation with metadata.

```python
from pydantic import BaseModel, Field, HttpUrl

class Citation(BaseModel):
    """Represents a single source citation from retrieved content.

    Used in structured response format to provide programmatically
    accessible citation information.
    """

    source_number: int = Field(
        ...,
        ge=1,
        le=20,
        description="Source number (1-indexed) referenced as [Source N] in answer",
        examples=[1, 2]
    )

    url: str = Field(
        ...,
        description="URL of the book page or section where content was retrieved",
        examples=["https://example.com/humanoid-robotics/chapter3"]
    )

    score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Relevance score from vector search (0.0 to 1.0)",
        examples=[0.892, 0.745]
    )

    class Config:
        json_schema_extra = {
            "example": {
                "source_number": 1,
                "url": "https://example.com/humanoid-robotics/chapter3",
                "score": 0.892
            }
        }
```

**Validation Rules**:
- `source_number`: 1-20 (matches max retrieval results)
- `url`: Valid string (HttpUrl validation optional)
- `score`: 0.0-1.0 (Qdrant similarity score range)

---

### ChatResponse

**Purpose**: Basic chat response with string answer.

**Usage**: POST /agent/chat (default response format)

```python
from pydantic import BaseModel, Field

class ChatResponse(BaseModel):
    """Basic response model for agent chat endpoint.

    Returns agent's natural language response with inline citations.
    Session ID echoed back for client-side tracking.
    """

    response: str = Field(
        ...,
        description="Agent's answer with inline [Source N] citations and Sources section",
        examples=[
            "The zero-moment point (ZMP) is a key concept in humanoid robot balance [Source 1]. "
            "It represents the point where the sum of all moments equals zero [Source 2].\n\n"
            "Sources:\n- [Source 1] https://example.com/chapter3\n- [Source 2] https://example.com/chapter5"
        ]
    )

    session_id: str = Field(
        ...,
        description="Echo of session_id from request for client tracking",
        examples=["user_123_conv_456"]
    )

    status: str = Field(
        default="success",
        description="Request status: success, guardrail_triggered, citation_validation_failed, error",
        examples=["success", "guardrail_triggered"]
    )

    class Config:
        json_schema_extra = {
            "example": {
                "response": "The zero-moment point (ZMP) is used for balance control in humanoid robots [Source 1].\n\nSources:\n- [Source 1] https://example.com/chapter3",
                "session_id": "user_123_conv_456",
                "status": "success"
            }
        }
```

**Validation Rules**:
- `response`: Required string (no length limit, as citations can make responses long)
- `session_id`: Required string (echoed from request)
- `status`: One of "success", "guardrail_triggered", "citation_validation_failed", "error"

**Status Values**:
- `success`: Normal response generated with citations
- `guardrail_triggered`: Input guardrail blocked request (off-topic, etc.)
- `citation_validation_failed`: Output guardrail rejected response (missing citations)
- `error`: Unexpected error during processing

---

### GroundedResponse

**Purpose**: Structured response with programmatically accessible citations.

**Usage**: POST /agent/chat (optional, enabled via structured output)

```python
from pydantic import BaseModel, Field
from typing import List

class GroundedResponse(BaseModel):
    """Structured response model with explicit citation extraction.

    Provides programmatic access to citations for frontend rendering
    (e.g., clickable source links, citation highlighting).
    """

    answer: str = Field(
        ...,
        description="Agent's answer with inline [Source N] citations",
        examples=[
            "The zero-moment point (ZMP) is a key concept in humanoid robot balance [Source 1]. "
            "It represents the point where the sum of all moments equals zero [Source 2]."
        ]
    )

    sources: List[Citation] = Field(
        ...,
        min_length=0,
        max_length=20,
        description="List of all sources cited in the answer",
        examples=[[
            {"source_number": 1, "url": "https://example.com/chapter3", "score": 0.892},
            {"source_number": 2, "url": "https://example.com/chapter5", "score": 0.854}
        ]]
    )

    confidence: str = Field(
        ...,
        description="Agent's confidence level based on retrieval scores: High (>0.8), Medium (0.5-0.8), Low (<0.5)",
        examples=["High", "Medium", "Low"]
    )

    retrieval_mode: str = Field(
        ...,
        description="Mode used for answering: full_book (with retrieval) or context_constrained (without retrieval)",
        examples=["full_book", "context_constrained"]
    )

    class Config:
        json_schema_extra = {
            "example": {
                "answer": "The zero-moment point (ZMP) is used for balance control [Source 1].",
                "sources": [
                    {"source_number": 1, "url": "https://example.com/chapter3", "score": 0.892}
                ],
                "confidence": "High",
                "retrieval_mode": "full_book"
            }
        }
```

**Validation Rules**:
- `answer`: Required string with inline citations
- `sources`: List of 0-20 Citation objects (0 for context-constrained mode)
- `confidence`: "High", "Medium", or "Low" (could be enum)
- `retrieval_mode`: "full_book" or "context_constrained"

**Advantages Over ChatResponse**:
- Programmatic citation access for frontend
- Confidence indicator for UI treatment
- Mode transparency for debugging

**Disadvantages**:
- Requires structured output support (GPT-4+)
- More rigid format may reduce natural language quality

---

## Internal Models

### MessageRole

**Purpose**: Enum for conversation message roles.

```python
from enum import Enum

class MessageRole(str, Enum):
    """Message role in conversation history."""

    USER = "user"
    ASSISTANT = "assistant"
    SYSTEM = "system"
    TOOL = "tool"
```

---

### ConversationMessage

**Purpose**: Represents a single message in conversation history.

**Usage**: Internal representation for session storage and debugging.

```python
from pydantic import BaseModel, Field
from datetime import datetime
from typing import Optional

class ConversationMessage(BaseModel):
    """Represents a single message in conversation history.

    Used for session storage, debugging, and conversation export.
    """

    role: MessageRole = Field(
        ...,
        description="Role of message sender: user, assistant, system, or tool"
    )

    content: str = Field(
        ...,
        description="Message content (text for user/assistant, JSON for tool)"
    )

    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="UTC timestamp when message was created"
    )

    tool_call_id: Optional[str] = Field(
        default=None,
        description="Tool call ID for tool messages (correlates with tool invocations)"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "role": "user",
                "content": "What is the zero-moment point?",
                "timestamp": "2025-12-15T10:30:00Z",
                "tool_call_id": null
            }
        }
```

---

### RetrievalToolOutput

**Purpose**: Structured representation of retrieval tool output for parsing.

**Usage**: Internal parsing of tool output string into structured data.

```python
from pydantic import BaseModel, Field
from typing import List

class RetrievedChunk(BaseModel):
    """Single retrieved chunk from retrieval tool."""

    source_number: int = Field(..., ge=1)
    url: str
    score: float = Field(..., ge=0.0, le=1.0)
    text: str

class RetrievalToolOutput(BaseModel):
    """Parsed output from retrieve_book_content tool.

    Used internally to extract citations and metadata from
    the tool's string output.
    """

    chunks: List[RetrievedChunk] = Field(
        ...,
        description="List of retrieved chunks with metadata"
    )

    query: str = Field(
        ...,
        description="Original query used for retrieval"
    )

    top_k: int = Field(
        ...,
        ge=1,
        le=20,
        description="Number of results requested"
    )
```

**Note**: This model is for internal use only and not exposed in API responses.

---

## Error Response Models

### ErrorDetail

**Purpose**: Structured error information.

```python
from pydantic import BaseModel, Field
from typing import Optional

class ErrorDetail(BaseModel):
    """Detailed error information for API responses."""

    code: str = Field(
        ...,
        description="Error code for programmatic handling",
        examples=["RETRIEVAL_FAILED", "INVALID_SESSION_ID", "GUARDRAIL_TRIGGERED"]
    )

    message: str = Field(
        ...,
        description="Human-readable error message",
        examples=["The search service is currently unavailable."]
    )

    details: Optional[str] = Field(
        default=None,
        description="Additional error details for debugging (only in dev mode)",
        examples=["HTTPStatusError: 500 Internal Server Error"]
    )

    class Config:
        json_schema_extra = {
            "example": {
                "code": "RETRIEVAL_FAILED",
                "message": "The search service is currently unavailable.",
                "details": "HTTPStatusError: 500 Internal Server Error"
            }
        }
```

---

### ErrorResponse

**Purpose**: Standard error response wrapper.

```python
from pydantic import BaseModel, Field

class ErrorResponse(BaseModel):
    """Standard error response for all API endpoints."""

    error: ErrorDetail = Field(
        ...,
        description="Error details"
    )

    session_id: Optional[str] = Field(
        default=None,
        description="Session ID from request if available"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "error": {
                    "code": "RETRIEVAL_FAILED",
                    "message": "The search service is currently unavailable.",
                    "details": None
                },
                "session_id": "user_123_conv_456"
            }
        }
```

---

## Model Relationships

```
┌─────────────────────┐
│  ChatRequest        │
│  - message          │
│  - session_id       │
└──────────┬──────────┘
           │
           │ Optional extension
           ▼
┌─────────────────────────────┐
│  ChatRequestWithContext     │
│  - message                  │
│  - session_id               │
│  - context_text (optional)  │
└──────────┬──────────────────┘
           │
           │ Processed by agent
           ▼
┌─────────────────────┐        ┌────────────────┐
│  ChatResponse       │        │  Citation      │
│  - response         │        │  - source_num  │
│  - session_id       │        │  - url         │
│  - status           │        │  - score       │
└─────────────────────┘        └────────┬───────┘
           │                            │
           │ Alternative:               │
           │ Structured output          │ List[Citation]
           ▼                            │
┌─────────────────────────────────────┐ │
│  GroundedResponse                   │ │
│  - answer                           │ │
│  - sources  ────────────────────────┘
│  - confidence                       │
│  - retrieval_mode                   │
└─────────────────────────────────────┘
```

---

## Validation Examples

### Valid ChatRequest
```python
request = ChatRequest(
    message="What is the zero-moment point?",
    session_id="user_123_conv_456"
)
# ✅ Passes validation
```

### Invalid ChatRequest (empty message)
```python
request = ChatRequest(
    message="",  # ❌ min_length=1
    session_id="user_123_conv_456"
)
# Raises ValidationError: message must be at least 1 character
```

### Invalid ChatRequest (illegal session_id)
```python
request = ChatRequest(
    message="What is ZMP?",
    session_id="user@email.com"  # ❌ Contains @ symbol
)
# Raises ValidationError: session_id pattern mismatch
```

### Valid ChatRequestWithContext (full-book mode)
```python
request = ChatRequestWithContext(
    message="What is the zero-moment point?",
    session_id="user_123_conv_456",
    context_text=None  # ✅ Explicit None triggers full-book mode
)
```

### Valid ChatRequestWithContext (context-constrained mode)
```python
request = ChatRequestWithContext(
    message="Explain this concept",
    session_id="user_123_conv_456",
    context_text="The ZMP is the point on the ground..."  # ✅ Triggers context mode
)
```

### Invalid ChatRequestWithContext (empty context_text)
```python
request = ChatRequestWithContext(
    message="Explain this",
    session_id="user_123_conv_456",
    context_text="   "  # ❌ Whitespace-only string
)
# Raises ValidationError: context_text must be None or contain non-whitespace content
```

---

## Usage in FastAPI Endpoints

### Basic Endpoint with ChatRequest/ChatResponse

```python
from fastapi import FastAPI, HTTPException
from agents import Agent, Runner, SQLiteSession

app = FastAPI()

@app.post("/agent/chat", response_model=ChatResponse)
async def chat(request: ChatRequest) -> ChatResponse:
    """Chat with RAG agent about humanoid robotics.

    Args:
        request: User message and session ID

    Returns:
        Agent response with citations and sources

    Raises:
        HTTPException: 500 if agent execution fails
    """
    try:
        session = SQLiteSession(request.session_id, "conversations.db")

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

    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

### Extended Endpoint with Dual-Mode Support

```python
@app.post("/agent/chat", response_model=ChatResponse)
async def chat_with_context(request: ChatRequestWithContext) -> ChatResponse:
    """Chat with RAG agent with optional context-constrained mode.

    When context_text is provided, agent operates in context-constrained mode
    without retrieval. Otherwise, operates in full-book mode.

    Args:
        request: User message, session ID, and optional context text

    Returns:
        Agent response with citations (if full-book mode) or explanation (if context mode)
    """
    session = SQLiteSession(request.session_id, "conversations.db")

    # Modify agent instructions based on mode
    if request.context_text:
        # Context-constrained mode: prepend context to user message
        enhanced_message = (
            f"[CONTEXT-CONSTRAINED MODE]\n"
            f"Given the following text from the book:\n\n"
            f"{request.context_text}\n\n"
            f"User question: {request.message}"
        )
    else:
        # Full-book mode: normal message
        enhanced_message = request.message

    result = await Runner.run(agent, enhanced_message, session=session, max_turns=10)

    return ChatResponse(
        response=result.final_output,
        session_id=request.session_id,
        status="success"
    )
```

### Structured Output Endpoint with GroundedResponse

```python
# Configure agent with structured output
grounded_agent = Agent(
    name="RAG Assistant",
    instructions=GROUNDING_INSTRUCTIONS,
    tools=[retrieve_book_content],
    output_type=GroundedResponse  # Force structured output
)

@app.post("/agent/chat/structured", response_model=GroundedResponse)
async def chat_structured(request: ChatRequest) -> GroundedResponse:
    """Chat with RAG agent returning structured response with explicit citations."""
    session = SQLiteSession(request.session_id, "conversations.db")

    result = await Runner.run(grounded_agent, request.message, session=session)

    # Extract structured output
    response = result.final_output_as(GroundedResponse)

    return response
```

---

## Implementation Notes

1. **Model Location**: Define all models in `backend/models.py` for centralization
2. **Validation**: Rely on Pydantic for all validation; avoid manual checks in endpoints
3. **OpenAPI Schema**: Models automatically generate OpenAPI schema for `/docs`
4. **Serialization**: Use `.model_dump()` for dict conversion, `.model_dump_json()` for JSON
5. **Immutability**: Consider `frozen=True` for Citation and GroundedResponse (value objects)
6. **Type Hints**: Import from `typing` for backward compatibility (Python 3.9+)

---

**Models Defined**: 2025-12-15
**Next Step**: Create API contract specification (contracts/api-contract.yaml)
