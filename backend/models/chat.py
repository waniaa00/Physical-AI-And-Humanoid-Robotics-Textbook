"""
Pydantic models for OpenAI Agent chat API
Defines request/response schemas with validation
"""
from pydantic import BaseModel, Field, field_validator
from typing import Optional, List, Dict
from datetime import datetime
import re


# Chat Models (T016-T017)

class ChatRequest(BaseModel):
    """Basic chat request with message and session ID"""
    message: str = Field(..., min_length=1, max_length=2000, description="User message")
    session_id: str = Field(..., description="Conversation session identifier")

    @field_validator('session_id')
    @classmethod
    def validate_session_id(cls, v):
        """Validate session_id format (alphanumeric, hyphens, underscores)"""
        if not re.match(r'^[a-zA-Z0-9_-]+$', v):
            raise ValueError("session_id must contain only alphanumeric characters, hyphens, and underscores")
        if len(v) < 3 or len(v) > 100:
            raise ValueError("session_id must be between 3 and 100 characters")
        return v


class ChatRequestWithContext(ChatRequest):
    """Chat request with optional selected text context and user personalization"""
    context_text: Optional[str] = Field(None, description="Selected text to explain (context-constrained mode)")
    user_id: Optional[str] = Field(None, description="User ID for personalization (UUID format)")

    @field_validator('context_text')
    @classmethod
    def validate_context_text(cls, v):
        """Ensure context_text is non-empty if provided"""
        if v is not None and not v.strip():
            raise ValueError("context_text cannot be empty or whitespace only")
        return v.strip() if v else None

    @field_validator('user_id')
    @classmethod
    def validate_user_id(cls, v):
        """Validate user_id is a valid UUID format if provided"""
        if v is not None:
            # Basic UUID format validation (8-4-4-4-12 hex digits)
            import re
            if not re.match(r'^[0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-[0-9a-fA-F]{12}$', v):
                raise ValueError("user_id must be a valid UUID format")
        return v


# Citation Models (T018)

class Citation(BaseModel):
    """Source citation with URL and relevance score"""
    source_number: int = Field(..., ge=1, description="Source number from [Source N] marker")
    url: str = Field(..., description="Source URL")
    score: float = Field(..., ge=0.0, le=1.0, description="Relevance score (0.0-1.0)")

    model_config = {
        "json_schema_extra": {
            "example": {
                "source_number": 1,
                "url": "https://example.com/docs/chapter1",
                "score": 0.87
            }
        }
    }


# Response Models (T019-T020)

class ChatResponse(BaseModel):
    """Basic chat response"""
    response: str = Field(..., description="Agent response text")
    session_id: str = Field(..., description="Conversation session identifier")
    status: str = Field("success", description="Response status (success/error)")


class GroundedResponse(BaseModel):
    """Response with citations and confidence"""
    answer: str = Field(..., description="Grounded answer text")
    sources: List[Citation] = Field(default_factory=list, description="List of source citations")
    confidence: str = Field(..., description="Confidence level (high/medium/low)")
    retrieval_mode: str = Field(..., description="Mode used: full-book or context-constrained")
    session_id: str = Field(..., description="Session identifier")

    model_config = {
        "json_schema_extra": {
            "example": {
                "answer": "Inverse kinematics is the process of determining joint angles...",
                "sources": [
                    {
                        "source_number": 1,
                        "url": "https://example.com/docs/chapter1",
                        "score": 0.87
                    }
                ],
                "confidence": "high",
                "retrieval_mode": "full-book",
                "session_id": "user_123"
            }
        }
    }


# Conversation History Models (T021)

class ConversationMessage(BaseModel):
    """Single message in conversation history"""
    role: str = Field(..., description="Message role: user, assistant, or tool")
    content: str = Field(..., description="Message content")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Message timestamp")
    tool_call_id: Optional[str] = Field(None, description="Tool call identifier if role=tool")

    model_config = {
        "json_schema_extra": {
            "example": {
                "role": "user",
                "content": "What is ZMP?",
                "timestamp": "2025-12-17T10:30:00Z",
                "tool_call_id": None
            }
        }
    }


# Error Models (T022)

class ErrorDetail(BaseModel):
    """Detailed error information"""
    error_type: str = Field(..., description="Error type classification")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[Dict] = Field(None, description="Additional error context")


class ErrorResponse(BaseModel):
    """Error response wrapper"""
    status: str = Field("error", description="Response status")
    error: ErrorDetail = Field(..., description="Error details")
    session_id: Optional[str] = Field(None, description="Session identifier if available")

    model_config = {
        "json_schema_extra": {
            "example": {
                "status": "error",
                "error": {
                    "error_type": "retrieval_failure",
                    "message": "Failed to retrieve book content",
                    "details": {"cause": "Search service unavailable"}
                },
                "session_id": "user_123"
            }
        }
    }
