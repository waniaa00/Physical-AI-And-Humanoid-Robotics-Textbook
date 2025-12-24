"""
Pydantic models for text summarization API requests and responses.
"""

from pydantic import BaseModel, Field, field_validator
from typing import Optional, List
from enum import Enum


class SummaryLength(str, Enum):
    """Enumeration of summary length options."""
    SHORT = "short"      # ~20% of original
    MEDIUM = "medium"    # ~30% of original
    LONG = "long"        # ~40% of original


class SummaryFocus(str, Enum):
    """Enumeration of summary focus areas."""
    CONCEPTS = "concepts"  # Focus on conceptual explanations
    CODE = "code"          # Include code snippets in summary
    BOTH = "both"          # Balance concepts and code


class SummarizationRequest(BaseModel):
    """
    Request model for text summarization.

    Attributes:
        text: Text to summarize (50-10,000 characters, 50-5,000 words)
        target_length: Desired summary length (short/medium/long)
        focus: Summary focus area (concepts/code/both)
        user_id: Optional user ID for logging
    """
    text: str = Field(
        ...,
        min_length=50,
        max_length=10000,
        description="Text to summarize"
    )
    target_length: SummaryLength = Field(
        default=SummaryLength.SHORT,
        description="Desired summary length (short=20%, medium=30%, long=40%)"
    )
    focus: SummaryFocus = Field(
        default=SummaryFocus.CONCEPTS,
        description="Summary focus area (concepts/code/both)"
    )
    user_id: Optional[str] = Field(
        default=None,
        description="Optional user ID for logging"
    )

    @field_validator('text')
    @classmethod
    def validate_text_length(cls, v: str) -> str:
        """Validate text is within acceptable word count range."""
        word_count = len(v.split())
        if word_count < 50:
            raise ValueError("Text is too short to summarize (minimum 50 words)")
        if word_count > 5000:
            raise ValueError("Text exceeds maximum 5,000 words")
        return v


class SummarizationResponse(BaseModel):
    """
    Response model for text summarization.

    Attributes:
        summary: Generated summary text
        original_word_count: Word count of original text
        summary_word_count: Word count of summary
        compression_ratio: Ratio of summary length to original (0-1)
        processing_time_ms: Summarization processing time in milliseconds
        key_points: Extracted key points (optional)
    """
    summary: str = Field(
        ...,
        description="Generated summary text"
    )
    original_word_count: int = Field(
        ...,
        description="Word count of original text"
    )
    summary_word_count: int = Field(
        ...,
        description="Word count of summary"
    )
    compression_ratio: float = Field(
        ...,
        description="Ratio of summary length to original (0-1)"
    )
    processing_time_ms: int = Field(
        ...,
        description="Summarization processing time in milliseconds"
    )
    key_points: Optional[List[str]] = Field(
        default=None,
        description="Extracted key points (optional)"
    )


class SummarizationError(BaseModel):
    """
    Error response model for summarization failures.

    Attributes:
        error: Error type
        message: Human-readable error message
        details: Optional additional error details
    """
    error: str = Field(
        ...,
        description="Error type (e.g., 'summarization_failed', 'text_too_short')"
    )
    message: str = Field(
        ...,
        description="Human-readable error message"
    )
    details: Optional[str] = Field(
        default=None,
        description="Additional error details"
    )
