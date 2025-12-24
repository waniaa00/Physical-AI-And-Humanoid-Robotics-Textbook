"""
Pydantic models for translation API requests and responses.
"""

from pydantic import BaseModel, Field, field_validator
from typing import Optional, Dict


class TranslationRequest(BaseModel):
    """
    Request model for text translation.

    Attributes:
        text: Text to translate (1-10,000 characters, max 5,000 words)
        target_lang: Target language code ('ur' for Urdu, 'en' for English)
        preserve_code: Whether to preserve code blocks during translation
        user_id: Optional user ID for logging and analytics
    """
    text: str = Field(
        ...,
        min_length=1,
        max_length=10000,
        description="Text to translate"
    )
    target_lang: str = Field(
        default="ur",
        pattern="^(ur|en)$",
        description="Target language code (ur=Urdu, en=English)"
    )
    preserve_code: bool = Field(
        default=True,
        description="Preserve code blocks and technical terms during translation"
    )
    user_id: Optional[str] = Field(
        default=None,
        description="Optional user ID for logging"
    )

    @field_validator('text')
    @classmethod
    def validate_text_length(cls, v: str) -> str:
        """Validate text does not exceed 5,000 words."""
        word_count = len(v.split())
        if word_count > 5000:
            raise ValueError("Text exceeds maximum 5,000 words")
        return v


class TranslationResponse(BaseModel):
    """
    Response model for text translation.

    Attributes:
        translated_text: Translated text with preserved formatting
        original_text: Original input text
        detected_lang: Detected source language code
        target_lang: Target language code
        word_count: Word count of original text
        translation_time_ms: Translation processing time in milliseconds
        preserved_elements: Count of preserved code blocks and technical terms
    """
    translated_text: str = Field(
        ...,
        description="Translated text with preserved formatting"
    )
    original_text: str = Field(
        ...,
        description="Original input text"
    )
    detected_lang: str = Field(
        ...,
        description="Detected source language code"
    )
    target_lang: str = Field(
        ...,
        description="Target language code"
    )
    word_count: int = Field(
        ...,
        description="Word count of original text"
    )
    translation_time_ms: int = Field(
        ...,
        description="Translation processing time in milliseconds"
    )
    preserved_elements: Dict[str, int] = Field(
        default_factory=dict,
        description="Count of preserved code blocks and technical terms"
    )


class TranslationError(BaseModel):
    """
    Error response model for translation failures.

    Attributes:
        error: Error type
        message: Human-readable error message
        details: Optional additional error details
    """
    error: str = Field(
        ...,
        description="Error type (e.g., 'translation_failed', 'invalid_input')"
    )
    message: str = Field(
        ...,
        description="Human-readable error message"
    )
    details: Optional[str] = Field(
        default=None,
        description="Additional error details"
    )
