"""
Pydantic models package for API requests and responses.
"""

from .chat import (
    ChatRequest,
    ChatRequestWithContext,
    ChatResponse,
    Citation,
    GroundedResponse,
    ConversationMessage,
    ErrorDetail,
    ErrorResponse,
)
from .translation import (
    TranslationRequest,
    TranslationResponse,
    TranslationError,
)
from .summarization import (
    SummarizationRequest,
    SummarizationResponse,
    SummarizationError,
    SummaryLength,
    SummaryFocus,
)
from .interests import (
    InterestSelectionRequest,
    InterestSelectionResponse,
    InterestUpdateRequest,
    InterestCategory,
    InterestCategoriesResponse,
    InterestError,
)

__all__ = [
    # Chat
    "ChatRequest",
    "ChatRequestWithContext",
    "ChatResponse",
    "Citation",
    "GroundedResponse",
    "ConversationMessage",
    "ErrorDetail",
    "ErrorResponse",
    # Translation
    "TranslationRequest",
    "TranslationResponse",
    "TranslationError",
    # Summarization
    "SummarizationRequest",
    "SummarizationResponse",
    "SummarizationError",
    "SummaryLength",
    "SummaryFocus",
    # Interests
    "InterestSelectionRequest",
    "InterestSelectionResponse",
    "InterestUpdateRequest",
    "InterestCategory",
    "InterestCategoriesResponse",
    "InterestError",
]
