"""
Utility modules for backend services.
"""

from .errors import (
    TranslationError,
    SummarizationError,
    PersonalizationError,
    DatabaseError,
)
from .text_processing import (
    detect_code_blocks,
    preserve_code_blocks,
    restore_code_blocks,
    count_words,
)

__all__ = [
    # Errors
    "TranslationError",
    "SummarizationError",
    "PersonalizationError",
    "DatabaseError",
    # Text processing
    "detect_code_blocks",
    "preserve_code_blocks",
    "restore_code_blocks",
    "count_words",
]
