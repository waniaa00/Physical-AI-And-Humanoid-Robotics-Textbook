"""
Custom exception classes for backend services.

Defines domain-specific exceptions for translation, summarization,
personalization, and database operations.
"""

from typing import Optional


class TranslationError(Exception):
    """
    Exception raised for translation-related errors.

    Attributes:
        message: Human-readable error message
        error_code: Optional error code for categorization
        original_exception: Original exception if wrapped
    """

    def __init__(
        self,
        message: str,
        error_code: Optional[str] = None,
        original_exception: Optional[Exception] = None
    ):
        self.message = message
        self.error_code = error_code or "TRANSLATION_ERROR"
        self.original_exception = original_exception
        super().__init__(self.message)

    def __str__(self) -> str:
        if self.original_exception:
            return f"{self.error_code}: {self.message} (caused by: {str(self.original_exception)})"
        return f"{self.error_code}: {self.message}"


class SummarizationError(Exception):
    """
    Exception raised for summarization-related errors.

    Attributes:
        message: Human-readable error message
        error_code: Optional error code for categorization
        original_exception: Original exception if wrapped
    """

    def __init__(
        self,
        message: str,
        error_code: Optional[str] = None,
        original_exception: Optional[Exception] = None
    ):
        self.message = message
        self.error_code = error_code or "SUMMARIZATION_ERROR"
        self.original_exception = original_exception
        super().__init__(self.message)

    def __str__(self) -> str:
        if self.original_exception:
            return f"{self.error_code}: {self.message} (caused by: {str(self.original_exception)})"
        return f"{self.error_code}: {self.message}"


class PersonalizationError(Exception):
    """
    Exception raised for personalization-related errors.

    Attributes:
        message: Human-readable error message
        error_code: Optional error code for categorization
        original_exception: Original exception if wrapped
    """

    def __init__(
        self,
        message: str,
        error_code: Optional[str] = None,
        original_exception: Optional[Exception] = None
    ):
        self.message = message
        self.error_code = error_code or "PERSONALIZATION_ERROR"
        self.original_exception = original_exception
        super().__init__(self.message)

    def __str__(self) -> str:
        if self.original_exception:
            return f"{self.error_code}: {self.message} (caused by: {str(self.original_exception)})"
        return f"{self.error_code}: {self.message}"


class DatabaseError(Exception):
    """
    Exception raised for database-related errors.

    Attributes:
        message: Human-readable error message
        error_code: Optional error code for categorization
        original_exception: Original exception if wrapped
    """

    def __init__(
        self,
        message: str,
        error_code: Optional[str] = None,
        original_exception: Optional[Exception] = None
    ):
        self.message = message
        self.error_code = error_code or "DATABASE_ERROR"
        self.original_exception = original_exception
        super().__init__(self.message)

    def __str__(self) -> str:
        if self.original_exception:
            return f"{self.error_code}: {self.message} (caused by: {str(self.original_exception)})"
        return f"{self.error_code}: {self.message}"


class ValidationError(Exception):
    """
    Exception raised for input validation errors.

    Attributes:
        message: Human-readable error message
        field: Field name that failed validation
        error_code: Optional error code for categorization
    """

    def __init__(
        self,
        message: str,
        field: Optional[str] = None,
        error_code: Optional[str] = None
    ):
        self.message = message
        self.field = field
        self.error_code = error_code or "VALIDATION_ERROR"
        super().__init__(self.message)

    def __str__(self) -> str:
        if self.field:
            return f"{self.error_code}: {self.field} - {self.message}"
        return f"{self.error_code}: {self.message}"


class RateLimitError(Exception):
    """
    Exception raised when rate limits are exceeded.

    Attributes:
        message: Human-readable error message
        retry_after: Seconds to wait before retrying
    """

    def __init__(
        self,
        message: str = "Rate limit exceeded",
        retry_after: Optional[int] = None
    ):
        self.message = message
        self.retry_after = retry_after
        super().__init__(self.message)

    def __str__(self) -> str:
        if self.retry_after:
            return f"RATE_LIMIT_ERROR: {self.message} (retry after {self.retry_after}s)"
        return f"RATE_LIMIT_ERROR: {self.message}"
