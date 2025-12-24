"""
Pydantic models for user interest management API requests and responses.
"""

from pydantic import BaseModel, Field, field_validator
from typing import List, Optional
from datetime import datetime


class InterestSelectionRequest(BaseModel):
    """
    Request model for saving user interests.

    Attributes:
        user_id: User UUID from better-auth
        interest_ids: List of interest category IDs (2-5 interests required)
        background: User background level ('student' or 'professional')
    """
    user_id: str = Field(
        ...,
        description="User UUID from better-auth"
    )
    interest_ids: List[int] = Field(
        ...,
        min_length=2,
        max_length=5,
        description="List of interest category IDs (2-5 required)"
    )
    background: str = Field(
        ...,
        pattern="^(student|professional)$",
        description="User background level (student/professional)"
    )

    @field_validator('interest_ids')
    @classmethod
    def validate_unique_interests(cls, v: List[int]) -> List[int]:
        """Validate that all interest IDs are unique."""
        if len(v) != len(set(v)):
            raise ValueError("Duplicate interests are not allowed")
        return v


class InterestCategory(BaseModel):
    """
    Model representing an interest category.

    Attributes:
        id: Interest category ID
        name: Display name
        slug: URL-friendly slug
        description: Category description
    """
    id: int = Field(..., description="Interest category ID")
    name: str = Field(..., description="Display name")
    slug: str = Field(..., description="URL-friendly slug")
    description: Optional[str] = Field(None, description="Category description")


class InterestSelectionResponse(BaseModel):
    """
    Response model for interest selection/retrieval.

    Attributes:
        user_id: User UUID
        interests: List of selected interest categories
        background: User background level
        created_at: Timestamp when profile was created
        updated_at: Timestamp when profile was last updated
    """
    user_id: str = Field(..., description="User UUID")
    interests: List[InterestCategory] = Field(..., description="Selected interest categories")
    background: str = Field(..., description="User background level")
    created_at: Optional[datetime] = Field(None, description="Profile creation timestamp")
    updated_at: Optional[datetime] = Field(None, description="Profile update timestamp")


class InterestUpdateRequest(BaseModel):
    """
    Request model for updating user interests.

    Attributes:
        interest_ids: Updated list of interest category IDs (2-5 required)
    """
    interest_ids: List[int] = Field(
        ...,
        min_length=2,
        max_length=5,
        description="Updated list of interest category IDs (2-5 required)"
    )

    @field_validator('interest_ids')
    @classmethod
    def validate_unique_interests(cls, v: List[int]) -> List[int]:
        """Validate that all interest IDs are unique."""
        if len(v) != len(set(v)):
            raise ValueError("Duplicate interests are not allowed")
        return v


class InterestCategoriesResponse(BaseModel):
    """
    Response model for retrieving all available interest categories.

    Attributes:
        categories: List of all available interest categories
        total: Total count of categories
    """
    categories: List[InterestCategory] = Field(..., description="Available interest categories")
    total: int = Field(..., description="Total count of categories")


class InterestError(BaseModel):
    """
    Error response model for interest management failures.

    Attributes:
        error: Error type
        message: Human-readable error message
        details: Optional additional error details
    """
    error: str = Field(
        ...,
        description="Error type (e.g., 'invalid_interest_count', 'user_not_found')"
    )
    message: str = Field(
        ...,
        description="Human-readable error message"
    )
    details: Optional[str] = Field(
        default=None,
        description="Additional error details"
    )
