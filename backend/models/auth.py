"""
Authentication models for Better Auth Integration.

This module defines Pydantic models for:
- User entity (database model representation)
- Session entity (authentication session)
- AuthToken (JWT response model)
- Sign-up and sign-in request/response models
"""

from pydantic import BaseModel, EmailStr, Field, field_validator
from typing import Optional, List
from datetime import datetime
from enum import Enum
import uuid


class AccountStatus(str, Enum):
    """User account status enumeration."""
    ACTIVE = "active"
    SUSPENDED = "suspended"
    DELETED = "deleted"


class User(BaseModel):
    """
    User entity model (T005).

    Represents an authenticated user account with email, hashed password,
    and account metadata.
    """
    id: uuid.UUID = Field(default_factory=uuid.uuid4)
    email: EmailStr
    password_hash: str
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    last_sign_in_at: Optional[datetime] = None
    account_status: AccountStatus = AccountStatus.ACTIVE

    class Config:
        from_attributes = True


class Session(BaseModel):
    """
    Session entity model (T006).

    Represents an active authentication session with token, expiration,
    and security metadata.
    """
    id: uuid.UUID = Field(default_factory=uuid.uuid4)
    user_id: uuid.UUID
    token_hash: str  # Hashed JWT for revocation lookup
    expires_at: datetime
    created_at: datetime = Field(default_factory=datetime.utcnow)
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None

    class Config:
        from_attributes = True


class AuthToken(BaseModel):
    """
    AuthToken Pydantic model (T007).

    Response model for JWT tokens returned to clients.
    """
    access_token: str
    token_type: str = "bearer"
    expires_in: int  # Seconds until expiration

    class Config:
        from_attributes = True


# Sign-Up Models (T016-T017)

class SignUpRequest(BaseModel):
    """
    Sign-up request model (T016).

    Used for new user registration with interest selection.
    """
    email: EmailStr
    password: str = Field(..., min_length=8, max_length=128)
    interests: List[int] = Field(..., min_length=2, max_length=5)
    background: str = Field(..., pattern="^(student|professional)$")
    language_preference: str = Field(default="en", pattern="^(en|ur)$")

    @field_validator('interests')
    @classmethod
    def validate_interest_count(cls, v):
        """Ensure 2-5 interests selected."""
        if not (2 <= len(v) <= 5):
            raise ValueError('Must select between 2 and 5 interests')
        if len(v) != len(set(v)):
            raise ValueError('Interest IDs must be unique')
        return v


class SignUpResponse(BaseModel):
    """
    Sign-up response model (T017).

    Response after successful user registration.
    """
    user_id: uuid.UUID
    session_token: str
    message: str = "Account created successfully"

    class Config:
        from_attributes = True


# Sign-In Models (T033-T034)

class SignInRequest(BaseModel):
    """
    Sign-in request model (T033).

    Used for returning user authentication.
    """
    email: EmailStr
    password: str


class SignInResponse(BaseModel):
    """
    Sign-in response model (T034).

    Response after successful authentication with user interests.
    """
    user_id: uuid.UUID
    session_token: str
    interests: List[int] = []
    background: Optional[str] = None
    language_preference: Optional[str] = None
    message: str = "Sign-in successful"

    class Config:
        from_attributes = True


# Interest Management Models (T066-T067)

class UpdateInterestsRequest(BaseModel):
    """
    Update interests request model (T066).

    Used for authenticated users to update their interest selections.
    """
    interests: List[int] = Field(..., min_length=2, max_length=5)

    @field_validator('interests')
    @classmethod
    def validate_interest_count(cls, v):
        """Ensure 2-5 interests selected."""
        if not (2 <= len(v) <= 5):
            raise ValueError('Must select between 2 and 5 interests')
        if len(v) != len(set(v)):
            raise ValueError('Interest IDs must be unique')
        return v


class UpdateInterestsResponse(BaseModel):
    """
    Update interests response model (T067).

    Response after successful interest update.
    """
    message: str = "Interests updated successfully"
    interests: List[int]

    class Config:
        from_attributes = True


# Error Response Models (T086)

class ErrorDetail(BaseModel):
    """Detailed error information."""
    error_code: str
    message: str
    details: Optional[dict] = None


class ErrorResponse(BaseModel):
    """Comprehensive error response model."""
    error: ErrorDetail
