"""
JWT token manager for authentication (T010).

This module provides JWT token creation, verification, and decoding
with 7-day expiration as per specification.
"""

from datetime import datetime, timedelta
from typing import Optional, Dict, Any
import os
import uuid
import logging
from jose import JWTError, jwt

logger = logging.getLogger(__name__)

# JWT configuration from environment variables
JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY", "default-secret-key-change-in-production")
JWT_ALGORITHM = os.getenv("JWT_ALGORITHM", "HS256")
JWT_EXPIRATION_DAYS = int(os.getenv("JWT_EXPIRATION_DAYS", "7"))


def create_access_token(user_id: uuid.UUID, email: str) -> tuple[str, datetime]:
    """
    Create JWT access token with 7-day expiry (T010).

    Args:
        user_id: User's unique identifier
        email: User's email address

    Returns:
        tuple[str, datetime]: (JWT token string, expiration datetime)
    """
    # Calculate expiration time (7 days from now)
    expires_delta = timedelta(days=JWT_EXPIRATION_DAYS)
    expires_at = datetime.utcnow() + expires_delta

    # JWT payload (claims)
    payload = {
        "sub": str(user_id),  # Subject (user ID)
        "email": email,
        "exp": expires_at,  # Expiration time
        "iat": datetime.utcnow(),  # Issued at
        "type": "access"  # Token type
    }

    # Encode JWT
    try:
        token = jwt.encode(payload, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)
        logger.info(f"Created access token for user {user_id}, expires at {expires_at}")
        return token, expires_at
    except Exception as e:
        logger.error(f"Error creating access token: {e}")
        raise


def verify_token(token: str) -> bool:
    """
    Verify JWT signature and expiration (T010, T047).

    Args:
        token: JWT token string

    Returns:
        bool: True if token is valid and not expired, False otherwise
    """
    try:
        # Decode and verify token signature + expiration
        jwt.decode(token, JWT_SECRET_KEY, algorithms=[JWT_ALGORITHM])
        return True
    except JWTError as e:
        logger.warning(f"Token verification failed: {e}")
        return False


def decode_token(token: str) -> Optional[Dict[str, Any]]:
    """
    Decode JWT and extract payload (T010).

    Args:
        token: JWT token string

    Returns:
        Optional[Dict]: Token payload if valid, None if invalid or expired
    """
    try:
        # Decode token and verify signature + expiration
        payload = jwt.decode(token, JWT_SECRET_KEY, algorithms=[JWT_ALGORITHM])
        return payload
    except JWTError as e:
        logger.warning(f"Token decode failed: {e}")
        return None


def get_user_id_from_token(token: str) -> Optional[uuid.UUID]:
    """
    Extract user ID from JWT token.

    Args:
        token: JWT token string

    Returns:
        Optional[UUID]: User ID if token is valid, None otherwise
    """
    payload = decode_token(token)
    if payload is None:
        return None

    try:
        user_id = uuid.UUID(payload.get("sub"))
        return user_id
    except (ValueError, TypeError) as e:
        logger.error(f"Invalid user ID in token: {e}")
        return None


def get_token_expiration(token: str) -> Optional[datetime]:
    """
    Get expiration datetime from JWT token.

    Args:
        token: JWT token string

    Returns:
        Optional[datetime]: Expiration datetime if token is valid, None otherwise
    """
    payload = decode_token(token)
    if payload is None:
        return None

    try:
        exp_timestamp = payload.get("exp")
        return datetime.fromtimestamp(exp_timestamp)
    except (ValueError, TypeError) as e:
        logger.error(f"Invalid expiration in token: {e}")
        return None
