"""
Authentication middleware for FastAPI (T012).

Provides:
- Optional authentication (get_optional_user) - for backward compatibility with guest users
- Required authentication (get_required_user) - for protected endpoints
- User context injection (request.state.user_id)
- Token tampering detection (T053)
"""

from typing import Optional
from fastapi import Request, Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
import uuid
import logging

logger = logging.getLogger(__name__)

# Security scheme for extracting Bearer token
security = HTTPBearer(auto_error=False)


class UserContext:
    """
    User context object injected into request.state.

    Contains authenticated user information for downstream handlers.
    """
    def __init__(self, user_id: uuid.UUID, email: str):
        self.user_id = user_id
        self.email = email


async def get_optional_user(
    request: Request,
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
) -> Optional[UserContext]:
    """
    Optional authentication dependency (T012, T057).

    Validates JWT if Authorization header present, otherwise allows guest access.
    Used for endpoints that support both authenticated and guest users.

    Args:
        request: FastAPI request object
        credentials: HTTP Bearer credentials (optional)

    Returns:
        Optional[UserContext]: User context if authenticated, None for guest users

    Side Effects:
        Sets request.state.user_id for downstream handlers (T060)
    """
    # Try to get token from Authorization header first, then from cookie
    token = None
    if credentials is not None:
        token = credentials.credentials
    else:
        # Check for session_token cookie
        token = request.cookies.get("session_token")

    # No token = guest user
    if token is None:
        request.state.user_id = None
        return None

    # Validate token
    from services.auth_service import AuthService
    auth_service = AuthService()

    try:
        is_valid, user_id = await auth_service.validate_session(token)

        if not is_valid or user_id is None:
            # T053: Token tampering detected
            logger.warning(f"Invalid or tampered token detected | ip={request.client.host}")
            request.state.user_id = None
            return None

        # Get user details
        user = await auth_service.user_repo.get_user_by_id(user_id)
        if user is None:
            request.state.user_id = None
            return None

        # Inject user context
        user_context = UserContext(user_id=user.id, email=user.email)
        request.state.user_id = user.id  # T060: Inject user_id into request state

        return user_context

    except Exception as e:
        logger.error(f"Error in optional authentication: {e}")
        request.state.user_id = None
        return None


async def get_required_user(
    request: Request,
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security)
) -> UserContext:
    """
    Required authentication dependency (T012, T058, T059).

    Validates JWT and returns user context. Returns 401 if not authenticated.
    Used for endpoints that require authentication (protected routes).

    Args:
        request: FastAPI request object
        credentials: HTTP Bearer credentials (required)

    Returns:
        UserContext: Authenticated user context

    Raises:
        HTTPException: 401 Unauthorized if token is missing or invalid

    Side Effects:
        Sets request.state.user_id for downstream handlers (T060)
    """
    # Try to get token from Authorization header first, then from cookie
    token = None
    if credentials is not None:
        token = credentials.credentials
    else:
        # Check for session_token cookie
        token = request.cookies.get("session_token")

    # No token found = unauthorized
    if token is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication required",
            headers={"WWW-Authenticate": "Bearer"},
        )

    # Validate token
    from services.auth_service import AuthService
    auth_service = AuthService()

    try:
        is_valid, user_id = await auth_service.validate_session(token)

        if not is_valid or user_id is None:
            # T053: Token tampering detected - return 401
            logger.warning(f"Invalid or tampered token in protected endpoint | ip={request.client.host}")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid or expired token",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Get user details
        user = await auth_service.user_repo.get_user_by_id(user_id)
        if user is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found",
                headers={"WWW-Authenticate": "Bearer"},
            )

        # Inject user context
        user_context = UserContext(user_id=user.id, email=user.email)
        request.state.user_id = user.id  # T060: Inject user_id into request state

        logger.info(f"Authenticated request | user_id={user.id} | path={request.url.path}")

        return user_context

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(f"Error in required authentication: {e}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Authentication failed",
            headers={"WWW-Authenticate": "Bearer"},
        )
