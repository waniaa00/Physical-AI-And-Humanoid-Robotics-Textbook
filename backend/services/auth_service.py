"""
Authentication service for Better Auth Integration (T011).

This service provides:
- User sign-up with email validation and password strength checks (T018, T020, T021)
- User sign-in with credential verification (T035)
- Session management
- Interest validation (T022)
- Audit logging (T025, T040)
"""

from typing import Optional, List, Tuple
import uuid
import re
import logging
from datetime import datetime
from repositories.user_repository import UserRepository
from utils.password_hasher import hash_password, verify_password
from utils.token_manager import create_access_token
from models.auth import User, SignUpRequest, SignUpResponse, SignInRequest, SignInResponse, AccountStatus

logger = logging.getLogger(__name__)


class AuthService:
    """
    Authentication service handling user registration, sign-in, and validation.
    """

    def __init__(self):
        """Initialize the authentication service."""
        self.user_repo = UserRepository()

    async def sign_up(
        self,
        request: SignUpRequest,
        ip_address: Optional[str] = None,
        user_agent: Optional[str] = None
    ) -> SignUpResponse:
        """
        Register a new user with email, password, and interests (T018).

        Performs:
        - Email format validation (T020)
        - Password strength validation (T021)
        - Interest count validation (T022)
        - Duplicate email check (T024)
        - Password hashing
        - User creation
        - Interest association (T023)
        - Audit logging (T025)

        Args:
            request: SignUpRequest with email, password, interests, background, language_preference
            ip_address: Client IP address for audit logging
            user_agent: Client user agent for audit logging

        Returns:
            SignUpResponse: User ID and session token

        Raises:
            ValueError: If validation fails
            Exception: If email already exists or database error
        """
        # T020: Email format validation (RFC 5322 - handled by EmailStr in Pydantic)
        email = request.email.lower()

        # T021: Password strength validation
        self._validate_password_strength(request.password)

        # T022: Interest count validation (2-5 interests - handled by Pydantic validator)
        # Additional validation: ensure interest IDs exist
        await self._validate_interest_ids(request.interests)

        # T024: Check for duplicate email
        existing_user = await self.user_repo.get_user_by_email(email)
        if existing_user is not None:
            logger.warning(f"Sign-up attempt with existing email: {email}")
            raise ValueError("Email already registered")

        # Hash password using bcrypt cost 12
        password_hash = hash_password(request.password)

        # Create user account
        try:
            user = await self.user_repo.create_user(
                email=email,
                password_hash=password_hash,
                account_status=AccountStatus.ACTIVE
            )

            # Create user profile with background and language preference
            await self.user_repo.create_user_profile(
                user_id=user.id,
                background=request.background,
                language_preference=request.language_preference
            )

            # T023: Save user interests
            await self.user_repo.save_user_interests(user.id, request.interests)

            # Create session token
            session_token, expires_at = create_access_token(user.id, user.email)

            # T025: Audit logging for sign-up
            logger.info(
                f"User sign-up successful | user_id={user.id} | email={email} | "
                f"ip={ip_address} | user_agent={user_agent} | interests={request.interests}"
            )

            return SignUpResponse(
                user_id=user.id,
                session_token=session_token,
                message="Account created successfully"
            )

        except ValueError as e:
            # Re-raise validation errors
            raise
        except Exception as e:
            # T025: Audit logging for failed sign-up
            logger.error(
                f"User sign-up failed | email={email} | error={str(e)} | "
                f"ip={ip_address} | user_agent={user_agent}"
            )
            raise

    async def sign_in(
        self,
        request: SignInRequest,
        ip_address: Optional[str] = None,
        user_agent: Optional[str] = None
    ) -> SignInResponse:
        """
        Authenticate user with email and password (T035).

        Performs:
        - User lookup by email
        - Password verification with bcrypt
        - Last sign-in timestamp update (T038)
        - User interests retrieval (T039)
        - Generic error handling (T037)
        - Audit logging (T040)

        Args:
            request: SignInRequest with email and password
            ip_address: Client IP address for audit logging
            user_agent: Client user agent for audit logging

        Returns:
            SignInResponse: User ID, session token, and interests

        Raises:
            ValueError: If credentials are invalid (generic error message)
        """
        email = request.email.lower()

        # Retrieve user by email
        user = await self.user_repo.get_user_by_email(email)

        # T037: Generic error message to prevent email enumeration
        if user is None:
            logger.warning(f"Sign-in attempt with non-existent email: {email} | ip={ip_address}")
            raise ValueError("Invalid credentials")

        # Verify password using bcrypt
        if not verify_password(request.password, user.password_hash):
            # T040: Audit logging for failed sign-in
            logger.warning(
                f"Sign-in failed - invalid password | user_id={user.id} | email={email} | "
                f"ip={ip_address} | user_agent={user_agent}"
            )
            raise ValueError("Invalid credentials")

        # Check account status
        if user.account_status != AccountStatus.ACTIVE:
            logger.warning(f"Sign-in attempt with suspended/deleted account: {user.id}")
            raise ValueError("Account is not active")

        # T038: Update last sign-in timestamp
        await self.user_repo.update_last_sign_in(user.id)

        # T039: Retrieve user interests
        interests = await self.user_repo.get_user_interests(user.id)

        # Retrieve user profile information
        user_profile = await self.user_repo.get_user_profile(user.id)

        # Create session token
        session_token, expires_at = create_access_token(user.id, user.email)

        # T040: Audit logging for successful sign-in
        logger.info(
            f"User sign-in successful | user_id={user.id} | email={email} | "
            f"ip={ip_address} | user_agent={user_agent}"
        )

        # Prepare profile information
        background = None
        language_preference = None
        if user_profile:
            background = user_profile.get('background')
            language_preference = user_profile.get('language_preference')

        return SignInResponse(
            user_id=user.id,
            session_token=session_token,
            interests=interests,
            background=background,
            language_preference=language_preference,
            message="Sign-in successful"
        )

    async def validate_session(self, token: str) -> Tuple[bool, Optional[uuid.UUID]]:
        """
        Validate JWT session token (T047, T048).

        Args:
            token: JWT access token

        Returns:
            Tuple[bool, Optional[UUID]]: (is_valid, user_id)
        """
        from utils.token_manager import verify_token, get_user_id_from_token

        # Verify token signature and expiration
        if not verify_token(token):
            return False, None

        # Extract user ID
        user_id = get_user_id_from_token(token)
        if user_id is None:
            return False, None

        # Verify user still exists and is active
        user = await self.user_repo.get_user_by_id(user_id)
        if user is None or user.account_status != AccountStatus.ACTIVE:
            return False, None

        return True, user_id

    def _validate_password_strength(self, password: str) -> None:
        """
        Validate password strength (T021).

        Requirements:
        - Minimum 8 characters
        - At least one uppercase letter
        - At least one lowercase letter
        - At least one number

        Args:
            password: Plain text password

        Raises:
            ValueError: If password does not meet strength requirements
        """
        if len(password) < 8:
            raise ValueError("Password must be at least 8 characters long")

        if not re.search(r'[A-Z]', password):
            raise ValueError("Password must contain at least one uppercase letter")

        if not re.search(r'[a-z]', password):
            raise ValueError("Password must contain at least one lowercase letter")

        if not re.search(r'[0-9]', password):
            raise ValueError("Password must contain at least one number")

    async def _validate_interest_ids(self, interest_ids: List[int]) -> None:
        """
        Validate that interest IDs exist in the database (T022).

        Args:
            interest_ids: List of interest category IDs

        Raises:
            ValueError: If any interest ID is invalid
        """
        # Query to check if all interest IDs exist
        # Assuming interest_categories table exists from Phase 5
        query = """
            SELECT COUNT(*) as count
            FROM interest_categories
            WHERE id = ANY($1::int[])
        """
        try:
            count = await self.user_repo.fetch_val(query, interest_ids)
            if count != len(interest_ids):
                raise ValueError("One or more interest IDs are invalid")
        except Exception as e:
            logger.error(f"Error validating interest IDs: {e}")
            raise ValueError("Failed to validate interests")
