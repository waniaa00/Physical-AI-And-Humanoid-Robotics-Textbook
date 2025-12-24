"""
User repository for authentication operations (T008).

This module provides database operations for user accounts including:
- Creating new users
- Retrieving users by email or ID
- Updating last sign-in timestamp
- Managing user interests (T023)
- Updating user interests (T068)
"""

from typing import Optional, List, Dict, Any
from datetime import datetime
import uuid
import logging
from repositories.base_repository import BaseRepository
from models.auth import User, AccountStatus

logger = logging.getLogger(__name__)


class UserRepository(BaseRepository):
    """
    User repository implementing database operations for user accounts.

    Methods:
        create_user: Create a new user account
        get_user_by_email: Retrieve user by email address
        get_user_by_id: Retrieve user by UUID
        update_last_sign_in: Update last sign-in timestamp
        save_user_interests: Associate interests with user (T023)
        get_user_interests: Retrieve user's interest IDs (T039)
        update_user_interests: Update user's interest selections (T068)
        invalidate_all_sessions: Invalidate all user sessions (T052)
    """

    async def create_user(
        self,
        email: str,
        password_hash: str,
        account_status: AccountStatus = AccountStatus.ACTIVE
    ) -> User:
        """
        Create a new user account.

        Args:
            email: User's email address (unique)
            password_hash: bcrypt hashed password
            account_status: Account status (default: active)

        Returns:
            User: Created user object

        Raises:
            Exception: If email already exists or database error
        """
        query = """
            INSERT INTO users (id, email, password_hash, created_at, updated_at, account_status)
            VALUES ($1, $2, $3, $4, $5, $6)
            RETURNING id, email, password_hash, created_at, updated_at, last_sign_in_at, account_status
        """
        user_id = uuid.uuid4()
        now = datetime.utcnow()

        try:
            record = await self.fetch_one(
                query,
                user_id,
                email.lower(),  # Store emails as lowercase for case-insensitive lookup
                password_hash,
                now,
                now,
                account_status.value
            )

            if record is None:
                raise Exception("Failed to create user")

            return User(
                id=record['id'],
                email=record['email'],
                password_hash=record['password_hash'],
                created_at=record['created_at'],
                updated_at=record['updated_at'],
                last_sign_in_at=record['last_sign_in_at'],
                account_status=AccountStatus(record['account_status'])
            )
        except Exception as e:
            logger.error(f"Error creating user: {e}")
            raise

    async def get_user_by_email(self, email: str) -> Optional[User]:
        """
        Retrieve user by email address.

        Args:
            email: User's email address

        Returns:
            Optional[User]: User object if found, None otherwise
        """
        query = """
            SELECT id, email, password_hash, created_at, updated_at, last_sign_in_at, account_status
            FROM users
            WHERE email = $1
        """
        try:
            record = await self.fetch_one(query, email.lower())

            if record is None:
                return None

            return User(
                id=record['id'],
                email=record['email'],
                password_hash=record['password_hash'],
                created_at=record['created_at'],
                updated_at=record['updated_at'],
                last_sign_in_at=record['last_sign_in_at'],
                account_status=AccountStatus(record['account_status'])
            )
        except Exception as e:
            logger.error(f"Error retrieving user by email: {e}")
            raise

    async def get_user_by_id(self, user_id: uuid.UUID) -> Optional[User]:
        """
        Retrieve user by UUID.

        Args:
            user_id: User's unique identifier

        Returns:
            Optional[User]: User object if found, None otherwise
        """
        query = """
            SELECT id, email, password_hash, created_at, updated_at, last_sign_in_at, account_status
            FROM users
            WHERE id = $1
        """
        try:
            record = await self.fetch_one(query, user_id)

            if record is None:
                return None

            return User(
                id=record['id'],
                email=record['email'],
                password_hash=record['password_hash'],
                created_at=record['created_at'],
                updated_at=record['updated_at'],
                last_sign_in_at=record['last_sign_in_at'],
                account_status=AccountStatus(record['account_status'])
            )
        except Exception as e:
            logger.error(f"Error retrieving user by ID: {e}")
            raise

    async def update_last_sign_in(self, user_id: uuid.UUID) -> None:
        """
        Update last sign-in timestamp for user (T038).

        Args:
            user_id: User's unique identifier
        """
        query = """
            UPDATE users
            SET last_sign_in_at = $1, updated_at = $1
            WHERE id = $2
        """
        now = datetime.utcnow()

        try:
            await self.execute(query, now, user_id)
            logger.info(f"Updated last_sign_in_at for user {user_id}")
        except Exception as e:
            logger.error(f"Error updating last_sign_in_at: {e}")
            raise

    async def save_user_interests(self, user_id: uuid.UUID, interest_ids: List[int]) -> None:
        """
        Associate interests with user (T023).

        This method inserts records into the user_interests junction table.

        Args:
            user_id: User's unique identifier
            interest_ids: List of interest category IDs (2-5 items)
        """
        # First, delete any existing interests for this user
        delete_query = "DELETE FROM user_interests WHERE user_id = $1"
        await self.execute(delete_query, user_id)

        # Then insert new interests
        insert_query = """
            INSERT INTO user_interests (user_id, interest_id, created_at)
            VALUES ($1, $2, $3)
        """
        now = datetime.utcnow()
        args_list = [(user_id, interest_id, now) for interest_id in interest_ids]

        try:
            await self.execute_many(insert_query, args_list)
            logger.info(f"Saved {len(interest_ids)} interests for user {user_id}")
        except Exception as e:
            logger.error(f"Error saving user interests: {e}")
            raise

    async def get_user_interests(self, user_id: uuid.UUID) -> List[int]:
        """
        Retrieve user's interest IDs (T039).

        Args:
            user_id: User's unique identifier

        Returns:
            List[int]: List of interest category IDs
        """
        query = """
            SELECT interest_id
            FROM user_interests
            WHERE user_id = $1
            ORDER BY interest_id
        """
        try:
            records = await self.fetch_all(query, user_id)
            return [record['interest_id'] for record in records]
        except Exception as e:
            logger.error(f"Error retrieving user interests: {e}")
            raise

    async def update_user_interests(self, user_id: uuid.UUID, interest_ids: List[int]) -> None:
        """
        Update user's interest selections (T068).

        Deletes old associations and inserts new ones.

        Args:
            user_id: User's unique identifier
            interest_ids: List of interest category IDs (2-5 items)
        """
        # Reuse save_user_interests which already handles delete + insert
        await self.save_user_interests(user_id, interest_ids)
        logger.info(f"Updated interests for user {user_id}")

    async def create_user_profile(self, user_id: uuid.UUID, background: str, language_preference: str) -> None:
        """
        Create user profile with background and language preference (T023).

        Args:
            user_id: User's unique identifier
            background: User's background ('student' or 'professional')
            language_preference: User's language preference ('en' or 'ur')
        """
        query = """
            INSERT INTO user_profiles (user_id, background, language_preference, created_at, updated_at)
            VALUES ($1, $2, $3, $4, $5)
        """
        now = datetime.utcnow()

        try:
            await self.execute(query, user_id, background, language_preference, now, now)
            logger.info(f"Created user profile for user {user_id}")
        except Exception as e:
            logger.error(f"Error creating user profile: {e}")
            raise

    async def get_user_profile(self, user_id: uuid.UUID) -> Optional[Dict[str, Any]]:
        """
        Retrieve user profile information (background, language preference).

        Args:
            user_id: User's unique identifier

        Returns:
            Optional[Dict]: User profile data if found, None otherwise
        """
        query = """
            SELECT user_id, background, language_preference, created_at, updated_at
            FROM user_profiles
            WHERE user_id = $1
        """
        try:
            record = await self.fetch_one(query, user_id)

            if record is None:
                return None

            return {
                'user_id': record['user_id'],
                'background': record['background'],
                'language_preference': record['language_preference'],
                'created_at': record['created_at'],
                'updated_at': record['updated_at']
            }
        except Exception as e:
            logger.error(f"Error retrieving user profile: {e}")
            raise

    async def invalidate_all_sessions(self, user_id: uuid.UUID) -> None:
        """
        Invalidate all active sessions for user (T052).

        Called when user changes password for security.

        Args:
            user_id: User's unique identifier
        """
        query = "DELETE FROM sessions WHERE user_id = $1"
        try:
            await self.execute(query, user_id)
            logger.info(f"Invalidated all sessions for user {user_id}")
        except Exception as e:
            logger.error(f"Error invalidating sessions: {e}")
            raise
