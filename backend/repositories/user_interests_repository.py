"""
User Interests Repository

Handles data access for user profiles and interests in Neon PostgreSQL.
Implements CRUD operations for user interest management.
"""

from typing import List, Optional, Dict, Any
from uuid import UUID
import logging
from repositories.base_repository import BaseRepository
from utils.errors import DatabaseError

logger = logging.getLogger(__name__)


class UserInterestsRepository(BaseRepository):
    """
    Repository for managing user interests data.

    Provides methods to save, retrieve, and update user interests
    stored in Neon PostgreSQL database.
    """

    async def save_user_interests(
        self,
        user_id: UUID,
        interest_ids: List[int],
        background: str = "student",
        language_preference: str = "en"
    ) -> Dict[str, Any]:
        """
        Save user interests to the database.

        Creates or updates user profile and associates selected interests.
        Uses transaction to ensure atomicity.

        Args:
            user_id: UUID of the user
            interest_ids: List of interest category IDs (2-5 interests required)
            background: User background ('student' or 'professional')
            language_preference: Preferred language ('en' or 'ur')

        Returns:
            Dict containing saved user_id, background, language_preference, and interest_ids

        Raises:
            DatabaseError: If database operation fails
            ValueError: If interest_ids list is invalid (not 2-5 interests)
        """
        # Validate interest_ids count
        if not interest_ids or len(interest_ids) < 2:
            raise ValueError("Minimum 2 interests required")
        if len(interest_ids) > 5:
            raise ValueError("Maximum 5 interests allowed")

        # Validate background and language_preference
        if background not in ('student', 'professional'):
            raise ValueError("Background must be 'student' or 'professional'")
        if language_preference not in ('en', 'ur'):
            raise ValueError("Language preference must be 'en' or 'ur'")

        pool = await self._get_pool()

        try:
            async with pool.acquire() as conn:
                # Start transaction
                async with conn.transaction():
                    # Insert or update user profile
                    profile_query = """
                        INSERT INTO user_profiles (user_id, background, language_preference)
                        VALUES ($1, $2, $3)
                        ON CONFLICT (user_id)
                        DO UPDATE SET
                            background = EXCLUDED.background,
                            language_preference = EXCLUDED.language_preference,
                            updated_at = CURRENT_TIMESTAMP
                        RETURNING user_id, background, language_preference, created_at, updated_at
                    """
                    profile_result = await conn.fetchrow(
                        profile_query,
                        str(user_id),
                        background,
                        language_preference
                    )

                    # Delete existing interests for this user
                    delete_query = "DELETE FROM user_interests WHERE user_id = $1"
                    await conn.execute(delete_query, str(user_id))

                    # Insert new interests
                    insert_interests_query = """
                        INSERT INTO user_interests (user_id, interest_id)
                        VALUES ($1, $2)
                    """
                    interest_data = [(str(user_id), interest_id) for interest_id in interest_ids]
                    await conn.executemany(insert_interests_query, interest_data)

                    logger.info(
                        f"Saved interests for user {user_id}: {len(interest_ids)} interests"
                    )

                    return {
                        "user_id": str(user_id),
                        "background": profile_result['background'],
                        "language_preference": profile_result['language_preference'],
                        "interest_ids": interest_ids,
                        "created_at": profile_result['created_at'].isoformat(),
                        "updated_at": profile_result.get('updated_at', profile_result['created_at']).isoformat()
                    }

        except ValueError as e:
            # Re-raise validation errors
            raise
        except Exception as e:
            logger.error(f"Error saving user interests: {e}")
            raise DatabaseError(
                message=f"Failed to save user interests: {str(e)}",
                error_code="INTERESTS_SAVE_FAILED",
                original_exception=e
            )

    async def get_user_interests(self, user_id: UUID) -> Optional[Dict[str, Any]]:
        """
        Retrieve user interests from the database.

        Fetches user profile and all associated interests with category details.

        Args:
            user_id: UUID of the user

        Returns:
            Dict containing user profile and interests, or None if user not found
            {
                "user_id": str,
                "background": str,
                "language_preference": str,
                "interests": [
                    {
                        "id": int,
                        "name": str,
                        "slug": str,
                        "description": str
                    },
                    ...
                ]
            }

        Raises:
            DatabaseError: If database operation fails
        """
        try:
            # Fetch user profile
            profile_query = """
                SELECT user_id, background, language_preference, created_at, updated_at
                FROM user_profiles
                WHERE user_id = $1
            """
            profile = await self.fetch_one(profile_query, str(user_id))

            if not profile:
                logger.info(f"User profile not found for user_id: {user_id}")
                return None

            # Fetch user interests with category details
            interests_query = """
                SELECT ic.id, ic.name, ic.slug, ic.description
                FROM user_interests ui
                JOIN interest_categories ic ON ui.interest_id = ic.id
                WHERE ui.user_id = $1
                ORDER BY ic.name
            """
            interests = await self.fetch_all(interests_query, str(user_id))

            return {
                "user_id": profile['user_id'],
                "background": profile['background'],
                "language_preference": profile['language_preference'],
                "created_at": profile['created_at'].isoformat(),
                "updated_at": profile.get('updated_at', profile['created_at']).isoformat(),
                "interests": self.dicts_from_records(interests)
            }

        except Exception as e:
            logger.error(f"Error retrieving user interests: {e}")
            raise DatabaseError(
                message=f"Failed to retrieve user interests: {str(e)}",
                error_code="INTERESTS_FETCH_FAILED",
                original_exception=e
            )

    async def update_user_interests(
        self,
        user_id: UUID,
        interest_ids: List[int],
        background: Optional[str] = None,
        language_preference: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Update user interests in the database.

        Updates interests and optionally updates background and language preference.
        If background or language_preference are None, they remain unchanged.

        Args:
            user_id: UUID of the user
            interest_ids: New list of interest category IDs (2-5 interests required)
            background: Optional new background ('student' or 'professional')
            language_preference: Optional new language preference ('en' or 'ur')

        Returns:
            Dict containing updated user_id, background, language_preference, and interest_ids

        Raises:
            DatabaseError: If database operation fails
            ValueError: If user not found or invalid parameters
        """
        # Validate interest_ids count
        if not interest_ids or len(interest_ids) < 2:
            raise ValueError("Minimum 2 interests required")
        if len(interest_ids) > 5:
            raise ValueError("Maximum 5 interests allowed")

        # Validate optional parameters if provided
        if background is not None and background not in ('student', 'professional'):
            raise ValueError("Background must be 'student' or 'professional'")
        if language_preference is not None and language_preference not in ('en', 'ur'):
            raise ValueError("Language preference must be 'en' or 'ur'")

        pool = await self._get_pool()

        try:
            async with pool.acquire() as conn:
                # Start transaction
                async with conn.transaction():
                    # Check if user profile exists
                    check_query = "SELECT user_id, background, language_preference FROM user_profiles WHERE user_id = $1"
                    existing_profile = await conn.fetchrow(check_query, str(user_id))

                    if not existing_profile:
                        raise ValueError(f"User profile not found for user_id: {user_id}")

                    # Determine final values (use existing if not provided)
                    final_background = background if background is not None else existing_profile['background']
                    final_language = language_preference if language_preference is not None else existing_profile['language_preference']

                    # Update user profile
                    update_profile_query = """
                        UPDATE user_profiles
                        SET background = $2, language_preference = $3, updated_at = CURRENT_TIMESTAMP
                        WHERE user_id = $1
                        RETURNING user_id, background, language_preference, created_at, updated_at
                    """
                    profile_result = await conn.fetchrow(
                        update_profile_query,
                        str(user_id),
                        final_background,
                        final_language
                    )

                    # Delete existing interests
                    delete_query = "DELETE FROM user_interests WHERE user_id = $1"
                    await conn.execute(delete_query, str(user_id))

                    # Insert new interests
                    insert_interests_query = """
                        INSERT INTO user_interests (user_id, interest_id)
                        VALUES ($1, $2)
                    """
                    interest_data = [(str(user_id), interest_id) for interest_id in interest_ids]
                    await conn.executemany(insert_interests_query, interest_data)

                    logger.info(
                        f"Updated interests for user {user_id}: {len(interest_ids)} interests"
                    )

                    return {
                        "user_id": str(user_id),
                        "background": profile_result['background'],
                        "language_preference": profile_result['language_preference'],
                        "interest_ids": interest_ids,
                        "created_at": profile_result['created_at'].isoformat(),
                        "updated_at": profile_result['updated_at'].isoformat()
                    }

        except ValueError as e:
            # Re-raise validation errors
            raise
        except Exception as e:
            logger.error(f"Error updating user interests: {e}")
            raise DatabaseError(
                message=f"Failed to update user interests: {str(e)}",
                error_code="INTERESTS_UPDATE_FAILED",
                original_exception=e
            )

    async def get_all_interest_categories(self) -> List[Dict[str, Any]]:
        """
        Retrieve all available interest categories.

        Returns:
            List of all interest categories
            [
                {
                    "id": int,
                    "name": str,
                    "slug": str,
                    "description": str
                },
                ...
            ]

        Raises:
            DatabaseError: If database operation fails
        """
        try:
            query = """
                SELECT id, name, slug, description
                FROM interest_categories
                ORDER BY name
            """
            categories = await self.fetch_all(query)

            return self.dicts_from_records(categories)

        except Exception as e:
            logger.error(f"Error retrieving interest categories: {e}")
            raise DatabaseError(
                message=f"Failed to retrieve interest categories: {str(e)}",
                error_code="CATEGORIES_FETCH_FAILED",
                original_exception=e
            )

    async def delete_user_interests(self, user_id: UUID) -> bool:
        """
        Delete all interests for a user (but keep the profile).

        Useful for account cleanup or resetting interests.

        Args:
            user_id: UUID of the user

        Returns:
            bool: True if interests were deleted, False if user had no interests

        Raises:
            DatabaseError: If database operation fails
        """
        try:
            query = "DELETE FROM user_interests WHERE user_id = $1"
            result = await self.execute(query, str(user_id))

            # Result format: "DELETE N" where N is number of rows deleted
            rows_deleted = int(result.split()[-1])

            logger.info(f"Deleted {rows_deleted} interests for user {user_id}")

            return rows_deleted > 0

        except Exception as e:
            logger.error(f"Error deleting user interests: {e}")
            raise DatabaseError(
                message=f"Failed to delete user interests: {str(e)}",
                error_code="INTERESTS_DELETE_FAILED",
                original_exception=e
            )
