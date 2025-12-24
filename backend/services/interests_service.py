"""
Interests Service

Business logic layer for user interest management.
Handles validation, processing, and orchestration of interest operations.
"""

from typing import List, Dict, Any, Optional
from uuid import UUID
import logging
from repositories.user_interests_repository import UserInterestsRepository
from utils.errors import ValidationError, DatabaseError

logger = logging.getLogger(__name__)


class InterestsService:
    """
    Service for managing user interests.

    Provides business logic for interest selection, validation,
    and user profile management.
    """

    # Interest category constraints
    MIN_INTERESTS = 2
    MAX_INTERESTS = 5

    # Valid background types
    VALID_BACKGROUNDS = {'student', 'professional'}

    # Valid language preferences
    VALID_LANGUAGES = {'en', 'ur'}

    def __init__(self, repository: Optional[UserInterestsRepository] = None):
        """
        Initialize the InterestsService.

        Args:
            repository: Optional UserInterestsRepository instance (creates new if None)
        """
        self.repository = repository or UserInterestsRepository()

    async def save_user_interests(
        self,
        user_id: UUID,
        interest_ids: List[int],
        background: str = "student",
        language_preference: str = "en"
    ) -> Dict[str, Any]:
        """
        Save user interests with validation.

        Args:
            user_id: UUID of the user
            interest_ids: List of interest category IDs
            background: User background ('student' or 'professional')
            language_preference: Preferred language ('en' or 'ur')

        Returns:
            Dict containing saved user profile and interests

        Raises:
            ValidationError: If validation fails
            DatabaseError: If database operation fails
        """
        # Validate interest selection
        self.validate_interest_selection(interest_ids)

        # Validate background
        if background not in self.VALID_BACKGROUNDS:
            raise ValidationError(
                message=f"Invalid background: '{background}'. Must be one of: {', '.join(self.VALID_BACKGROUNDS)}",
                error_code="INVALID_BACKGROUND"
            )

        # Validate language preference
        if language_preference not in self.VALID_LANGUAGES:
            raise ValidationError(
                message=f"Invalid language preference: '{language_preference}'. Must be one of: {', '.join(self.VALID_LANGUAGES)}",
                error_code="INVALID_LANGUAGE"
            )

        try:
            # Save to database
            result = await self.repository.save_user_interests(
                user_id=user_id,
                interest_ids=interest_ids,
                background=background,
                language_preference=language_preference
            )

            logger.info(
                f"Successfully saved interests for user {user_id}: "
                f"{len(interest_ids)} interests, background={background}, lang={language_preference}"
            )

            return result

        except ValueError as e:
            # Convert ValueError to ValidationError
            raise ValidationError(
                message=str(e),
                error_code="VALIDATION_FAILED"
            )
        except DatabaseError:
            # Re-raise DatabaseError as-is
            raise
        except Exception as e:
            logger.error(f"Unexpected error saving interests: {e}")
            raise DatabaseError(
                message="An unexpected error occurred while saving interests",
                error_code="SAVE_FAILED",
                original_exception=e
            )

    async def get_user_interests(self, user_id: UUID) -> Optional[Dict[str, Any]]:
        """
        Retrieve user interests.

        Args:
            user_id: UUID of the user

        Returns:
            Dict containing user profile and interests, or None if not found

        Raises:
            DatabaseError: If database operation fails
        """
        try:
            result = await self.repository.get_user_interests(user_id)

            if result:
                logger.info(f"Retrieved interests for user {user_id}: {len(result.get('interests', []))} interests")
            else:
                logger.info(f"No interests found for user {user_id}")

            return result

        except DatabaseError:
            raise
        except Exception as e:
            logger.error(f"Unexpected error retrieving interests: {e}")
            raise DatabaseError(
                message="An unexpected error occurred while retrieving interests",
                error_code="FETCH_FAILED",
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
        Update user interests with validation.

        Args:
            user_id: UUID of the user
            interest_ids: New list of interest category IDs
            background: Optional new background
            language_preference: Optional new language preference

        Returns:
            Dict containing updated user profile and interests

        Raises:
            ValidationError: If validation fails
            DatabaseError: If database operation fails
        """
        # Validate interest selection
        self.validate_interest_selection(interest_ids)

        # Validate optional background
        if background is not None and background not in self.VALID_BACKGROUNDS:
            raise ValidationError(
                message=f"Invalid background: '{background}'. Must be one of: {', '.join(self.VALID_BACKGROUNDS)}",
                error_code="INVALID_BACKGROUND"
            )

        # Validate optional language preference
        if language_preference is not None and language_preference not in self.VALID_LANGUAGES:
            raise ValidationError(
                message=f"Invalid language preference: '{language_preference}'. Must be one of: {', '.join(self.VALID_LANGUAGES)}",
                error_code="INVALID_LANGUAGE"
            )

        try:
            # Update in database
            result = await self.repository.update_user_interests(
                user_id=user_id,
                interest_ids=interest_ids,
                background=background,
                language_preference=language_preference
            )

            logger.info(f"Successfully updated interests for user {user_id}: {len(interest_ids)} interests")

            return result

        except ValueError as e:
            # Convert ValueError to ValidationError
            raise ValidationError(
                message=str(e),
                error_code="VALIDATION_FAILED"
            )
        except DatabaseError:
            raise
        except Exception as e:
            logger.error(f"Unexpected error updating interests: {e}")
            raise DatabaseError(
                message="An unexpected error occurred while updating interests",
                error_code="UPDATE_FAILED",
                original_exception=e
            )

    async def get_all_interest_categories(self) -> List[Dict[str, Any]]:
        """
        Retrieve all available interest categories.

        Returns:
            List of all interest categories

        Raises:
            DatabaseError: If database operation fails
        """
        try:
            categories = await self.repository.get_all_interest_categories()

            logger.info(f"Retrieved {len(categories)} interest categories")

            return categories

        except DatabaseError:
            raise
        except Exception as e:
            logger.error(f"Unexpected error retrieving categories: {e}")
            raise DatabaseError(
                message="An unexpected error occurred while retrieving categories",
                error_code="CATEGORIES_FETCH_FAILED",
                original_exception=e
            )

    def validate_interest_selection(self, interest_ids: List[int]) -> None:
        """
        Validate interest selection meets requirements.

        Validates:
        - interest_ids is a list
        - Contains 2-5 unique interests
        - All IDs are positive integers

        Args:
            interest_ids: List of interest category IDs

        Raises:
            ValidationError: If validation fails
        """
        # Check if list
        if not isinstance(interest_ids, list):
            raise ValidationError(
                message="interest_ids must be a list",
                error_code="INVALID_TYPE"
            )

        # Check if empty
        if not interest_ids:
            raise ValidationError(
                message=f"You must select at least {self.MIN_INTERESTS} interests",
                error_code="TOO_FEW_INTERESTS"
            )

        # Remove duplicates and check count
        unique_interests = list(set(interest_ids))

        if len(unique_interests) < self.MIN_INTERESTS:
            raise ValidationError(
                message=f"You must select at least {self.MIN_INTERESTS} different interests (selected: {len(unique_interests)})",
                error_code="TOO_FEW_INTERESTS"
            )

        if len(unique_interests) > self.MAX_INTERESTS:
            raise ValidationError(
                message=f"You can select at most {self.MAX_INTERESTS} interests (selected: {len(unique_interests)})",
                error_code="TOO_MANY_INTERESTS"
            )

        # Validate all IDs are positive integers
        for interest_id in unique_interests:
            if not isinstance(interest_id, int) or interest_id <= 0:
                raise ValidationError(
                    message=f"Invalid interest ID: {interest_id}. Must be a positive integer.",
                    error_code="INVALID_INTEREST_ID"
                )

        logger.debug(f"Validated interest selection: {len(unique_interests)} unique interests")

    async def delete_user_interests(self, user_id: UUID) -> bool:
        """
        Delete all interests for a user.

        Args:
            user_id: UUID of the user

        Returns:
            bool: True if interests were deleted

        Raises:
            DatabaseError: If database operation fails
        """
        try:
            result = await self.repository.delete_user_interests(user_id)

            if result:
                logger.info(f"Deleted interests for user {user_id}")
            else:
                logger.info(f"No interests to delete for user {user_id}")

            return result

        except DatabaseError:
            raise
        except Exception as e:
            logger.error(f"Unexpected error deleting interests: {e}")
            raise DatabaseError(
                message="An unexpected error occurred while deleting interests",
                error_code="DELETE_FAILED",
                original_exception=e
            )

    async def get_user_interest_names(self, user_id: UUID) -> List[str]:
        """
        Get list of interest names for a user (for display purposes).

        Args:
            user_id: UUID of the user

        Returns:
            List of interest names (e.g., ["Software Engineering", "Robotics"])

        Raises:
            DatabaseError: If database operation fails
        """
        user_data = await self.get_user_interests(user_id)

        if not user_data or 'interests' not in user_data:
            return []

        return [interest['name'] for interest in user_data['interests']]

    async def check_user_has_interests(self, user_id: UUID) -> bool:
        """
        Check if user has selected any interests.

        Args:
            user_id: UUID of the user

        Returns:
            bool: True if user has interests, False otherwise

        Raises:
            DatabaseError: If database operation fails
        """
        user_data = await self.get_user_interests(user_id)

        if not user_data:
            return False

        return len(user_data.get('interests', [])) >= self.MIN_INTERESTS
