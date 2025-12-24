"""
Personalization Service

Generates personalized system prompts and analogies based on user interests.
Integrates with RAG chatbot to provide contextually relevant explanations.
"""

from typing import Dict, List, Optional, Any
from uuid import UUID
import logging
from services.interests_service import InterestsService
from utils.errors import ValidationError

logger = logging.getLogger(__name__)


class PersonalizationService:
    """
    Service for personalizing chatbot responses based on user interests.

    Provides methods to build personalized system prompts, create analogies,
    and adapt explanations based on user background and interests.
    """

    # Interest-to-analogy mappings
    INTEREST_ANALOGIES = {
        "software-engineering": {
            "analogies": [
                "Consider how this is similar to designing a software architecture",
                "Think of this like a class hierarchy in object-oriented programming",
                "This is analogous to API design patterns you might use in web development",
                "Like refactoring code for maintainability, this approach optimizes for...",
            ],
            "examples": [
                "For instance, if you're building a REST API",
                "Similar to how you'd structure a microservices application",
                "Just as you would use design patterns like Factory or Observer",
            ],
            "keywords": ["code", "algorithm", "function", "class", "API", "database", "programming"],
        },
        "mechanical-engineering": {
            "analogies": [
                "Think of this like mechanical linkages in a machine",
                "Similar to how gears transfer motion and force",
                "This is analogous to stress distribution in mechanical structures",
                "Like optimizing material selection for strength and weight",
            ],
            "examples": [
                "For example, in a car's suspension system",
                "Similar to how a robotic arm's joints work",
                "Just like analyzing forces in a truss bridge",
            ],
            "keywords": ["force", "torque", "mechanism", "motion", "dynamics", "kinematics", "material"],
        },
        "electrical-engineering": {
            "analogies": [
                "Think of this like current flowing through a circuit",
                "Similar to voltage regulation in power systems",
                "This is analogous to signal processing in communication systems",
                "Like filtering noise from a sensor signal",
            ],
            "examples": [
                "For instance, in a motor control circuit",
                "Similar to how PWM controls actuators",
                "Just as you'd design a feedback control system",
            ],
            "keywords": ["circuit", "voltage", "current", "sensor", "actuator", "signal", "power"],
        },
        "artificial-intelligence-machine-learning": {
            "analogies": [
                "Think of this like training a neural network",
                "Similar to how machine learning models learn patterns from data",
                "This is analogous to optimization in gradient descent",
                "Like feature engineering for better model performance",
            ],
            "examples": [
                "For example, in computer vision applications",
                "Similar to how reinforcement learning agents explore environments",
                "Just as you'd tune hyperparameters for optimal accuracy",
            ],
            "keywords": ["model", "training", "data", "algorithm", "neural network", "learning", "prediction"],
        },
        "robotics-hardware": {
            "analogies": [
                "Think of this like selecting actuators for a robot",
                "Similar to how you'd design a gripper mechanism",
                "This is analogous to sensor integration in robotic systems",
                "Like balancing payload capacity with mobility",
            ],
            "examples": [
                "For instance, when choosing motors for a mobile robot",
                "Similar to how servos and encoders work together",
                "Just as you'd integrate IMUs and cameras",
            ],
            "keywords": ["actuator", "sensor", "motor", "gripper", "camera", "hardware", "component"],
        },
        "computer-vision": {
            "analogies": [
                "Think of this like feature detection in images",
                "Similar to how convolutional neural networks process visual data",
                "This is analogous to object detection pipelines",
                "Like applying filters for edge detection",
            ],
            "examples": [
                "For example, in facial recognition systems",
                "Similar to how SLAM algorithms build maps",
                "Just as you'd use OpenCV for image processing",
            ],
            "keywords": ["image", "camera", "detection", "recognition", "vision", "visual", "perception"],
        },
        "control-systems": {
            "analogies": [
                "Think of this like PID controller tuning",
                "Similar to how feedback loops stabilize systems",
                "This is analogous to state-space representation",
                "Like designing controllers for optimal response time",
            ],
            "examples": [
                "For instance, in temperature control systems",
                "Similar to how autopilot maintains aircraft stability",
                "Just as you'd implement a Kalman filter",
            ],
            "keywords": ["control", "feedback", "PID", "stability", "response", "tuning", "loop"],
        },
        "human-robot-interaction": {
            "analogies": [
                "Think of this like designing intuitive user interfaces",
                "Similar to how social robots interpret human gestures",
                "This is analogous to natural language processing for voice commands",
                "Like ensuring safety in collaborative robotics",
            ],
            "examples": [
                "For example, in assistive robotics for elderly care",
                "Similar to how chatbots understand user intent",
                "Just as you'd design gestures for robot control",
            ],
            "keywords": ["interaction", "gesture", "voice", "interface", "social", "collaborative", "safety"],
        },
    }

    # Background-specific tone adjustments
    BACKGROUND_TONES = {
        "student": {
            "tone": "educational and encouraging",
            "approach": "step-by-step explanations with learning resources",
            "complexity": "intermediate, with foundational concepts explained",
            "style": "Break down concepts into digestible parts, provide analogies, and suggest further reading.",
        },
        "professional": {
            "tone": "concise and technical",
            "approach": "practical applications with industry best practices",
            "complexity": "advanced, assume foundational knowledge",
            "style": "Focus on implementation details, trade-offs, and real-world applications.",
        },
    }

    def __init__(self, interests_service: Optional[InterestsService] = None):
        """
        Initialize the PersonalizationService.

        Args:
            interests_service: Optional InterestsService instance (creates new if None)
        """
        self.interests_service = interests_service or InterestsService()

    async def build_personalized_system_prompt(
        self, user_id: UUID, base_prompt: str
    ) -> Dict[str, Any]:
        """
        Build a personalized system prompt based on user interests and background.

        Args:
            user_id: UUID of the user
            base_prompt: Base system prompt for the agent

        Returns:
            Dict containing:
                - personalized_prompt: Enhanced system prompt
                - user_interests: List of interest names
                - is_personalized: Boolean indicating if personalization was applied
                - background: User's background (student/professional)
                - language_preference: User's language preference

        Raises:
            DatabaseError: If database operation fails
        """
        try:
            # Fetch user profile and interests
            user_profile = await self.interests_service.get_user_interests(user_id)

            if not user_profile or not user_profile.get("interests"):
                # User has no interests, return base prompt
                logger.info(f"No interests found for user {user_id}, using base prompt")
                return {
                    "personalized_prompt": base_prompt,
                    "user_interests": [],
                    "is_personalized": False,
                    "background": None,
                    "language_preference": "en",
                }

            # Extract user data
            interests = user_profile["interests"]
            background = user_profile.get("background", "student")
            language_preference = user_profile.get("language_preference", "en")
            interest_names = [interest["name"] for interest in interests]
            interest_slugs = [interest["slug"] for interest in interests]

            logger.info(
                f"Building personalized prompt for user {user_id}: "
                f"background={background}, interests={interest_names}"
            )

            # Build personalized prompt
            personalized_prompt = self._construct_prompt(
                base_prompt=base_prompt,
                interests=interests,
                interest_slugs=interest_slugs,
                background=background,
                language_preference=language_preference,
            )

            return {
                "personalized_prompt": personalized_prompt,
                "user_interests": interest_names,
                "is_personalized": True,
                "background": background,
                "language_preference": language_preference,
            }

        except Exception as e:
            logger.error(f"Error building personalized prompt: {e}")
            # Fallback to base prompt on error
            return {
                "personalized_prompt": base_prompt,
                "user_interests": [],
                "is_personalized": False,
                "background": None,
                "language_preference": "en",
            }

    def _construct_prompt(
        self,
        base_prompt: str,
        interests: List[Dict],
        interest_slugs: List[str],
        background: str,
        language_preference: str,
    ) -> str:
        """
        Construct the personalized system prompt.

        Args:
            base_prompt: Original system prompt
            interests: List of user interests with details
            interest_slugs: List of interest slugs for mapping
            background: User's background (student/professional)
            language_preference: User's language preference

        Returns:
            Enhanced system prompt with personalization
        """
        # Get background tone
        tone_config = self.BACKGROUND_TONES.get(background, self.BACKGROUND_TONES["student"])

        # Build interest-specific guidance
        interest_guidance = self._build_interest_guidance(interest_slugs)

        # Construct personalized prompt
        personalized_sections = []

        # Add background-specific instructions
        personalized_sections.append(
            f"\n## Personalization Context\n\n"
            f"You are communicating with a **{background}** who has expressed interest in:\n"
        )

        # List user interests
        for interest in interests:
            personalized_sections.append(f"- {interest['name']}: {interest['description']}")

        # Add tone and approach guidance
        personalized_sections.append(
            f"\n## Communication Style\n\n"
            f"- **Tone**: {tone_config['tone']}\n"
            f"- **Approach**: {tone_config['approach']}\n"
            f"- **Complexity**: {tone_config['complexity']}\n"
            f"- **Style**: {tone_config['style']}\n"
        )

        # Add interest-specific guidance
        if interest_guidance:
            personalized_sections.append(
                f"\n## Interest-Based Explanations\n\n{interest_guidance}"
            )

        # Add language guidance if Urdu
        if language_preference == "ur":
            personalized_sections.append(
                f"\n## Language Preference\n\n"
                f"The user prefers content in Urdu. While you should respond in English "
                f"(as translation is handled separately), be mindful that the user may "
                f"benefit from simpler vocabulary and clearer explanations that translate well."
            )

        # Add grounding reminder
        personalized_sections.append(
            f"\n## Important: Stay Grounded\n\n"
            f"While personalizing your response with relevant examples and analogies, "
            f"**always prioritize factual accuracy**. Base your answers on the provided "
            f"context from the knowledge base. Use analogies and examples to enhance understanding, "
            f"but never sacrifice correctness for relevance."
        )

        # Combine base prompt with personalization
        return base_prompt + "\n" + "".join(personalized_sections)

    def _build_interest_guidance(self, interest_slugs: List[str]) -> str:
        """
        Build guidance text based on user interests.

        Args:
            interest_slugs: List of interest slugs

        Returns:
            Formatted guidance text with analogies and examples
        """
        guidance_parts = []

        for slug in interest_slugs:
            if slug in self.INTEREST_ANALOGIES:
                mapping = self.INTEREST_ANALOGIES[slug]
                interest_name = slug.replace("-", " ").title()

                guidance_parts.append(f"\n**{interest_name}**:")
                guidance_parts.append(
                    f"- When relevant, use analogies like: \"{mapping['analogies'][0]}\""
                )
                guidance_parts.append(
                    f"- Provide examples such as: \"{mapping['examples'][0]}\""
                )
                guidance_parts.append(
                    f"- Emphasize keywords: {', '.join(mapping['keywords'][:5])}"
                )

        return "\n".join(guidance_parts) if guidance_parts else ""

    def get_relevant_analogies(self, interest_slug: str, count: int = 2) -> List[str]:
        """
        Get relevant analogies for a specific interest.

        Args:
            interest_slug: Slug of the interest category
            count: Number of analogies to return

        Returns:
            List of analogy strings
        """
        if interest_slug in self.INTEREST_ANALOGIES:
            analogies = self.INTEREST_ANALOGIES[interest_slug]["analogies"]
            return analogies[:count]
        return []

    def get_relevant_examples(self, interest_slug: str, count: int = 2) -> List[str]:
        """
        Get relevant examples for a specific interest.

        Args:
            interest_slug: Slug of the interest category
            count: Number of examples to return

        Returns:
            List of example strings
        """
        if interest_slug in self.INTEREST_ANALOGIES:
            examples = self.INTEREST_ANALOGIES[interest_slug]["examples"]
            return examples[:count]
        return []

    def get_relevant_keywords(self, interest_slug: str) -> List[str]:
        """
        Get relevant keywords for a specific interest.

        Args:
            interest_slug: Slug of the interest category

        Returns:
            List of keyword strings
        """
        if interest_slug in self.INTEREST_ANALOGIES:
            return self.INTEREST_ANALOGIES[interest_slug]["keywords"]
        return []

    async def should_personalize(self, user_id: Optional[UUID]) -> bool:
        """
        Check if personalization should be applied for a user.

        Args:
            user_id: UUID of the user (None for guest users)

        Returns:
            Boolean indicating if personalization should be applied
        """
        if not user_id:
            return False

        try:
            has_interests = await self.interests_service.check_user_has_interests(user_id)
            return has_interests
        except Exception as e:
            logger.error(f"Error checking if user should be personalized: {e}")
            return False
