"""
Translation service for converting text to Urdu.

Uses deep-translator library with Google Translate for Urdu translation.
Preserves code blocks and technical terms during translation.
"""

import time
import logging
from typing import Dict, Optional
from deep_translator import GoogleTranslator
from utils.text_processing import (
    preserve_code_blocks,
    restore_code_blocks,
    count_words,
    count_preserved_elements,
)
from utils.errors import TranslationError

logger = logging.getLogger(__name__)


class TranslationService:
    """
    Service for translating text to Urdu with code preservation.

    Features:
    - Translates English text to Urdu (and vice versa)
    - Preserves code blocks, inline code, and technical terms
    - Optional caching for performance
    - Error handling with graceful fallbacks
    """

    def __init__(self, enable_cache: bool = True):
        """
        Initialize the translation service.

        Args:
            enable_cache: Whether to enable translation caching (default: True)
        """
        self.enable_cache = enable_cache
        self._cache: Dict[str, str] = {}
        self._translator_en_to_ur = GoogleTranslator(source='en', target='ur')
        self._translator_ur_to_en = GoogleTranslator(source='ur', target='en')

    def translate_to_urdu(
        self,
        text: str,
        preserve_code: bool = True,
        user_id: Optional[str] = None
    ) -> Dict:
        """
        Translate text from English to Urdu.

        Args:
            text: Text to translate
            preserve_code: Whether to preserve code blocks during translation
            user_id: Optional user ID for logging

        Returns:
            Dictionary with translation results:
            - translated_text: Translated text
            - original_text: Original input
            - detected_lang: Detected source language
            - target_lang: Target language ('ur')
            - word_count: Word count of original text
            - translation_time_ms: Processing time in milliseconds
            - preserved_elements: Count of preserved code blocks

        Raises:
            TranslationError: If translation fails
        """
        start_time = time.time()

        try:
            # Validate input
            if not text or not text.strip():
                raise TranslationError("Text is empty", error_code="EMPTY_TEXT")

            word_count = count_words(text)
            if word_count > 5000:
                raise TranslationError(
                    f"Text exceeds maximum 5,000 words (got {word_count})",
                    error_code="TEXT_TOO_LONG"
                )

            # Check cache
            cache_key = f"en_ur_{text[:100]}"  # Use first 100 chars as cache key
            if self.enable_cache and cache_key in self._cache:
                logger.info(f"Cache hit for translation (user_id={user_id})")
                cached_translation = self._cache[cache_key]
                elapsed_ms = int((time.time() - start_time) * 1000)

                return {
                    "translated_text": cached_translation,
                    "original_text": text,
                    "detected_lang": "en",
                    "target_lang": "ur",
                    "word_count": word_count,
                    "translation_time_ms": elapsed_ms,
                    "preserved_elements": {},
                }

            # Preserve code blocks if requested
            placeholder_map = {}
            text_to_translate = text

            if preserve_code:
                text_to_translate, placeholder_map = preserve_code_blocks(text)
                logger.info(f"Preserved {len(placeholder_map)} code blocks")

            # Translate text
            try:
                translated = self._translator_en_to_ur.translate(text_to_translate)
            except Exception as e:
                logger.error(f"Translation API error: {e}")
                raise TranslationError(
                    "Translation service unavailable. Please try again later.",
                    error_code="API_ERROR",
                    original_exception=e
                )

            # Restore code blocks
            if preserve_code and placeholder_map:
                translated = restore_code_blocks(translated, placeholder_map)

            # Cache the result
            if self.enable_cache:
                self._cache[cache_key] = translated

            # Calculate metrics
            elapsed_ms = int((time.time() - start_time) * 1000)
            preserved_elements = count_preserved_elements(placeholder_map) if preserve_code else {}

            logger.info(
                f"Translation completed: {word_count} words in {elapsed_ms}ms "
                f"(user_id={user_id}, preserved={len(placeholder_map)} blocks)"
            )

            return {
                "translated_text": translated,
                "original_text": text,
                "detected_lang": "en",
                "target_lang": "ur",
                "word_count": word_count,
                "translation_time_ms": elapsed_ms,
                "preserved_elements": preserved_elements,
            }

        except TranslationError:
            raise
        except Exception as e:
            logger.error(f"Unexpected translation error: {e}")
            raise TranslationError(
                "An unexpected error occurred during translation",
                error_code="UNEXPECTED_ERROR",
                original_exception=e
            )

    def translate_to_english(
        self,
        text: str,
        user_id: Optional[str] = None
    ) -> Dict:
        """
        Translate text from Urdu to English.

        Args:
            text: Urdu text to translate
            user_id: Optional user ID for logging

        Returns:
            Dictionary with translation results (same structure as translate_to_urdu)

        Raises:
            TranslationError: If translation fails
        """
        start_time = time.time()

        try:
            # Validate input
            if not text or not text.strip():
                raise TranslationError("Text is empty", error_code="EMPTY_TEXT")

            word_count = count_words(text)

            # Check cache
            cache_key = f"ur_en_{text[:100]}"
            if self.enable_cache and cache_key in self._cache:
                logger.info(f"Cache hit for Urdu->English translation (user_id={user_id})")
                cached_translation = self._cache[cache_key]
                elapsed_ms = int((time.time() - start_time) * 1000)

                return {
                    "translated_text": cached_translation,
                    "original_text": text,
                    "detected_lang": "ur",
                    "target_lang": "en",
                    "word_count": word_count,
                    "translation_time_ms": elapsed_ms,
                    "preserved_elements": {},
                }

            # Translate text
            try:
                translated = self._translator_ur_to_en.translate(text)
            except Exception as e:
                logger.error(f"Translation API error: {e}")
                raise TranslationError(
                    "Translation service unavailable. Please try again later.",
                    error_code="API_ERROR",
                    original_exception=e
                )

            # Cache the result
            if self.enable_cache:
                self._cache[cache_key] = translated

            # Calculate metrics
            elapsed_ms = int((time.time() - start_time) * 1000)

            logger.info(
                f"Urdu->English translation completed: {word_count} words in {elapsed_ms}ms "
                f"(user_id={user_id})"
            )

            return {
                "translated_text": translated,
                "original_text": text,
                "detected_lang": "ur",
                "target_lang": "en",
                "word_count": word_count,
                "translation_time_ms": elapsed_ms,
                "preserved_elements": {},
            }

        except TranslationError:
            raise
        except Exception as e:
            logger.error(f"Unexpected translation error: {e}")
            raise TranslationError(
                "An unexpected error occurred during translation",
                error_code="UNEXPECTED_ERROR",
                original_exception=e
            )

    def clear_cache(self) -> None:
        """Clear the translation cache."""
        self._cache.clear()
        logger.info("Translation cache cleared")

    def get_cache_size(self) -> int:
        """Get the current cache size."""
        return len(self._cache)
