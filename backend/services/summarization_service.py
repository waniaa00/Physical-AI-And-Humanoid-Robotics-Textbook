"""
Summarization service for creating concise summaries of text.

Uses Cohere LLM for generating high-quality summaries with configurable
length and focus (concepts vs. code).
"""

import time
import logging
import os
import cohere
from typing import Dict, List, Optional
from models.summarization import SummaryLength, SummaryFocus
from utils.text_processing import count_words, clean_whitespace
from utils.errors import SummarizationError

logger = logging.getLogger(__name__)


class SummarizationService:
    """
    Service for summarizing text using Cohere LLM.

    Features:
    - Configurable summary length (short/medium/long = 20%/30%/40%)
    - Focus control (concepts, code, or both)
    - Automatic key points extraction
    - Length validation (50-5000 words)
    - Error handling with graceful fallbacks
    """

    def __init__(self, cohere_api_key: Optional[str] = None):
        """
        Initialize the summarization service.

        Args:
            cohere_api_key: Cohere API key (defaults to env variable)
        """
        api_key = cohere_api_key or os.getenv("COHERE_API_KEY")
        if not api_key:
            raise ValueError("COHERE_API_KEY environment variable is not set")

        self.cohere_client = cohere.Client(api_key)
        self._cache: Dict[str, Dict] = {}

    def summarize_text(
        self,
        text: str,
        target_length: SummaryLength = SummaryLength.SHORT,
        focus: SummaryFocus = SummaryFocus.CONCEPTS,
        user_id: Optional[str] = None
    ) -> Dict:
        """
        Summarize text using Cohere LLM.

        Args:
            text: Text to summarize (50-5000 words)
            target_length: Desired summary length (short/medium/long)
            focus: Summary focus area (concepts/code/both)
            user_id: Optional user ID for logging

        Returns:
            Dictionary with summarization results:
            - summary: Generated summary text
            - original_word_count: Word count of original text
            - summary_word_count: Word count of summary
            - compression_ratio: Ratio of summary to original (0-1)
            - processing_time_ms: Processing time in milliseconds
            - key_points: Extracted key points (optional)

        Raises:
            SummarizationError: If summarization fails
        """
        start_time = time.time()

        try:
            # Validate input
            if not text or not text.strip():
                raise SummarizationError("Text is empty", error_code="EMPTY_TEXT")

            # Clean whitespace
            cleaned_text = clean_whitespace(text)
            original_word_count = count_words(cleaned_text)

            # Validate word count
            if original_word_count < 50:
                raise SummarizationError(
                    f"Text is too short to summarize (minimum 50 words, got {original_word_count})",
                    error_code="TEXT_TOO_SHORT"
                )

            if original_word_count > 5000:
                raise SummarizationError(
                    f"Text exceeds maximum 5,000 words (got {original_word_count})",
                    error_code="TEXT_TOO_LONG"
                )

            # Check cache
            cache_key = f"{text[:100]}_{target_length}_{focus}"
            if cache_key in self._cache:
                logger.info(f"Cache hit for summarization (user_id={user_id})")
                cached_result = self._cache[cache_key]
                elapsed_ms = int((time.time() - start_time) * 1000)
                cached_result["processing_time_ms"] = elapsed_ms
                return cached_result

            # Build summarization prompt based on focus
            summary_prompt = self._build_prompt(cleaned_text, target_length, focus, original_word_count)

            # Generate summary using Cohere Chat API (migrated from Generate API)
            try:
                response = self.cohere_client.chat(
                    message=summary_prompt,
                    max_tokens=self._calculate_max_tokens(original_word_count, target_length),
                    temperature=0.3,  # Low temperature for consistent, factual summaries
                    stop_sequences=[]
                )

                summary = response.text.strip()

            except Exception as e:
                logger.error(f"Cohere API error: {e}")
                raise SummarizationError(
                    "Summarization service unavailable. Please try again later.",
                    error_code="API_ERROR",
                    original_exception=e
                )

            # Calculate metrics
            summary_word_count = count_words(summary)
            compression_ratio = summary_word_count / original_word_count if original_word_count > 0 else 0

            # Extract key points (optional)
            key_points = self._extract_key_points(summary) if len(summary) > 100 else None

            result = {
                "summary": summary,
                "original_word_count": original_word_count,
                "summary_word_count": summary_word_count,
                "compression_ratio": round(compression_ratio, 2),
                "processing_time_ms": int((time.time() - start_time) * 1000),
                "key_points": key_points,
            }

            # Cache the result
            self._cache[cache_key] = result.copy()

            logger.info(
                f"Summarization completed: {original_word_count} words → {summary_word_count} words "
                f"({compression_ratio:.0%} compression) in {result['processing_time_ms']}ms "
                f"(user_id={user_id})"
            )

            return result

        except SummarizationError:
            raise
        except Exception as e:
            logger.error(f"Unexpected summarization error: {e}")
            raise SummarizationError(
                "An unexpected error occurred during summarization",
                error_code="UNEXPECTED_ERROR",
                original_exception=e
            )

    def _build_prompt(
        self,
        text: str,
        target_length: SummaryLength,
        focus: SummaryFocus,
        word_count: int
    ) -> str:
        """
        Build the summarization prompt based on parameters.

        Args:
            text: Text to summarize
            target_length: Target summary length
            focus: Summary focus area
            word_count: Original text word count

        Returns:
            Formatted prompt for Cohere
        """
        # Calculate target word count based on length
        length_ratios = {
            SummaryLength.SHORT: 0.20,   # 20% of original
            SummaryLength.MEDIUM: 0.30,  # 30% of original
            SummaryLength.LONG: 0.40,    # 40% of original
        }
        target_words = int(word_count * length_ratios[target_length])

        # Build focus instructions
        focus_instructions = {
            SummaryFocus.CONCEPTS: "Focus on conceptual explanations, definitions, and theoretical aspects. Omit code examples.",
            SummaryFocus.CODE: "Include important code snippets and implementation details. Explain what the code does.",
            SummaryFocus.BOTH: "Balance conceptual explanations with relevant code examples where applicable."
        }

        prompt = f"""Summarize the following technical text. {focus_instructions[focus]}

Requirements:
- Keep the summary concise (approximately {target_words} words)
- Maintain technical accuracy
- Preserve important terminology
- Write in clear, accessible language
- Use bullet points for key concepts if appropriate

Text to summarize:
{text}

Summary:"""

        return prompt

    def _calculate_max_tokens(self, original_word_count: int, target_length: SummaryLength) -> int:
        """
        Calculate maximum tokens for Cohere based on target summary length.

        Args:
            original_word_count: Word count of original text
            target_length: Target summary length

        Returns:
            Maximum number of tokens
        """
        length_ratios = {
            SummaryLength.SHORT: 0.20,
            SummaryLength.MEDIUM: 0.30,
            SummaryLength.LONG: 0.40,
        }

        # Rough conversion: 1 word ≈ 1.3 tokens
        target_words = int(original_word_count * length_ratios[target_length])
        max_tokens = int(target_words * 1.5)  # Add 50% buffer

        # Clamp between reasonable limits
        return max(100, min(max_tokens, 2000))

    def _extract_key_points(self, summary: str) -> Optional[List[str]]:
        """
        Extract key points from summary (simple bullet point detection).

        Args:
            summary: Generated summary text

        Returns:
            List of key points if found, None otherwise
        """
        lines = summary.split('\n')
        key_points = []

        for line in lines:
            line = line.strip()
            # Detect bullet points (-, *, •, numbers)
            if line and (
                line.startswith('-') or
                line.startswith('*') or
                line.startswith('•') or
                (len(line) > 2 and line[0].isdigit() and line[1] in '.)')
            ):
                # Remove bullet marker
                point = line.lstrip('-*•0123456789.) ').strip()
                if point:
                    key_points.append(point)

        return key_points if key_points else None

    def clear_cache(self) -> None:
        """Clear the summarization cache."""
        self._cache.clear()
        logger.info("Summarization cache cleared")

    def get_cache_size(self) -> int:
        """Get the current cache size."""
        return len(self._cache)
