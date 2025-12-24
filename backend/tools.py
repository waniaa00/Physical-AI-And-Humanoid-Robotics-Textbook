"""
Retrieval tools for OpenAI Agent
Provides book content retrieval functionality with [Source N] citation formatting
"""
import httpx
from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)

# Search API endpoint - Use environment variable or default to HF Spaces
import os
SEARCH_API_URL = os.getenv("SEARCH_API_URL", "https://wnxddev-humanoid-robotics-api.hf.space/search")


def format_retrieval_results(search_results: List[Dict[str, Any]]) -> str:
    """
    Format search results with [Source N] markers for citation

    Args:
        search_results: List of search result dictionaries from /search endpoint

    Returns:
        Formatted string with numbered sources and URLs
    """
    if not search_results:
        return "No relevant content found in the book."

    formatted_parts = [
        "Retrieved content from the book:",
        "",
        "INSTRUCTIONS FOR ASSISTANT: You MUST cite these sources using [Source 1], [Source 2], etc. in your response. Include a 'Sources:' section at the end listing all URLs.",
        ""
    ]

    for idx, result in enumerate(search_results, start=1):
        text = result.get('text', '')
        metadata = result.get('metadata', {})
        url = metadata.get('url', 'Unknown URL')
        score = result.get('score', 0.0)

        # Format each chunk with source marker
        source_text = f"[Source {idx}] {text}\nURL: {url}\nRelevance Score: {score:.2f}\n"
        formatted_parts.append(source_text)

    return "\n".join(formatted_parts)


def handle_retrieval_error(error: Exception) -> str:
    """
    Handle retrieval errors gracefully
    Returns user-friendly error message
    """
    logger.error(f"Retrieval error: {str(error)}")
    return f"Failed to retrieve book content. The search service may be temporarily unavailable. Error: {str(error)}"


async def retrieve_book_content(
    query: str,
    top_k: int = 5,
    filters: Dict[str, str] = None
) -> str:
    """
    Retrieve relevant book content from the vector database

    This function calls the /search endpoint to find semantically similar content
    and formats results with [Source N] citation markers.

    Args:
        query: Search query string
        top_k: Number of results to retrieve (1-20, default 5)
        filters: Optional metadata filters (url_contains, url_exact, source, chunk_index)

    Returns:
        Formatted string with retrieved content and source citations

    Example:
        >>> content = await retrieve_book_content("What is inverse kinematics?", top_k=3)
        >>> print(content)
        [Source 1] Inverse kinematics is the process...
        URL: https://example.com/chapter1
        Relevance Score: 0.87

        [Source 2] ...
    """
    try:
        # Prepare request payload
        payload = {
            "query": query,
            "top_k": top_k
        }
        if filters:
            payload["filters"] = filters

        # Call search API
        async with httpx.AsyncClient(timeout=30.0) as client:
            response = await client.post(SEARCH_API_URL, json=payload)
            response.raise_for_status()

            data = response.json()
            results = data.get('results', [])

            # Format results with citations
            formatted_content = format_retrieval_results(results)

            logger.info(f"Retrieved {len(results)} results for query: {query[:50]}...")
            return formatted_content

    except httpx.HTTPError as e:
        return handle_retrieval_error(e)
    except Exception as e:
        return handle_retrieval_error(e)


# For OpenAI Agents SDK integration (will be used in agent_config.py)
# The function above can be wrapped with @function_tool decorator
