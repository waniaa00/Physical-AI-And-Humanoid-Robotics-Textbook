"""
Retrieval tools for OpenAI Agent
Provides book content retrieval functionality with [Source N] citation formatting
"""
from typing import List, Dict, Any, Optional
import logging
import os

logger = logging.getLogger(__name__)

# Import search dependencies
from qdrant_client import QdrantClient
import cohere

# Initialize clients using environment variables
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL", "https://your-cluster-url.qdrant.tech")
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "humanoid-robotics-embeddings")

co = cohere.Client(COHERE_API_KEY)
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    timeout=30.0
)


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


def embed_query(query: str) -> List[float]:
    """
    Generate embedding for a search query using Cohere
    Returns 1024-dimensional vector
    """
    try:
        response = co.embed(
            texts=[query],
            model='embed-english-v3.0',
            input_type='search_query'
        )
        return response.embeddings[0]
    except Exception as e:
        logger.error(f"Cohere embedding error: {str(e)}")
        raise


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

    This function performs semantic search directly using Cohere + Qdrant
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
        # Step 1: Generate query embedding
        logger.info(f"Retrieving content for query: {query[:50]}...")
        query_vector = embed_query(query)

        # Step 2: Search Qdrant
        search_response = qdrant_client.query_points(
            collection_name=COLLECTION_NAME,
            query=query_vector,
            limit=top_k
        )

        # query_points returns QueryResponse with .points attribute
        search_results = search_response.points if hasattr(search_response, 'points') else search_response
        logger.info(f"Found {len(search_results)} results from Qdrant")

        # Step 3: Format results for agent consumption
        results_for_formatting = []
        for result in search_results:
            metadata_dict = result.payload.get('metadata', {})
            results_for_formatting.append({
                'text': result.payload.get('text', ''),
                'score': result.score,
                'metadata': {
                    'url': metadata_dict.get('url', ''),
                    'chunk_index': metadata_dict.get('chunk_index', 0),
                    'source': metadata_dict.get('source', 'unknown')
                }
            })

        # Format results with citations
        formatted_content = format_retrieval_results(results_for_formatting)
        logger.info(f"Retrieved {len(results_for_formatting)} results for query: {query[:50]}...")
        return formatted_content

    except Exception as e:
        return handle_retrieval_error(e)


# For OpenAI Agents SDK integration (will be used in agent_config.py)
# The function above can be wrapped with @function_tool decorator
