# Data Model: RAG Retrieval Pipeline

**Feature**: 2-rag-retrieval
**Date**: 2025-12-15
**Purpose**: Define request/response data structures and validation rules

## Overview

This document specifies the data models for the RAG retrieval pipeline, including request schemas, response schemas, and internal data structures. All models use Pydantic for validation and FastAPI integration.

## Request Models

### SearchRequest

Primary model for semantic search queries.

```python
from pydantic import BaseModel, Field, field_validator
from typing import Optional, Dict, Any

class SearchRequest(BaseModel):
    """
    Request model for semantic search endpoint.

    Validates user query parameters and optional filters.
    """

    query: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="Natural language query text",
        examples=["What is inverse kinematics?"]
    )

    top_k: int = Field(
        default=5,
        ge=1,
        le=20,
        description="Number of results to return (1-20)",
        examples=[5, 10]
    )

    filters: Optional[Dict[str, str]] = Field(
        default=None,
        description="Optional metadata filters",
        examples=[
            {"url_contains": "module1"},
            {"source": "website"}
        ]
    )

    @field_validator("query")
    @classmethod
    def query_not_empty(cls, v: str) -> str:
        """Ensure query is not just whitespace."""
        if not v.strip():
            raise ValueError("Query cannot be empty or whitespace only")
        return v.strip()

    @field_validator("filters")
    @classmethod
    def validate_filters(cls, v: Optional[Dict[str, str]]) -> Optional[Dict[str, str]]:
        """Validate filter structure and allowed keys."""
        if v is None:
            return v

        allowed_keys = {"url_contains", "url_exact", "source", "chunk_index"}
        invalid_keys = set(v.keys()) - allowed_keys

        if invalid_keys:
            raise ValueError(
                f"Invalid filter keys: {invalid_keys}. "
                f"Allowed keys: {allowed_keys}"
            )

        return v

    class Config:
        json_schema_extra = {
            "example": {
                "query": "How do bipedal robots maintain balance?",
                "top_k": 5,
                "filters": {
                    "url_contains": "locomotion"
                }
            }
        }
```

**Validation Rules**:
- `query`: 1-1000 characters, non-empty after stripping whitespace
- `top_k`: Integer between 1 and 20 (inclusive)
- `filters`: Optional dict with allowed keys only

**Filter Options**:
- `url_contains`: Substring match in URL field
- `url_exact`: Exact URL match
- `source`: Exact source match (e.g., "website")
- `chunk_index`: Specific chunk index (rarely used)

### TestRetrievalRequest

Model for running test suite validation.

```python
class TestRetrievalRequest(BaseModel):
    """
    Request model for test retrieval endpoint.

    Runs predefined test queries and validates results.
    """

    query_ids: Optional[list[int]] = Field(
        default=None,
        description="Specific test query IDs to run (None = all)",
        examples=[[1, 2, 3], None]
    )

    top_k: int = Field(
        default=5,
        ge=1,
        le=20,
        description="Number of results to return per query",
        examples=[5]
    )

    @field_validator("query_ids")
    @classmethod
    def validate_query_ids(cls, v: Optional[list[int]]) -> Optional[list[int]]:
        """Ensure query IDs are valid (1-20)."""
        if v is None:
            return v

        if not all(1 <= qid <= 20 for qid in v):
            raise ValueError("Query IDs must be between 1 and 20")

        return v

    class Config:
        json_schema_extra = {
            "example": {
                "query_ids": [1, 2, 3, 11],
                "top_k": 5
            }
        }
```

## Response Models

### RetrievedChunk

Model for a single search result.

```python
class ChunkMetadata(BaseModel):
    """Metadata associated with a retrieved chunk."""

    url: str = Field(
        ...,
        description="Source URL of the chunk",
        examples=["https://physical-ai-and-humanoid-robotics-t-two.vercel.app/docs/module1/chapter3-kinematics"]
    )

    chunk_index: int = Field(
        ...,
        ge=0,
        description="Index of chunk within source document",
        examples=[0, 5, 12]
    )

    source: str = Field(
        ...,
        description="Source type (e.g., 'website')",
        examples=["website"]
    )


class RetrievedChunk(BaseModel):
    """
    Single search result containing text, score, and metadata.

    Represents one relevant chunk retrieved from vector database.
    """

    text: str = Field(
        ...,
        description="Chunk text content",
        examples=["Inverse kinematics (IK) is the process of determining joint angles..."]
    )

    score: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Similarity score (0.0-1.0, higher is more relevant)",
        examples=[0.87, 0.75, 0.62]
    )

    metadata: ChunkMetadata = Field(
        ...,
        description="Associated metadata for the chunk"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "text": "Inverse kinematics (IK) is the process of determining joint angles...",
                "score": 0.87,
                "metadata": {
                    "url": "https://physical-ai-and-humanoid-robotics-t-two.vercel.app/docs/module1/chapter3-kinematics",
                    "chunk_index": 5,
                    "source": "website"
                }
            }
        }
```

### SearchResponse

Primary response model for search endpoint.

```python
class SearchResponse(BaseModel):
    """
    Response model for semantic search endpoint.

    Contains search results, metadata, and performance metrics.
    """

    query: str = Field(
        ...,
        description="Original query text",
        examples=["What is inverse kinematics?"]
    )

    results: list[RetrievedChunk] = Field(
        ...,
        description="List of retrieved chunks, ordered by relevance",
        examples=[[]]
    )

    total_results: int = Field(
        ...,
        ge=0,
        description="Number of results returned",
        examples=[5, 0]
    )

    latency_ms: int = Field(
        ...,
        ge=0,
        description="Query processing time in milliseconds",
        examples=[456, 789]
    )

    filters_applied: Optional[Dict[str, str]] = Field(
        default=None,
        description="Metadata filters that were applied",
        examples=[{"url_contains": "module1"}]
    )

    message: Optional[str] = Field(
        default=None,
        description="Optional message (e.g., for empty results)",
        examples=["No relevant content found for this query"]
    )

    class Config:
        json_schema_extra = {
            "example": {
                "query": "How do bipedal robots maintain balance?",
                "results": [
                    {
                        "text": "Bipedal locomotion requires maintaining the center of mass...",
                        "score": 0.89,
                        "metadata": {
                            "url": "https://...chapter9-bipedal-locomotion",
                            "chunk_index": 3,
                            "source": "website"
                        }
                    }
                ],
                "total_results": 5,
                "latency_ms": 567,
                "filters_applied": None,
                "message": None
            }
        }
```

### TestResult

Model for single test case result.

```python
class TestResult(BaseModel):
    """Result of a single test query."""

    query_id: int = Field(
        ...,
        description="Test query ID (1-20)",
        examples=[1, 11]
    )

    query_text: str = Field(
        ...,
        description="Test query text",
        examples=["What is inverse kinematics?"]
    )

    expected_chapter: str = Field(
        ...,
        description="Expected chapter pattern in results",
        examples=["chapter3-kinematics", "chapter9-bipedal-locomotion"]
    )

    found_in_top_k: bool = Field(
        ...,
        description="Whether expected content was found in top-k results",
        examples=[True, False]
    )

    top_result_url: str = Field(
        ...,
        description="URL of top result",
        examples=["https://...chapter3-kinematics"]
    )

    top_result_score: float = Field(
        ...,
        description="Score of top result",
        examples=[0.87]
    )


class TestRetrievalResponse(BaseModel):
    """
    Response model for test retrieval endpoint.

    Contains test suite results and aggregate metrics.
    """

    total_queries: int = Field(
        ...,
        description="Total number of test queries run",
        examples=[20, 4]
    )

    successful_queries: int = Field(
        ...,
        description="Number of queries that returned expected content",
        examples=[17, 3]
    )

    success_rate: float = Field(
        ...,
        ge=0.0,
        le=1.0,
        description="Success rate (successful/total)",
        examples=[0.85, 0.75]
    )

    meets_target: bool = Field(
        ...,
        description="Whether success rate meets 85% target",
        examples=[True, False]
    )

    results: list[TestResult] = Field(
        ...,
        description="Detailed results for each test query"
    )

    avg_latency_ms: int = Field(
        ...,
        description="Average latency per query in milliseconds",
        examples=[523]
    )

    class Config:
        json_schema_extra = {
            "example": {
                "total_queries": 20,
                "successful_queries": 17,
                "success_rate": 0.85,
                "meets_target": True,
                "results": [],
                "avg_latency_ms": 523
            }
        }
```

## Internal Data Structures

### TestQuery

Internal model for test case definitions.

```python
class TestQuery(BaseModel):
    """
    Internal model for test query definition.

    Not exposed in API, used for test suite validation.
    """

    id: int = Field(..., description="Query ID (1-20)")
    query: str = Field(..., description="Query text")
    expected_chapter_pattern: str = Field(
        ...,
        description="Regex pattern or substring for expected chapter URL",
        examples=["chapter3-kinematics", "module1/chapter"]
    )
    category: str = Field(
        ...,
        description="Query category",
        examples=["Fundamentals", "Simulation", "Control", "AI"]
    )
```

**Test Suite Definition** (constant in code):

```python
TEST_QUERIES = [
    TestQuery(id=1, query="What is inverse kinematics?",
              expected_chapter_pattern="chapter3-kinematics", category="Fundamentals"),
    TestQuery(id=2, query="How does ROS2 message passing work?",
              expected_chapter_pattern="chapter2-ros2", category="Fundamentals"),
    # ... (18 more queries)
]
```

### QueryEmbeddingCache (Future Enhancement)

```python
from datetime import datetime
from typing import List

class CachedEmbedding(BaseModel):
    """
    Cache entry for query embeddings (future optimization).

    NOT IMPLEMENTED IN INITIAL VERSION.
    """

    query: str
    embedding: List[float]
    timestamp: datetime
    hit_count: int = 0
```

## Data Flow

### Search Request Flow

```
1. Client sends SearchRequest
   ↓
2. FastAPI validates request (Pydantic)
   ↓
3. Backend generates query embedding (Cohere API)
   ↓
4. Backend performs vector search (Qdrant)
   ↓
5. Backend formats results as SearchResponse
   ↓
6. FastAPI serializes and returns JSON
```

### Test Request Flow

```
1. Client sends TestRetrievalRequest
   ↓
2. FastAPI validates request
   ↓
3. Backend loads test queries from TEST_QUERIES
   ↓
4. For each query:
   - Generate embedding
   - Search Qdrant
   - Validate results against expected_chapter_pattern
   ↓
5. Backend aggregates results into TestRetrievalResponse
   ↓
6. FastAPI returns JSON with success metrics
```

## Validation Rules Summary

| Field | Type | Constraints | Default |
|-------|------|-------------|---------|
| `query` | str | 1-1000 chars, non-empty | required |
| `top_k` | int | 1-20 | 5 |
| `filters` | dict | Allowed keys only | None |
| `query_ids` | list[int] | Values 1-20 | None (all) |
| `score` | float | 0.0-1.0 | N/A (output) |
| `chunk_index` | int | ≥0 | N/A (output) |

## Error Responses

### ValidationError (422)

```python
{
    "detail": [
        {
            "type": "value_error",
            "loc": ["body", "top_k"],
            "msg": "Input should be less than or equal to 20",
            "input": 50
        }
    ]
}
```

### ServiceError (503)

```python
{
    "detail": "Embedding service unavailable"
}
```

## OpenAPI Schema Generation

All models include:
- Field descriptions for auto-generated documentation
- Example values for API testing
- Validation constraints (min/max, regex patterns)
- Proper JSON serialization

FastAPI automatically generates:
- `/docs` - Swagger UI with examples
- `/redoc` - ReDoc documentation
- `/openapi.json` - OpenAPI 3.0 schema

## Database Schema (Qdrant Collection)

### Existing Collection Structure

**Collection Name**: `humanoid-robotics-embeddings`

**Vector Config**:
```python
{
    "size": 1024,  # Cohere embed-english-v3.0 dimensions
    "distance": "Cosine"  # Similarity metric
}
```

**Point Structure**:
```python
{
    "id": "uuid-string",  # Unique identifier
    "vector": [1024 floats],  # Embedding
    "payload": {
        "text": "chunk content string",
        "metadata": {
            "url": "https://...",
            "chunk_index": 0,
            "source": "website"
        }
    }
}
```

**No schema changes required** - retrieval uses existing structure.

## Type Aliases

```python
from typing import TypeAlias

# Query embedding vector (1024 dimensions)
QueryVector: TypeAlias = list[float]

# Qdrant search result
QdrantResult: TypeAlias = "qdrant_client.http.models.ScoredPoint"

# Filter dictionary
FilterDict: TypeAlias = Dict[str, str]
```

## Summary

This data model provides:
- ✅ Strong type safety with Pydantic validation
- ✅ Clear request/response contracts
- ✅ Comprehensive examples for API documentation
- ✅ Internal structures for test suite
- ✅ Validation rules preventing invalid inputs
- ✅ Error handling schemas

**No database migrations needed** - uses existing Qdrant collection structure.

**Dependencies**: Pydantic v2+ (included in FastAPI)
