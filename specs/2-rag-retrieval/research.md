# Research: RAG Retrieval Pipeline

**Feature**: 2-rag-retrieval
**Date**: 2025-12-15
**Purpose**: Technical investigation for implementing semantic retrieval over Qdrant vector store

## Executive Summary

The RAG retrieval pipeline will extend the existing vector ingestion backend (`backend/main.py`) with query-time semantic search capabilities. The existing infrastructure provides:
- FastAPI application framework
- Qdrant Cloud connection and client initialization
- Cohere API integration for embeddings (embed-english-v3.0, 1024 dimensions)
- Environment-based configuration (.env files)
- Logging infrastructure

This research validates the technical approach for adding retrieval endpoints and query processing to the existing backend.

## Existing Infrastructure Analysis

### Current Backend Architecture

**Location**: `backend/main.py`
**Framework**: FastAPI with uvicorn ASGI server
**Dependencies** (from `backend/pyproject.toml`):
- `fastapi>=0.104.0` - Web framework
- `qdrant-client>=1.8.0` - Vector database client
- `cohere>=5.5.0` - Embedding generation
- `python-dotenv>=1.0.0` - Configuration management
- `aiohttp>=3.9.0` - Async HTTP for web crawling (ingestion)
- `beautifulsoup4>=4.12.0`, `lxml>=4.9.0` - HTML/XML parsing (ingestion)
- `uvicorn>=0.24.0` - ASGI server

### Current Configuration

**Environment Variables** (from `backend/.env.example`):
```env
COHERE_API_KEY=your-cohere-api-key-here
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_URL=your-qdrant-cluster-url-here
```

**Current Setup** (from `backend/main.py`):
```python
CO_API_KEY = os.getenv("CO_API_KEY")  # Note: Variable name inconsistency
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL", "https://your-cluster-url.qdrant.tech")
COLLECTION_NAME = "humanoid-robotics-embeddings"

co = cohere.Client(CO_API_KEY)
qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
```

**Configuration Issue Identified**: Environment variable name mismatch
- `.env.example` uses `COHERE_API_KEY`
- `main.py` reads `CO_API_KEY`
- **Resolution**: Use consistent naming (`COHERE_API_KEY` in both)

### Existing Endpoints

1. **POST /ingest** - Vector ingestion pipeline (existing)
2. **GET /verify** - Collection verification (existing)
3. **POST /search** - Semantic search endpoint (to be added)
4. **POST /test-retrieval** - Quality validation endpoint (to be added)

## Research Questions & Findings

### RQ1: Qdrant Search API Capabilities

**Question**: What search methods and filtering options does Qdrant support for semantic retrieval?

**Investigation**:
- Reviewed Qdrant Python client documentation and API
- Examined existing ingestion code for collection structure

**Findings**:
Qdrant provides `search()` method with comprehensive capabilities:

```python
results = qdrant_client.search(
    collection_name="humanoid-robotics-embeddings",
    query_vector=[...],  # 1024-dim embedding from Cohere
    limit=5,  # top-k results
    with_payload=True,  # Include metadata and text
    with_vectors=False,  # Don't return vectors (saves bandwidth)
    query_filter=Filter(...)  # Optional metadata filtering
)
```

**Search Result Structure**:
```python
ScoredPoint(
    id=uuid,
    score=float,  # Similarity score (cosine distance)
    payload={
        "text": "chunk content",
        "metadata": {
            "url": "https://...",
            "chunk_index": int,
            "source": "website"
        }
    }
)
```

**Metadata Filtering** (Qdrant Filter DSL):
```python
from qdrant_client.http.models import Filter, FieldCondition, MatchValue

# Filter by URL (exact match)
Filter(
    must=[
        FieldCondition(
            key="metadata.url",
            match=MatchValue(value="https://...chapter3...")
        )
    ]
)

# Filter by URL pattern (contains)
Filter(
    must=[
        FieldCondition(
            key="metadata.url",
            match=MatchValue(value="module1")  # Partial match
        )
    ]
)
```

**Decision**: Use `search()` method with optional `query_filter` parameter for metadata-based filtering.

### RQ2: Query Embedding Generation

**Question**: How should user queries be converted to embeddings for vector search?

**Investigation**:
- Reviewed existing `embed()` function in `backend/main.py`
- Checked Cohere API documentation for query embedding

**Findings**:
Existing embedding function uses `input_type="search_document"` for document indexing:

```python
def embed(texts: List[str]) -> List[List[float]]:
    response = co.embed(
        texts=texts,
        model="embed-english-v3.0",
        input_type="search_document"  # For documents
    )
    return response.embeddings
```

For query-time embeddings, Cohere recommends `input_type="search_query"`:

```python
def embed_query(query: str) -> List[float]:
    response = co.embed(
        texts=[query],
        model="embed-english-v3.0",
        input_type="search_query"  # For queries (optimized)
    )
    return response.embeddings[0]
```

**Key Difference**:
- `search_document`: Optimized for indexing content (captures document semantics)
- `search_query`: Optimized for user queries (captures search intent)
- Both produce 1024-dim vectors compatible with same vector space

**Decision**: Create separate `embed_query()` function using `input_type="search_query"` for optimal retrieval quality.

### RQ3: Performance Optimization

**Question**: What strategies can ensure <2 second retrieval latency?

**Investigation**:
- Analyzed Qdrant search performance characteristics
- Reviewed Cohere API latency benchmarks

**Findings**:

**Latency Breakdown**:
1. Query embedding generation: ~200-500ms (Cohere API call)
2. Vector search: ~50-200ms (Qdrant, depends on collection size)
3. Response formatting: ~10-50ms (Python processing)
4. **Total estimated**: 260-750ms (well under 2s target)

**Optimization Strategies**:

1. **Concurrent Requests**: Use async/await for independent operations
   ```python
   # Not needed for single query, but useful for batch queries
   async with asyncio.TaskGroup() as tg:
       embeddings = tg.create_task(embed_query_async(query))
       # Other parallel tasks if needed
   ```

2. **Connection Pooling**: Qdrant client reuses connections automatically
   - Current setup: Single `qdrant_client` instance (✓ correct)
   - Cohere client: Single `co` instance (✓ correct)

3. **Payload Optimization**: Don't return vectors in search results
   ```python
   with_vectors=False  # Saves ~4KB per result (1024 floats × 4 bytes)
   ```

4. **Limit Result Size**: Default top_k=5 keeps response small
   - 5 chunks × ~500 chars ≈ 2.5KB response body
   - vs. top_k=20 ≈ 10KB response body

**Decision**: No special caching needed initially. Standard async FastAPI + connection pooling sufficient for <2s target.

### RQ4: Error Handling & Edge Cases

**Question**: How should the system handle retrieval failures and edge cases?

**Investigation**:
- Reviewed Qdrant error types
- Examined Cohere API error responses
- Analyzed edge case scenarios from spec

**Findings**:

**Qdrant Error Types**:
- `qdrant_client.http.exceptions.UnexpectedResponse` - API errors
- Network timeouts - Connection failures
- Empty results - Valid case (no similar content)

**Cohere Error Types**:
- `cohere.error.CohereAPIError` - API errors (rate limits, auth)
- `cohere.error.CohereConnectionError` - Network issues

**Edge Case Handling**:

1. **No semantically similar content** (score < threshold)
   ```python
   if not results or results[0].score < 0.3:  # Low similarity
       return {"results": [], "message": "No relevant content found"}
   ```

2. **top_k exceeds collection size**
   ```python
   # Qdrant handles gracefully: returns all available points
   # No special handling needed
   ```

3. **Query embedding failure**
   ```python
   try:
       query_vector = embed_query(query)
   except cohere.error.CohereAPIError as e:
       raise HTTPException(status_code=503, detail="Embedding service unavailable")
   ```

4. **Qdrant connection failure**
   ```python
   try:
       results = qdrant_client.search(...)
   except UnexpectedResponse as e:
       raise HTTPException(status_code=503, detail="Search service unavailable")
   ```

5. **Invalid metadata filter**
   ```python
   # Validate filter structure before passing to Qdrant
   # Invalid filter → return 400 Bad Request
   ```

**Decision**: Implement try-except blocks for both Cohere and Qdrant operations, with appropriate HTTP status codes (503 for service errors, 400 for invalid input).

### RQ5: Test Query Design

**Question**: What test queries effectively validate retrieval accuracy?

**Investigation**:
- Reviewed book content structure from ingestion logs
- Identified key topics covered in the book

**Findings**:

**Book Content Coverage** (from ingestion):
- Module 1: Introduction, ROS2, Kinematics, Dynamics
- Module 2: Gazebo, Isaac Sim, Computer Vision, SLAM
- Module 3: Bipedal Locomotion, Manipulation, Whole-Body Control, HRI
- Module 4: VLA Models, LLM Planning, Integration, Deployment

**Proposed Test Queries** (20 queries covering major topics):

**Category 1: Fundamentals (5 queries)**
1. "What is inverse kinematics?"
2. "How does ROS2 message passing work?"
3. "Explain forward kinematics for humanoid robots"
4. "What are the main components of a humanoid robot?"
5. "How do joint angles relate to end effector position?"

**Category 2: Simulation & Perception (5 queries)**
6. "How to simulate humanoid robots in Gazebo?"
7. "What is Isaac Sim used for in robotics?"
8. "How does computer vision work for humanoid robots?"
9. "Explain SLAM for robot navigation"
10. "What sensors do humanoid robots use for perception?"

**Category 3: Control & Locomotion (5 queries)**
11. "How do bipedal robots maintain balance while walking?"
12. "What is whole-body control for humanoid robots?"
13. "Explain manipulation tasks for humanoid robots"
14. "How does a humanoid robot grasp objects?"
15. "What control strategies are used for stable walking?"

**Category 4: AI & Integration (5 queries)**
16. "What are Vision-Language-Action models?"
17. "How do VLA models work for robotics?"
18. "Explain LLM-based planning for robots"
19. "How to deploy humanoid robots in real environments?"
20. "What is human-robot interaction?"

**Expected Results**:
- Each query should return chunks from relevant chapters
- E.g., Query 1 → chunks from "chapter3-kinematics"
- E.g., Query 11 → chunks from "chapter9-bipedal-locomotion"

**Success Metric**: 17/20 queries (85%) return expected content in top 5 results

**Decision**: Implement test endpoint with these 20 queries and validation logic comparing returned chunk URLs against expected chapter patterns.

## Technology Stack Validation

### Confirmed Stack

| Component | Technology | Version | Justification |
|-----------|-----------|---------|---------------|
| Language | Python | 3.13+ | Existing backend standard |
| Web Framework | FastAPI | 0.104.0+ | Async support, OpenAPI docs, existing |
| Vector DB | Qdrant Cloud | N/A (cloud) | Existing collection with 403+ embeddings |
| Vector DB Client | qdrant-client | 1.8.0+ | Official Python client, existing |
| Embeddings | Cohere API | N/A (cloud) | embed-english-v3.0 model, existing |
| Embedding Client | cohere | 5.5.0+ | Official Python SDK, existing |
| Server | uvicorn | 0.24.0+ | ASGI server for FastAPI, existing |
| Config | python-dotenv | 1.0.0+ | Environment variable management, existing |

**No new dependencies required** - all components already in place.

### Architecture Pattern

**Pattern**: Extend existing monolithic FastAPI application
- **Pros**: Simple, consistent with ingestion pipeline, shared clients/config
- **Cons**: Ingestion and retrieval in same process (acceptable for prototype)

**Alternative Considered**: Separate retrieval service
- **Rejected**: Over-engineering for current scale (<1000 queries/day expected)
- **Future**: Can split later if needed

## API Design Considerations

### Endpoint Structure

**Option 1: RESTful Endpoints**
```
POST /search              # Main search endpoint
POST /test-retrieval      # Quality validation
GET  /collections/info    # Collection stats
```

**Option 2: Single Parameterized Endpoint**
```
POST /retrieve?mode=search
POST /retrieve?mode=test
```

**Decision**: Option 1 (RESTful) - clearer intent, better documentation

### Request/Response Models

**Search Request**:
```python
{
    "query": "string",
    "top_k": 5,  # optional, default 5
    "filters": {  # optional
        "url_contains": "module1",
        "source": "website"
    }
}
```

**Search Response**:
```python
{
    "query": "string",
    "results": [
        {
            "text": "chunk content",
            "score": 0.87,
            "metadata": {
                "url": "https://...",
                "chunk_index": 3,
                "source": "website"
            }
        }
    ],
    "total_results": 5,
    "latency_ms": 456
}
```

## Performance Benchmarks

### Expected Metrics

Based on research and Qdrant/Cohere documentation:

| Metric | Target | Expected | Notes |
|--------|--------|----------|-------|
| Single query latency | <2s | 300-800ms | Cohere + Qdrant + processing |
| Batch 10 queries latency | <3s avg | 500-1200ms | With async parallelization |
| Collection size | N/A | 400-500 chunks | Current ingestion result |
| Vector dimensionality | 1024 | 1024 | Cohere embed-english-v3.0 |
| Relevance rate | 85% | TBD | Will measure with test suite |

### Scalability Considerations

**Current Scale**: ~400 embeddings, <10 queries/day (development)
**Production Scale**: ~1000 embeddings, ~100 queries/day (estimated)

**Bottlenecks**:
1. **Cohere API rate limits**: 10,000 requests/month (free tier) or unlimited (paid)
   - Current usage: <1,000/month (ingestion + retrieval)
   - **Status**: No concern
2. **Qdrant Cloud limits**: Varies by plan
   - **Action**: Verify plan supports expected QPS
3. **FastAPI concurrency**: Handles 1000+ req/s on modern hardware
   - **Status**: No concern for expected load

## Security Considerations

### API Key Management

**Current Approach**: Environment variables via `.env` file
- ✓ Keys not in version control (.env in .gitignore)
- ✓ Separate config for dev/staging/prod
- ✗ No key rotation mechanism

**Recommendations**:
1. Document key rotation process in operational guide
2. Consider secrets manager for production (AWS Secrets Manager, HashiCorp Vault)
3. Implement API key authentication for retrieval endpoints (future)

### Input Validation

**Query Sanitization**:
- Max query length: 1000 characters (prevent abuse)
- top_k bounds: 1-20 (prevent large responses)
- Metadata filter validation: Check structure before passing to Qdrant

**Rate Limiting** (future consideration):
- Not needed initially (<10 req/day)
- Consider adding if deployed publicly

## Implementation Risks

### Risk Matrix

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Cohere API downtime | Low | High | Implement retry logic, cache recent embeddings |
| Qdrant connection failure | Low | High | Health check endpoint, error handling |
| Low retrieval relevance | Medium | Medium | Test suite validation, tuning similarity thresholds |
| Config variable mismatch | Medium | Low | Standardize naming (COHERE_API_KEY) |
| Slow query performance | Low | Medium | Monitor latency, optimize if needed |

## Research Conclusions

### Technical Feasibility: ✅ CONFIRMED

1. **Infrastructure**: Existing backend fully supports retrieval with no new dependencies
2. **Performance**: Expected latency (300-800ms) well under 2s target
3. **Accuracy**: Cohere embed-english-v3.0 optimized for semantic search with `input_type="search_query"`
4. **Scalability**: Current architecture handles expected load (<100 queries/day)

### Key Implementation Decisions

1. **Extend `backend/main.py`** with retrieval endpoints (no separate service)
2. **Create `embed_query()` function** using `input_type="search_query"` (separate from `embed()`)
3. **Use Qdrant `search()` method** with optional `query_filter` for metadata filtering
4. **Implement 20-query test suite** for validation (85% relevance target)
5. **Fix configuration naming** (`CO_API_KEY` → `COHERE_API_KEY` consistency)
6. **Add comprehensive error handling** for Cohere and Qdrant failures

### Unknowns Requiring Implementation Testing

1. **Actual retrieval relevance**: Need to run test suite against real data
2. **Similarity score thresholds**: What score indicates "relevant" vs "not relevant"?
3. **Optimal top_k default**: Is 5 best, or should we use 3 or 7?
4. **Metadata filter patterns**: Do users prefer exact match or contains?

### Ready for Design Phase

All technical questions resolved. Proceeding to:
- Data model design (request/response schemas)
- API contract specification (OpenAPI)
- Implementation task breakdown
