# Implementation Plan: RAG Retrieval Pipeline

**Branch**: `2-rag-retrieval` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/2-rag-retrieval/spec.md`

## Summary

Extend existing vector ingestion backend (`backend/main.py`) with semantic retrieval capabilities for humanoid robotics book content. Implementation adds three new FastAPI endpoints (`/search`, `/test-retrieval`, `/health`) that leverage existing Qdrant and Cohere infrastructure to provide query-time semantic search, quality validation, and service health monitoring.

**Core Functionality**:
- Convert user queries to embeddings via Cohere API (search_query input type)
- Perform vector similarity search against Qdrant collection "humanoid-robotics-embeddings"
- Support configurable top-k results (1-20, default 5)
- Enable metadata filtering (URL, source, chunk_index)
- Validate retrieval quality with 20-query test suite (85% success target)
- Monitor performance and service health

**Technical Approach** (from research):
- Reuse existing FastAPI app, Qdrant client, Cohere client
- Add new `embed_query()` function using `input_type="search_query"`
- Implement Pydantic models for request/response validation
- Use Qdrant `search()` method with optional `query_filter`
- No new dependencies required

## Technical Context

**Language/Version**: Python 3.13+
**Primary Dependencies**: FastAPI 0.104.0+, qdrant-client 1.8.0+, cohere 5.5.0+, python-dotenv 1.0.0+
**Storage**: Qdrant Cloud (collection: "humanoid-robotics-embeddings", 403+ vectors)
**Testing**: Manual testing via FastAPI `/docs`, automated test suite via `/test-retrieval` endpoint
**Target Platform**: Linux/Windows server with Python 3.13+
**Project Type**: Single (monolithic FastAPI application)
**Performance Goals**: <2s single query latency, <3s avg for 10 concurrent queries, 85% test suite success rate
**Constraints**: <2000ms retrieval latency (single query), Cohere API rate limits (10k+ requests/month), Qdrant Cloud service limits
**Scale/Scope**: ~400 embedded chunks, <100 queries/day expected, 3 new endpoints, ~500 LOC addition

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle V: AI-Native Authoring Workflow ✅

- ✅ Development follows Spec-Driven Development using SpecKit-Plus
- ✅ Planning preceded generation (spec → research → design → plan)
- ✅ All artifacts are human-auditable and editable (Markdown, Python, YAML)
- ✅ PHR will be created for this planning session

### Principle VI: Code & Simulation Standards ✅

- ✅ Python as supported language
- ✅ Code will include comments explaining logic
- ✅ No hardcoded secrets (using environment variables)
- ✅ Error handling demonstrated (try-except blocks for API calls)
- ✅ Minimal yet complete examples (test queries, API examples)

### Principle VII: Quality Gates & Validation ✅

- ✅ Code will be tested via FastAPI `/docs` and `/test-retrieval` endpoint
- ✅ Test suite provides validation (20 queries, 85% target)
- ✅ No unverifiable claims (all metrics measurable via test endpoint)
- ✅ Internal consistency enforced by Pydantic models

**Constitution Status**: PASSED - No violations, ready to proceed

## Project Structure

### Documentation (this feature)

```text
specs/2-rag-retrieval/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 research findings
├── data-model.md        # Phase 1 data models
├── contracts/
│   └── api-contract.yaml # Phase 1 API contract (OpenAPI 3.0)
├── quickstart.md        # Phase 1 quickstart guide
├── checklists/
│   └── requirements.md  # Specification quality checklist
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT YET CREATED)
```

### Source Code (repository root)

```text
backend/
├── main.py              # FastAPI app (MODIFIED - add retrieval endpoints)
├── pyproject.toml       # Dependencies (NO CHANGES - all deps exist)
├── .env.example         # Environment variables (MODIFIED - fix naming)
├── .env                 # User's actual config (not in git)
└── README.md            # Usage documentation (MODIFIED - add retrieval docs)

specs/
├── 1-vector-ingestion-pipeline/  # Completed feature
└── 2-rag-retrieval/              # This feature

history/
└── prompts/
    ├── 1-vector-ingestion-pipeline/
    └── 2-rag-retrieval/          # PHRs for this feature
```

**Structure Decision**: Extend existing monolithic FastAPI application in `backend/main.py` rather than creating separate service. This maintains simplicity and shares configuration/clients between ingestion and retrieval. Future refactoring can split services if scale requires.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

No constitution violations detected. This section is not applicable.

## Phase 0: Research (COMPLETED ✅)

**Output**: `specs/2-rag-retrieval/research.md`

**Key Findings**:
1. ✅ Qdrant `search()` method supports semantic retrieval with metadata filtering
2. ✅ Cohere `embed()` with `input_type="search_query"` optimizes query embeddings
3. ✅ Expected latency 300-800ms (well under 2s target)
4. ✅ No new dependencies needed - reuse existing infrastructure
5. ✅ Test suite design: 20 queries across 4 categories (Fundamentals, Simulation, Control, AI)
6. ✅ Configuration issue identified: `CO_API_KEY` → `COHERE_API_KEY` naming mismatch

**Research Questions Resolved**:
- RQ1: Qdrant search API capabilities → Use `search()` with `query_filter` parameter
- RQ2: Query embedding generation → Use `embed()` with `input_type="search_query"`
- RQ3: Performance optimization → No caching needed, async FastAPI sufficient
- RQ4: Error handling → Try-except for Cohere and Qdrant with HTTP 503 on failure
- RQ5: Test query design → 20 queries covering major book topics

## Phase 1: Design (COMPLETED ✅)

**Outputs**:
- `specs/2-rag-retrieval/data-model.md` - Pydantic models and validation rules
- `specs/2-rag-retrieval/contracts/api-contract.yaml` - OpenAPI 3.0 specification
- `specs/2-rag-retrieval/quickstart.md` - Setup and usage guide (see below)

### Data Models

**Request Models**:
1. `SearchRequest` - Query text, optional top_k (1-20, default 5), optional filters
2. `TestRetrievalRequest` - Optional query_ids (1-20), top_k parameter

**Response Models**:
1. `SearchResponse` - Query, results list, total count, latency, optional filters/message
2. `RetrievedChunk` - Text, score (0.0-1.0), metadata (url, chunk_index, source)
3. `TestRetrievalResponse` - Total queries, success count, success rate, meets_target flag, results list, avg latency
4. `TestResult` - Query ID, text, expected chapter, found flag, top result URL/score
5. `HealthResponse` - Status, Qdrant connected, collection exists, vector count, Cohere available

**Internal Models**:
- `TestQuery` - Test case definition (id, query, expected_chapter_pattern, category)

### API Endpoints

**POST /search**
- Purpose: Semantic search over book content
- Input: SearchRequest (query + optional top_k + optional filters)
- Output: SearchResponse (results + metadata + performance metrics)
- Errors: 400 (validation), 503 (service unavailable)

**POST /test-retrieval**
- Purpose: Run quality validation test suite
- Input: TestRetrievalRequest (optional query_ids + top_k)
- Output: TestRetrievalResponse (success rate + detailed results)
- Errors: 400 (validation), 503 (service unavailable)

**GET /health**
- Purpose: Service health check
- Input: None
- Output: HealthResponse (status + dependencies status)
- Errors: 503 (unhealthy)

### Architecture Decisions

**AD-001: Extend Existing FastAPI App**
- **Decision**: Add retrieval endpoints to `backend/main.py` (monolithic)
- **Alternatives Considered**: Separate microservice
- **Rationale**: Simplicity, shared clients/config, low traffic (<100 req/day), easy to split later if needed
- **Tradeoffs**: Ingestion and retrieval share same process (acceptable for prototype)

**AD-002: Separate Query Embedding Function**
- **Decision**: Create `embed_query()` using `input_type="search_query"`
- **Alternatives Considered**: Reuse existing `embed()` function
- **Rationale**: Cohere optimizes differently for documents vs queries, improves retrieval quality
- **Tradeoffs**: Slight code duplication (minimal - single function)

**AD-003: No Embedding Cache**
- **Decision**: No caching layer for initial version
- **Alternatives Considered**: Redis or in-memory cache
- **Rationale**: Low query volume, latency already <2s target, premature optimization
- **Tradeoffs**: Repeated queries regenerate embeddings (acceptable for <100 queries/day)
- **Future**: Can add caching if query volume increases (>1000 queries/day)

**AD-004: Metadata Filtering via Qdrant Query Filter**
- **Decision**: Use Qdrant's native `query_filter` parameter
- **Alternatives Considered**: Post-search filtering in Python
- **Rationale**: Better performance (filters before search), leverages database capabilities
- **Tradeoffs**: Tied to Qdrant filter DSL (acceptable - already using Qdrant)

**AD-005: Test Suite in-code (No External File)**
- **Decision**: Define 20 test queries as Python constant in code
- **Alternatives Considered**: JSON/YAML file, database table
- **Rationale**: Simple, version-controlled, no file I/O overhead
- **Tradeoffs**: Modifying tests requires code change (acceptable for stable test suite)

**AD-006: Fix Configuration Naming**
- **Decision**: Standardize on `COHERE_API_KEY` (update code to match `.env.example`)
- **Alternatives Considered**: Update `.env.example` to match code
- **Rationale**: `COHERE_API_KEY` more explicit and consistent with `QDRANT_API_KEY`
- **Tradeoffs**: Breaking change for existing `.env` files (low impact - dev only)

### Integration Points

**Existing Components (Reused)**:
1. **FastAPI App** (`app = FastAPI(...)`) - Add new routes
2. **Qdrant Client** (`qdrant_client`) - Use `search()` method
3. **Cohere Client** (`co`) - Use `embed()` for queries
4. **Environment Config** (`load_dotenv()`) - Add/fix variable names
5. **Logging** (`logging` module) - Log retrieval operations

**New Components (Added)**:
1. **`embed_query()` function** - Generate query embeddings with `search_query` input type
2. **Request/Response Models** - Pydantic classes for validation
3. **`/search` endpoint** - Semantic search handler
4. **`/test-retrieval` endpoint** - Quality validation handler
5. **`/health` endpoint** - Health check handler
6. **`TEST_QUERIES` constant** - 20 test queries with expected results

### File Modifications

**backend/main.py** (EXTENSIVE MODIFICATIONS):
- Add imports: `from pydantic import BaseModel, Field, field_validator`
- Add Pydantic models (8 classes)
- Add `embed_query()` function
- Add `/search` endpoint handler
- Add `/test-retrieval` endpoint handler
- Add `/health` endpoint handler
- Add `TEST_QUERIES` constant (20 entries)
- Update: Change `CO_API_KEY` to `COHERE_API_KEY` (lines 23, 31)

**backend/.env.example** (MINOR UPDATE):
```diff
- CO_API_KEY=your-cohere-api-key-here
+ COHERE_API_KEY=your-cohere-api-key-here
```

**backend/README.md** (NEW FILE):
- Add usage documentation for retrieval endpoints
- Document request/response formats
- Provide curl examples
- Explain test suite

## Phase 2: Task Breakdown

**Output**: `specs/2-rag-retrieval/tasks.md` (generated by `/sp.tasks` command)

**High-Level Task Categories** (to be detailed in tasks.md):
1. **Configuration Fix** (1 task)
   - Update environment variable naming consistency
2. **Core Functions** (2 tasks)
   - Implement `embed_query()` function
   - Implement query filter builder
3. **Pydantic Models** (2 tasks)
   - Implement request models (SearchRequest, TestRetrievalRequest)
   - Implement response models (SearchResponse, TestRetrievalResponse, HealthResponse, etc.)
4. **API Endpoints** (3 tasks)
   - Implement `/search` endpoint with error handling
   - Implement `/test-retrieval` endpoint with test suite
   - Implement `/health` endpoint with dependency checks
5. **Test Suite** (1 task)
   - Define TEST_QUERIES constant with 20 queries
6. **Documentation** (1 task)
   - Create README.md with usage examples
7. **Testing & Validation** (1 task)
   - Manual testing via `/docs` and curl
   - Run test suite and verify 85% success rate

**Estimated Total Tasks**: 11 tasks across 7 categories

**Dependencies**:
- Configuration → Core Functions → Models → Endpoints → Testing
- Test Suite can be developed in parallel with Models

## Risk Assessment

### Technical Risks

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Cohere API rate limits | Low | Medium | Monitor usage, upgrade plan if needed, implement retry |
| Qdrant connection failures | Low | High | Health check endpoint, retry logic, graceful degradation |
| Low retrieval relevance (<85%) | Medium | Medium | Test suite validation, threshold tuning, query refinement |
| Configuration naming breaks existing installs | Medium | Low | Document breaking change, provide migration guide |
| Performance regression (>2s latency) | Low | Medium | Monitor latency metrics, optimize if needed |

### Implementation Risks

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| Pydantic model complexity | Low | Low | Use simple validation rules, reference existing models |
| Test suite maintenance burden | Medium | Low | Keep tests stable, only update for new content |
| Missing edge cases in error handling | Medium | Medium | Comprehensive try-except, test failure scenarios |
| Inconsistent metadata filtering behavior | Low | Medium | Document filter semantics clearly, add tests |

## Success Metrics

### Implementation Success

- ✅ All 3 endpoints implemented and functional
- ✅ Pydantic models validate input correctly (test with invalid data)
- ✅ Error handling catches Cohere and Qdrant failures
- ✅ Configuration naming updated consistently
- ✅ README documentation complete with examples

### Functional Success (from spec)

- **SC-001**: 85%+ relevance rate (test suite)
- **SC-002**: <2s single query latency
- **SC-003**: <3s avg latency for 10 concurrent queries
- **SC-004**: 100% metadata filtering accuracy
- **SC-005**: Exact top_k result counts (1-20)
- **SC-006**: 85%+ test suite success rate (17+/20 queries)
- **SC-007**: Graceful edge case handling (no crashes)
- **SC-008**: Retrieval quality matches baseline

### Validation Checklist

**Pre-Testing**:
- [ ] Code compiles without errors
- [ ] FastAPI `/docs` generates OpenAPI UI
- [ ] Environment variables configured correctly
- [ ] Qdrant collection accessible (403+ vectors)

**Functional Testing**:
- [ ] `/search` returns results for known query
- [ ] top_k parameter controls result count
- [ ] Metadata filters restrict results correctly
- [ ] Empty query returns 400 error
- [ ] Invalid top_k (0, 50) returns 400 error
- [ ] Cohere unavailable returns 503 error
- [ ] Qdrant unavailable returns 503 error

**Test Suite Validation**:
- [ ] `/test-retrieval` runs all 20 queries
- [ ] Success rate calculated correctly
- [ ] meets_target flag accurate
- [ ] Latency metrics reasonable (<2s per query)
- [ ] At least 17/20 queries succeed (85%)

**Health Check**:
- [ ] `/health` returns 200 when services healthy
- [ ] Reports Qdrant connection status
- [ ] Reports collection vector count
- [ ] Returns 503 when Qdrant unavailable

## Non-Functional Requirements

### Performance

**Latency Targets**:
- Single query: <2000ms (95th percentile)
- 10 concurrent queries: <3000ms average
- Health check: <500ms

**Optimization Strategies**:
- Async/await for I/O operations
- Connection pooling (automatic via clients)
- No vector payloads in search results (`with_vectors=False`)
- Limit top_k to 20 to control response size

### Scalability

**Current Scale**: ~400 embeddings, <100 queries/day
**Expected Scale**: ~1000 embeddings, ~500 queries/day (future)

**Scalability Considerations**:
- Qdrant handles millions of vectors (current scale negligible)
- FastAPI handles 1000+ req/s (overkill for expected load)
- Cohere API: 10k+ requests/month (sufficient)

**Future Scaling** (if needed):
- Add Redis caching for frequent queries
- Implement rate limiting at FastAPI level
- Consider read replicas for Qdrant (Qdrant Cloud feature)

### Security

**Authentication**: None (internal API, future enhancement)
**Input Validation**: Pydantic models enforce constraints
**Secrets Management**: Environment variables (`.env` not in git)
**API Key Protection**: Keys never logged or exposed in responses

**Security Considerations**:
- Rate limiting (future) - prevent abuse
- Query length limits (1000 chars) - prevent DoS
- top_k bounds (1-20) - prevent resource exhaustion
- No user-generated Qdrant queries - prevent injection

### Observability

**Logging**:
- All retrieval operations logged (query, result count, latency)
- Error conditions logged with stack traces
- Use Python `logging` module at INFO level

**Metrics** (embedded in responses):
- `latency_ms` - query processing time
- `total_results` - result count
- `score` - similarity scores
- `success_rate` - test suite metric

**Future Enhancements**:
- Structured logging (JSON format)
- Integration with monitoring tools (Prometheus, Datadog)
- Distributed tracing (OpenTelemetry)

## Deployment Considerations

### Environment Requirements

**Runtime**:
- Python 3.13+
- OS: Linux or Windows with Bash support
- Memory: 512MB minimum (FastAPI + clients)
- Network: Outbound HTTPS to Cohere API and Qdrant Cloud

**External Services**:
- Qdrant Cloud cluster (existing)
- Cohere API access (existing)

### Configuration

**Environment Variables** (`.env` file):
```env
COHERE_API_KEY=your-cohere-api-key
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_URL=https://your-cluster.qdrant.tech
```

**Deployment Steps**:
1. Ensure vector ingestion pipeline has run (collection populated)
2. Update `.env` with API keys
3. Run `uv pip install -r requirements.txt` (no new deps)
4. Start server: `python main.py --run-server`
5. Verify health: `curl http://localhost:8000/health`
6. Run tests: `curl -X POST http://localhost:8000/test-retrieval`

### Rollback Plan

**Minimal Risk** - No database migrations or destructive changes

If issues arise:
1. Stop server
2. Revert code changes in `backend/main.py`
3. Restore original `.env` (if needed)
4. Restart server

Ingestion functionality remains unaffected (separate endpoints).

## Future Enhancements

### Phase 3 (Post-MVP)

1. **Embedding Cache** - Redis cache for frequent queries (when volume >1000/day)
2. **Authentication** - API key or JWT authentication
3. **Rate Limiting** - Per-user or per-IP rate limits
4. **Advanced Filters** - Date ranges, numerical comparisons, full-text search
5. **Query Analytics** - Track popular queries, failed searches, low-scoring results
6. **Batch Search** - Process multiple queries in single request
7. **Re-ranking** - ML-based re-ranking of initial retrieval results
8. **Feedback Loop** - Collect relevance feedback to improve retrieval

### Integration with Chatbot

**Next Feature**: RAG Chatbot Integration (feature 3)

Will build on this retrieval API:
- Use `/search` endpoint to fetch relevant chunks
- Pass chunks as context to LLM for answer generation
- Implement conversational interface (chat history, follow-up questions)
- Add citation/source attribution to generated answers

## Appendix: Code Examples

### Example: embed_query() Function

```python
def embed_query(query: str) -> List[float]:
    """
    Generate query embedding using Cohere API.

    Uses input_type="search_query" optimized for search intent.

    Args:
        query: Natural language query string

    Returns:
        1024-dimensional embedding vector

    Raises:
        cohere.error.CohereAPIError: If Cohere API fails
    """
    try:
        response = co.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"  # Optimized for queries
        )
        return response.embeddings[0]
    except cohere.error.CohereAPIError as e:
        logging.error(f"Cohere API error: {str(e)}")
        raise HTTPException(
            status_code=503,
            detail="Embedding service unavailable"
        )
```

### Example: /search Endpoint

```python
@app.post("/search", response_model=SearchResponse)
async def search(request: SearchRequest):
    """Semantic search endpoint."""
    import time
    start_time = time.time()

    try:
        # Generate query embedding
        query_vector = embed_query(request.query)

        # Build Qdrant filter (if needed)
        query_filter = None
        if request.filters:
            query_filter = build_qdrant_filter(request.filters)

        # Perform vector search
        results = qdrant_client.search(
            collection_name=COLLECTION_NAME,
            query_vector=query_vector,
            limit=request.top_k,
            with_payload=True,
            with_vectors=False,
            query_filter=query_filter
        )

        # Format response
        retrieved_chunks = [
            RetrievedChunk(
                text=r.payload["text"],
                score=r.score,
                metadata=ChunkMetadata(**r.payload["metadata"])
            )
            for r in results
        ]

        latency_ms = int((time.time() - start_time) * 1000)

        return SearchResponse(
            query=request.query,
            results=retrieved_chunks,
            total_results=len(retrieved_chunks),
            latency_ms=latency_ms,
            filters_applied=request.filters,
            message="No relevant content found" if not retrieved_chunks else None
        )

    except Exception as e:
        logging.error(f"Search error: {str(e)}")
        raise HTTPException(status_code=503, detail=str(e))
```

### Example: Test Query Definitions

```python
TEST_QUERIES = [
    {"id": 1, "query": "What is inverse kinematics?",
     "expected": "chapter3-kinematics", "category": "Fundamentals"},
    {"id": 2, "query": "How does ROS2 message passing work?",
     "expected": "chapter2-ros2", "category": "Fundamentals"},
    {"id": 11, "query": "How do bipedal robots maintain balance?",
     "expected": "chapter9-bipedal-locomotion", "category": "Control"},
    {"id": 16, "query": "What are Vision-Language-Action models?",
     "expected": "chapter13-vla-models", "category": "AI"},
    # ... (16 more queries)
]
```

## References

- [Spec Document](./spec.md) - Feature requirements and success criteria
- [Research Document](./research.md) - Technical investigation findings
- [Data Model](./data-model.md) - Pydantic models and validation rules
- [API Contract](./contracts/api-contract.yaml) - OpenAPI 3.0 specification
- [Quickstart Guide](./quickstart.md) - Setup and usage instructions
- [Qdrant Python Client Docs](https://qdrant.tech/documentation/quick-start/)
- [Cohere Embed API Docs](https://docs.cohere.com/reference/embed)
- [FastAPI Documentation](https://fastapi.tiangolo.com/)
