---
description: "Task list for RAG Retrieval Pipeline implementation"
---

# Tasks: RAG Retrieval Pipeline

**Input**: Design documents from `/specs/2-rag-retrieval/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/api-contract.yaml

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Project structure**: `backend/` (single FastAPI application)
- All code modifications in `backend/main.py`
- Configuration in `backend/.env` and `backend/.env.example`

---

## Phase 1: Setup & Configuration

**Purpose**: Fix configuration issues and prepare environment

- [ ] **T001** [P] [Setup] Fix environment variable naming in `backend/.env.example`
  - **File**: `backend/.env.example`
  - **Change**: Rename `CO_API_KEY` to `COHERE_API_KEY` for consistency
  - **Acceptance**:
    - `.env.example` uses `COHERE_API_KEY` (not `CO_API_KEY`)
    - All three variables documented: `COHERE_API_KEY`, `QDRANT_API_KEY`, `QDRANT_URL`
    - File includes usage comments
  - **Test**: Verify variable names match plan.md specification

- [ ] **T002** [P] [Setup] Update environment variable reference in `backend/main.py`
  - **File**: `backend/main.py` (line ~23)
  - **Change**: Update `CO_API_KEY = os.getenv("CO_API_KEY")` to `COHERE_API_KEY = os.getenv("COHERE_API_KEY")`
  - **Also update**: Line ~31 where `co = cohere.Client(CO_API_KEY)` to use `COHERE_API_KEY`
  - **Acceptance**:
    - Code reads `COHERE_API_KEY` from environment
    - Cohere client initialized with correct variable
    - No references to `CO_API_KEY` remain in code
  - **Test**: Run `python main.py` and verify no environment variable errors

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] **T003** [P] [Foundation] Add Pydantic model imports to `backend/main.py`
  - **File**: `backend/main.py` (imports section, top of file)
  - **Add imports**:
    ```python
    from pydantic import BaseModel, Field, field_validator
    from typing import Optional, Dict, List
    import time
    ```
  - **Acceptance**:
    - All required Pydantic imports added
    - `time` module imported for latency tracking
    - No import errors when running file
  - **Test**: Run `python -c "import backend.main"` (if module structure supports) or check syntax

- [ ] **T004** [P] [Foundation] Implement `embed_query()` function in `backend/main.py`
  - **File**: `backend/main.py` (after existing `embed()` function)
  - **Location**: Insert after line ~162 (after existing `embed()` function)
  - **Implementation**: Create new function using `input_type="search_query"` (see plan.md Appendix for code)
  - **Acceptance**:
    - Function signature: `def embed_query(query: str) -> List[float]`
    - Uses `co.embed()` with `input_type="search_query"`
    - Returns 1024-dimensional vector
    - Handles `cohere.error.CohereAPIError` with HTTPException(503)
    - Includes logging for errors
  - **Test**: Call function with test query and verify it returns 1024-float list

- [ ] **T005** [Foundation] Implement `build_qdrant_filter()` helper function in `backend/main.py`
  - **File**: `backend/main.py` (after `embed_query()` function)
  - **Implementation**:
    ```python
    def build_qdrant_filter(filters: Dict[str, str]):
        """Build Qdrant filter from request filters."""
        from qdrant_client.http.models import Filter, FieldCondition, MatchValue

        conditions = []

        if "url_contains" in filters:
            conditions.append(
                FieldCondition(
                    key="metadata.url",
                    match=MatchValue(value=filters["url_contains"])
                )
            )

        if "url_exact" in filters:
            conditions.append(
                FieldCondition(
                    key="metadata.url",
                    match=MatchValue(value=filters["url_exact"])
                )
            )

        if "source" in filters:
            conditions.append(
                FieldCondition(
                    key="metadata.source",
                    match=MatchValue(value=filters["source"])
                )
            )

        if "chunk_index" in filters:
            conditions.append(
                FieldCondition(
                    key="metadata.chunk_index",
                    match=MatchValue(value=int(filters["chunk_index"]))
                )
            )

        return Filter(must=conditions) if conditions else None
    ```
  - **Acceptance**:
    - Handles all 4 filter types: `url_contains`, `url_exact`, `source`, `chunk_index`
    - Returns `Filter` object with conditions or `None` if no filters
    - Correctly maps to Qdrant metadata structure
  - **Test**: Call with various filter combinations and verify Filter object structure
  - **Depends on**: T004 (placement after embed_query)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Basic Semantic Search (Priority: P1) 🎯 MVP

**Goal**: Implement core retrieval functionality that searches embeddings and returns ranked results

**Independent Test**: Submit query "What is inverse kinematics?" and verify top 5 results contain relevant chunks from kinematics chapter

### Implementation for User Story 1

- [ ] **T006** [P] [US1] Implement `ChunkMetadata` Pydantic model in `backend/main.py`
  - **File**: `backend/main.py` (after foundational functions, before endpoints)
  - **Implementation**:
    ```python
    class ChunkMetadata(BaseModel):
        """Metadata associated with a retrieved chunk."""
        url: str = Field(..., description="Source URL")
        chunk_index: int = Field(..., ge=0, description="Chunk index")
        source: str = Field(..., description="Source type")
    ```
  - **Acceptance**:
    - Model has 3 required fields: `url`, `chunk_index`, `source`
    - `chunk_index` validated as >= 0
    - Model includes field descriptions
  - **Test**: Instantiate model with sample data and verify validation

- [ ] **T007** [P] [US1] Implement `RetrievedChunk` Pydantic model in `backend/main.py`
  - **File**: `backend/main.py` (after `ChunkMetadata`)
  - **Implementation**: See data-model.md for complete specification
  - **Acceptance**:
    - Fields: `text` (str), `score` (float, 0.0-1.0), `metadata` (ChunkMetadata)
    - Score validation: `ge=0.0`, `le=1.0`
    - Includes example in `Config.json_schema_extra`
  - **Test**: Create instance and verify score validation (reject 1.5, accept 0.87)

- [ ] **T008** [P] [US1] Implement `SearchRequest` Pydantic model in `backend/main.py`
  - **File**: `backend/main.py` (after `RetrievedChunk`)
  - **Implementation**: See data-model.md for complete specification
  - **Acceptance**:
    - Fields: `query` (str, 1-1000 chars), `top_k` (int, 1-20, default 5), `filters` (optional dict)
    - Query validator removes whitespace and rejects empty strings
    - Filters validator checks allowed keys: `url_contains`, `url_exact`, `source`, `chunk_index`
    - Includes validation error messages
  - **Test**: Test with invalid inputs (empty query, top_k=50, invalid filter key) and verify ValidationError

- [ ] **T009** [P] [US1] Implement `SearchResponse` Pydantic model in `backend/main.py`
  - **File**: `backend/main.py` (after `SearchRequest`)
  - **Implementation**: See data-model.md for complete specification
  - **Acceptance**:
    - Fields: `query`, `results` (list), `total_results`, `latency_ms`, `filters_applied`, `message`
    - All numeric fields validated (ge=0)
    - Includes example in Config
  - **Test**: Create instance with sample data and verify serialization to JSON

- [ ] **T010** [US1] Implement `/search` POST endpoint in `backend/main.py`
  - **File**: `backend/main.py` (in endpoints section, after existing endpoints)
  - **Implementation**: See plan.md Appendix for code example
  - **Steps**:
    1. Record start time for latency tracking
    2. Call `embed_query()` to get query vector
    3. Call `build_qdrant_filter()` if filters present
    4. Call `qdrant_client.search()` with parameters
    5. Format results as list of `RetrievedChunk`
    6. Calculate latency
    7. Return `SearchResponse`
  - **Error Handling**:
    - Catch `cohere.error.CohereAPIError` → 503
    - Catch Qdrant errors → 503
    - Catch general exceptions → 503 with error message
  - **Logging**: Log query, result count, and latency at INFO level
  - **Acceptance**:
    - Endpoint: `POST /search`
    - Request model: `SearchRequest`
    - Response model: `SearchResponse`
    - Returns exactly `top_k` results (or fewer if collection smaller)
    - Results ordered by score (descending)
    - Latency tracked and included in response
    - Error handling for Cohere and Qdrant failures
  - **Test**:
    - Submit query via curl/Swagger UI
    - Verify 5 results returned by default
    - Verify results contain text, score, metadata
    - Verify scores in descending order
  - **Depends on**: T003, T004, T005, T006, T007, T008, T009

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

**Manual Test for US1**:
```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{"query": "What is inverse kinematics?", "top_k": 5}'
```

Expected: 5 results from kinematics-related chapters with scores > 0.7

---

## Phase 4: User Story 2 - Configurable Retrieval Parameters (Priority: P2)

**Goal**: Enable users to adjust top_k parameter to control result count

**Independent Test**: Submit same query with top_k=3, top_k=10, and verify correct counts returned

### Implementation for User Story 2

- [ ] **T011** [US2] Add top_k parameter validation test cases
  - **File**: Testing via `/search` endpoint (already implemented in T010)
  - **Action**: No new code - verify existing validation works
  - **Acceptance**:
    - top_k=1 returns 1 result
    - top_k=20 returns up to 20 results
    - top_k=0 returns 400 error
    - top_k=50 returns 400 error
    - top_k not specified defaults to 5
  - **Test**: Submit requests with various top_k values via curl/Swagger

- [ ] **T012** [US2] Document top_k usage in FastAPI auto-generated docs
  - **File**: `backend/main.py` (SearchRequest model and /search endpoint)
  - **Action**: Ensure docstrings are clear
  - **Acceptance**:
    - SearchRequest.top_k field has description
    - /search endpoint docstring mentions top_k parameter
    - Swagger UI shows top_k range (1-20)
  - **Test**: Open http://localhost:8000/docs and verify documentation clarity

**Checkpoint**: User Story 2 complete - top_k configuration works independently

**Manual Test for US2**:
```bash
# Test top_k=3
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{"query": "bipedal locomotion", "top_k": 3}'

# Verify: total_results == 3
```

---

## Phase 5: User Story 3 - Metadata Filtering (Priority: P2)

**Goal**: Enable users to filter results by URL patterns, source, or chunk index

**Independent Test**: Submit query with url_contains="module1" and verify all results from Module 1

### Implementation for User Story 3

- [ ] **T013** [US3] Test metadata filtering with url_contains filter
  - **File**: Testing via `/search` endpoint (already implemented in T010)
  - **Action**: Verify `build_qdrant_filter()` correctly handles url_contains
  - **Acceptance**:
    - Query with `filters: {"url_contains": "module1"}` returns only Module 1 results
    - Query with `filters: {"url_contains": "locomotion"}` returns only locomotion chapter results
    - Verify `filters_applied` field in response shows applied filters
  - **Test**: Submit filtered queries and manually check result URLs
  - **Depends on**: T010 (search endpoint must be working)

- [ ] **T014** [US3] Test metadata filtering with multiple filter types
  - **File**: Testing via `/search` endpoint
  - **Action**: Test combinations of filters
  - **Acceptance**:
    - `url_contains` + `source` filters work together (AND logic)
    - `url_exact` filter matches exact URL
    - `chunk_index` filter returns specific chunk
    - Invalid filter keys return 400 error
  - **Test Cases**:
    1. `{"url_contains": "module1", "source": "website"}` → Only Module 1 website chunks
    2. `{"url_exact": "https://...chapter3"}` → Only chunks from that exact URL
    3. `{"invalid_key": "value"}` → 400 ValidationError
  - **Depends on**: T013

- [ ] **T015** [US3] Test edge case: filter with no matches
  - **File**: Testing via `/search` endpoint
  - **Action**: Submit query with filter that matches zero chunks
  - **Acceptance**:
    - Returns empty results array (`results: []`)
    - `total_results: 0`
    - `message` field explains no matches found
    - Does not crash or return 500 error
  - **Test**: `{"query": "robots", "filters": {"url_contains": "nonexistent"}}`
  - **Expected**: `{"results": [], "total_results": 0, "message": "No relevant content found"}`
  - **Depends on**: T014

**Checkpoint**: User Story 3 complete - metadata filtering works independently

**Manual Test for US3**:
```bash
# Test URL filtering
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How do robots work?",
    "top_k": 5,
    "filters": {"url_contains": "module1"}
  }'

# Verify: All result URLs contain "module1"
```

---

## Phase 6: User Story 5 - Retrieval Quality Testing (Priority: P1) 🎯 MVP

**Goal**: Implement test suite to validate retrieval accuracy with 20 predefined queries

**Independent Test**: Run test suite and verify 85%+ success rate (17+ queries return expected content)

### Implementation for User Story 5

- [ ] **T016** [P] [US5] Define `TEST_QUERIES` constant in `backend/main.py`
  - **File**: `backend/main.py` (after Pydantic models, before endpoints)
  - **Implementation**: Create list of 20 test queries based on research.md findings
  - **Structure**:
    ```python
    TEST_QUERIES = [
        {"id": 1, "query": "What is inverse kinematics?", "expected_chapter": "chapter3-kinematics", "category": "Fundamentals"},
        {"id": 2, "query": "How does ROS2 message passing work?", "expected_chapter": "chapter2-ros2", "category": "Fundamentals"},
        # ... 18 more queries covering all 4 categories
    ]
    ```
  - **Categories**: Fundamentals (5), Simulation (5), Control (5), AI (5)
  - **Acceptance**:
    - Exactly 20 queries defined
    - Each has id (1-20), query text, expected_chapter pattern, category
    - Covers major book topics from research.md test query list
  - **Test**: Verify list length and structure

- [ ] **T017** [P] [US5] Implement `TestRetrievalRequest` Pydantic model in `backend/main.py`
  - **File**: `backend/main.py` (after SearchResponse)
  - **Implementation**: See data-model.md
  - **Acceptance**:
    - Fields: `query_ids` (optional list[int]), `top_k` (int, default 5)
    - Validates query_ids are between 1-20
    - None for query_ids means "run all"
  - **Test**: Create instance with valid/invalid query_ids

- [ ] **T018** [P] [US5] Implement `TestResult` and `TestRetrievalResponse` Pydantic models in `backend/main.py`
  - **File**: `backend/main.py` (after TestRetrievalRequest)
  - **Implementation**: See data-model.md
  - **Acceptance**:
    - `TestResult`: query_id, query_text, expected_chapter, found_in_top_k, top_result_url, top_result_score
    - `TestRetrievalResponse`: total_queries, successful_queries, success_rate, meets_target, results, avg_latency_ms
    - Success rate calculated correctly (successful/total)
    - meets_target flag set based on >= 0.85 threshold
  - **Test**: Create instances with sample data

- [ ] **T019** [US5] Implement `/test-retrieval` POST endpoint in `backend/main.py`
  - **File**: `backend/main.py` (after /search endpoint)
  - **Implementation**:
    1. Parse request to get query_ids (or use all if None)
    2. Filter TEST_QUERIES based on query_ids
    3. For each test query:
       - Call embed_query()
       - Call qdrant_client.search() with top_k
       - Check if expected_chapter appears in any top-k result URLs
       - Record result (found_in_top_k, top_result_url, top_result_score)
       - Track latency per query
    4. Calculate aggregate metrics (total, successful, success_rate, avg_latency)
    5. Set meets_target = (success_rate >= 0.85)
    6. Return TestRetrievalResponse
  - **Error Handling**: Same as /search (503 for API failures)
  - **Logging**: Log test run start, individual query results, final metrics
  - **Acceptance**:
    - Endpoint: `POST /test-retrieval`
    - Request model: `TestRetrievalRequest`
    - Response model: `TestRetrievalResponse`
    - Runs all 20 queries if query_ids=None
    - Runs specific queries if query_ids provided
    - Checks expected_chapter as substring in result URLs
    - Calculates success_rate correctly
    - Returns detailed results for each query
  - **Test**:
    - Run full test suite (20 queries)
    - Verify success_rate >= 0.85
    - Run subset (query_ids=[1,2,3])
  - **Depends on**: T016, T017, T018

**Checkpoint**: User Story 5 complete - test suite validates retrieval quality

**Manual Test for US5**:
```bash
# Run full test suite
curl -X POST http://localhost:8000/test-retrieval \
  -H "Content-Type: application/json" \
  -d '{"query_ids": null, "top_k": 5}'

# Expected: success_rate >= 0.85, meets_target = true
```

---

## Phase 7: User Story 4 - Retrieval Performance Validation (Priority: P3)

**Goal**: Add performance monitoring and logging to track retrieval latency

**Independent Test**: Run queries and verify latency_ms values are reasonable (<2000ms)

### Implementation for User Story 4

- [ ] **T020** [US4] Add latency tracking to `/search` endpoint
  - **File**: `backend/main.py` (/search endpoint - already implemented in T010)
  - **Action**: Verify latency calculation exists and is accurate
  - **Acceptance**:
    - `latency_ms` field in SearchResponse populated
    - Latency measured from request start to response ready
    - Typical latency 300-800ms under normal conditions
  - **Test**: Submit query and check `latency_ms` in response
  - **Note**: This is verification only - already implemented in T010

- [ ] **T021** [US4] Add comprehensive logging for retrieval operations
  - **File**: `backend/main.py` (/search and /test-retrieval endpoints)
  - **Action**: Add INFO level logging statements
  - **Log Events**:
    - Search request received (query text, top_k, filters)
    - Embedding generation time
    - Qdrant search time
    - Result count and latency
    - Test suite run (start, progress, final metrics)
  - **Acceptance**:
    - All key operations logged at INFO level
    - Errors logged at ERROR level with stack traces
    - Query text truncated to 100 chars in logs (for long queries)
    - No sensitive data logged (API keys, full embeddings)
  - **Test**: Run queries and check console/log output
  - **Depends on**: T010, T019

- [ ] **T022** [US4] Test concurrent query performance
  - **File**: Manual testing (no code changes)
  - **Action**: Use load testing tool or script to send 10 concurrent queries
  - **Acceptance**:
    - Average latency <3000ms with 10 concurrent queries
    - No crashes or 500 errors
    - All queries return valid results
  - **Test Script** (example):
    ```bash
    for i in {1..10}; do
      curl -X POST http://localhost:8000/search \
        -H "Content-Type: application/json" \
        -d '{"query": "test query '$i'", "top_k": 5}' &
    done
    wait
    ```
  - **Depends on**: T010

**Checkpoint**: User Story 4 complete - performance monitoring in place

---

## Phase 8: Health Check Endpoint

**Goal**: Add service health check endpoint to verify connectivity to dependencies

**Independent Test**: Call /health and verify status="healthy"

### Implementation for Health Check

- [ ] **T023** [P] [Health] Implement `HealthResponse` Pydantic model in `backend/main.py`
  - **File**: `backend/main.py` (after TestRetrievalResponse)
  - **Implementation**:
    ```python
    class HealthResponse(BaseModel):
        """Health check response."""
        status: str = Field(..., description="Overall service status")
        qdrant_connected: bool
        collection_exists: bool
        collection_vectors_count: int
        cohere_available: bool
    ```
  - **Acceptance**:
    - 5 fields as specified
    - Status can be "healthy" or "unhealthy"
  - **Test**: Create instance with sample data

- [ ] **T024** [Health] Implement `/health` GET endpoint in `backend/main.py`
  - **File**: `backend/main.py` (after /test-retrieval endpoint)
  - **Implementation**:
    1. Check Qdrant connection: Try `qdrant_client.get_collections()`
    2. Check collection exists: Look for COLLECTION_NAME in collections
    3. Get vector count: `qdrant_client.get_collection(COLLECTION_NAME).vectors_count`
    4. Check Cohere: Try simple embed call with "test"
    5. Set status="healthy" if all checks pass, "unhealthy" otherwise
    6. Return 200 if healthy, 503 if unhealthy
  - **Error Handling**: Catch connection errors and set flags to False
  - **Acceptance**:
    - Endpoint: `GET /health`
    - Response model: `HealthResponse`
    - Returns 200 with status="healthy" when all services up
    - Returns 503 with status="unhealthy" when any service down
    - Provides detailed status for each dependency
  - **Test**:
    - Normal case: All services up → 200, status="healthy"
    - Failure case: Stop Qdrant → 503, status="unhealthy", qdrant_connected=false
  - **Depends on**: T023

**Checkpoint**: Health check endpoint functional

**Manual Test for Health**:
```bash
curl http://localhost:8000/health

# Expected:
# {
#   "status": "healthy",
#   "qdrant_connected": true,
#   "collection_exists": true,
#   "collection_vectors_count": 403,
#   "cohere_available": true
# }
```

---

## Phase 9: Documentation & Polish

**Purpose**: Complete documentation and validate full system

- [ ] **T025** [P] [Doc] Create `backend/README.md` with retrieval API documentation
  - **File**: `backend/README.md` (new file)
  - **Content**:
    - Overview of retrieval endpoints
    - Quick start guide
    - API examples with curl
    - Configuration instructions
    - Troubleshooting section
  - **Acceptance**:
    - README exists with clear structure
    - All 3 endpoints documented with examples
    - Configuration variables explained
    - Links to Swagger UI documentation
  - **Test**: Follow README instructions and verify they work

- [ ] **T026** [P] [Doc] Update main `README.md` with retrieval pipeline section
  - **File**: Root `README.md`
  - **Action**: Add section about RAG retrieval pipeline feature
  - **Content**:
    - Brief description
    - Link to backend/README.md
    - Link to quickstart.md
  - **Acceptance**:
    - Root README mentions retrieval feature
    - Links to detailed documentation work
  - **Test**: Review README for completeness

- [ ] **T027** [Polish] Validate quickstart.md against implementation
  - **File**: `specs/2-rag-retrieval/quickstart.md` (existing)
  - **Action**: Walk through quickstart guide step-by-step
  - **Acceptance**:
    - All commands work as documented
    - Examples produce expected output
    - Troubleshooting section covers actual issues
    - No broken references or outdated information
  - **Test**: Fresh installation following quickstart exactly

- [ ] **T028** [Polish] Run full integration test cycle
  - **Action**: Complete end-to-end testing
  - **Test Sequence**:
    1. Start server: `python main.py --run-server`
    2. Health check: `curl http://localhost:8000/health` → healthy
    3. Simple search: `curl POST /search` → 5 results
    4. Filtered search: `curl POST /search` with filters → correct results
    5. top_k variations: Test 1, 10, 20 → correct counts
    6. Test suite: `curl POST /test-retrieval` → success_rate >= 0.85
    7. Edge cases: empty results, invalid inputs → proper errors
  - **Acceptance**:
    - All tests pass
    - No crashes or unexpected errors
    - Performance within targets (<2s latency)
    - Test suite meets 85% success threshold
  - **Depends on**: All previous tasks

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: T001, T002 - Can start immediately
- **Foundational (Phase 2)**: T003, T004, T005 - Depends on Setup, BLOCKS all user stories
- **User Story 1 (Phase 3)**: T006-T010 - Depends on Foundational
- **User Story 2 (Phase 4)**: T011-T012 - Depends on US1 completion
- **User Story 3 (Phase 5)**: T013-T015 - Depends on US1 completion
- **User Story 5 (Phase 6)**: T016-T019 - Depends on US1 completion
- **User Story 4 (Phase 7)**: T020-T022 - Depends on US1 and US5 completion
- **Health Check (Phase 8)**: T023-T024 - Can start after Foundational
- **Documentation (Phase 9)**: T025-T028 - Depends on all features complete

### Critical Path

```
T001, T002 (Setup)
  ↓
T003, T004, T005 (Foundation)
  ↓
T006, T007, T008, T009 → T010 (US1 - Core Search)
  ↓
[Parallel from here]
├─ T011, T012 (US2 - Configurable top_k)
├─ T013, T014, T015 (US3 - Filtering)
├─ T016, T017, T018 → T019 (US5 - Test Suite)
└─ T023 → T024 (Health Check)
  ↓
T020, T021, T022 (US4 - Performance)
  ↓
T025, T026, T027, T028 (Documentation & Polish)
```

### Parallel Opportunities

**Within Foundation** (Phase 2):
- T003, T004 can run in parallel (different locations in file)
- T005 depends on T004 (uses similar patterns)

**US1 Models** (Phase 3):
- T006, T007, T008, T009 can all run in parallel (independent Pydantic models)

**After US1 Complete**:
- US2 (T011-T012), US3 (T013-T015), US5 (T016-T019), Health (T023-T024) can all start in parallel

**Documentation** (Phase 9):
- T025, T026 can run in parallel (different files)

---

## Implementation Strategy

### MVP First (User Story 1 + User Story 5)

1. **Phase 1**: Setup (T001-T002)
2. **Phase 2**: Foundation (T003-T005)
3. **Phase 3**: User Story 1 (T006-T010) - Core search
4. **Phase 6**: User Story 5 (T016-T019) - Test validation
5. **STOP and VALIDATE**: Run test suite, verify 85%+ success
6. **Result**: Working retrieval API with quality validation

### Full Feature Delivery

1. Complete MVP (US1 + US5)
2. Add US2 (T011-T012) - Configurable top_k
3. Add US3 (T013-T015) - Metadata filtering
4. Add Health Check (T023-T024)
5. Add US4 (T020-T022) - Performance monitoring
6. Polish & Documentation (T025-T028)

### Parallel Team Strategy

With 2 developers:

1. **Both**: Setup + Foundation (T001-T005)
2. **Developer A**: US1 (T006-T010) → US5 (T016-T019)
3. **Developer B**: Health Check (T023-T024) → US2 (T011-T012) → US3 (T013-T015)
4. **Both**: US4 (T020-T022) → Documentation (T025-T028)

---

## Task Summary

| Phase | Tasks | Priority | Dependencies |
|-------|-------|----------|--------------|
| Setup | T001-T002 | P0 | None |
| Foundation | T003-T005 | P0 | Setup |
| US1 (Search) | T006-T010 | P1 | Foundation |
| US2 (top_k) | T011-T012 | P2 | US1 |
| US3 (Filtering) | T013-T015 | P2 | US1 |
| US5 (Testing) | T016-T019 | P1 | Foundation |
| US4 (Performance) | T020-T022 | P3 | US1, US5 |
| Health Check | T023-T024 | P2 | Foundation |
| Documentation | T025-T028 | P3 | All features |

**Total Tasks**: 28 tasks
**MVP Tasks**: 13 tasks (Setup + Foundation + US1 + US5)
**Estimated Effort**: 2-3 days for MVP, 4-5 days for full feature

---

## Notes

- All tasks modify `backend/main.py` except documentation tasks
- No new dependencies required - reuse existing infrastructure
- [P] tasks can run in parallel when in same phase
- User stories are independently testable after US1 complete
- Stop at any checkpoint to validate story works in isolation
- Test suite (US5) provides quality validation for retrieval accuracy
- Health check enables production monitoring
