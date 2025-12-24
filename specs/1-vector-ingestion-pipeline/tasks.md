# Task Breakdown: Vector Ingestion Pipeline

**Feature**: 1-vector-ingestion-pipeline
**Branch**: `1-vector-ingestion-pipeline`
**Created**: 2025-12-16
**Status**: Ready for Implementation

## Overview

This document breaks down Feature 1 (Vector Ingestion Pipeline) into 32 actionable tasks across 6 phases. Tasks are organized by user story (US1-US3) to enable independent implementation and testing.

**Total Tasks**: 32
**User Stories**: 3 (US1: P1, US2: P2, US3: P3)
**Estimated Duration**: 3-4 days

---

## Task Organization by Phase

- **Phase 0**: Setup & Configuration (6 tasks)
- **Phase 1**: Foundational Prerequisites (4 tasks)
- **Phase 2**: US1 - Automated Book Content Ingestion (8 tasks)
- **Phase 3**: US2 - Embeddings Generation and Storage (8 tasks)
- **Phase 4**: US3 - Pipeline Monitoring and Reliability (4 tasks)
- **Phase 5**: Polish & Integration (2 tasks)

---

## User Story Mapping

- **US1 (P1)**: Automated Book Content Ingestion - Phase 2
- **US2 (P2)**: Embeddings Generation and Storage - Phase 3
- **US3 (P3)**: Pipeline Monitoring and Reliability - Phase 4

---

## Phase 0: Setup & Configuration

**Goal**: Initialize project structure, dependencies, and environment configuration
**Duration**: 0.5 day
**Dependencies**: None

### Tasks

- [ ] T001 Create backend project structure
  - **File**: `backend/` directory structure
  - **Action**: Create directories: `backend/`
  - **Content**: Ensure backend directory exists at project root
  - **Acceptance**: backend/ directory exists

- [ ] T002 Create pyproject.toml with dependencies
  - **File**: `backend/pyproject.toml`
  - **Action**: Define project metadata and dependencies
  - **Content**:
    ```toml
    [project]
    name = "vector-ingestion-pipeline"
    version = "0.1.0"
    requires-python = ">=3.13"
    dependencies = [
        "fastapi>=0.104.0",
        "uvicorn[standard]>=0.24.0",
        "aiohttp>=3.9.0",
        "beautifulsoup4>=4.12.0",
        "cohere>=4.32.0",
        "qdrant-client>=1.6.0",
        "python-dotenv>=1.0.0",
        "lxml>=4.9.0",
    ]
    ```
  - **Acceptance**: pyproject.toml created with all required dependencies

- [ ] T003 Generate requirements.txt from pyproject.toml
  - **File**: `backend/requirements.txt`
  - **Action**: Convert pyproject.toml dependencies to requirements.txt format
  - **Content**: List all dependencies with version constraints
  - **Acceptance**: requirements.txt exists and contains all dependencies

- [ ] T004 [P] Create .env.example with required environment variables
  - **File**: `backend/.env.example`
  - **Content**:
    ```
    CO_API_KEY=your_cohere_api_key_here
    QDRANT_API_KEY=your_qdrant_api_key_here
    QDRANT_URL=https://your-cluster-url.qdrant.tech
    COLLECTION_NAME=humanoid-robotics-embeddings
    TARGET_WEBSITE_URL=https://physical-ai-and-humanoid-robotics-t-two.vercel.app/
    ```
  - **Acceptance**: .env.example created with all required variables

- [ ] T005 [P] Create .env file for local development (gitignored)
  - **File**: `backend/.env`
  - **Action**: Copy .env.example and fill with actual API keys
  - **Acceptance**: .env created and added to .gitignore

- [ ] T006 Create README.md with setup instructions
  - **File**: `backend/README.md`
  - **Content**: Document setup steps, environment variables, and usage instructions
  - **Sections**: Installation, Configuration, Running the Pipeline, API Endpoints
  - **Acceptance**: README.md created with complete setup guide

**Phase 0 Acceptance Criteria**:
- ✅ Project structure initialized
- ✅ Dependencies defined in pyproject.toml
- ✅ Environment variables configured
- ✅ Documentation complete

---

## Phase 1: Foundational Prerequisites

**Goal**: Implement core utilities and configurations needed by all user stories
**Duration**: 0.5 day
**Dependencies**: Phase 0

### Tasks

- [ ] T007 Initialize FastAPI application in main.py
  - **File**: `backend/main.py`
  - **Content**:
    ```python
    import asyncio
    from typing import List, Dict, Any
    import aiohttp
    from bs4 import BeautifulSoup
    from qdrant_client import QdrantClient
    from qdrant_client.http.models import Distance, VectorParams, PointStruct
    import cohere
    import logging
    import os
    from fastapi import FastAPI, HTTPException
    import uvicorn
    from dotenv import load_dotenv

    # Load environment variables
    load_dotenv()

    # Initialize FastAPI app
    app = FastAPI(title="Vector Ingestion Pipeline")

    # Configuration
    CO_API_KEY = os.getenv("CO_API_KEY")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    QDRANT_URL = os.getenv("QDRANT_URL")
    COLLECTION_NAME = os.getenv("COLLECTION_NAME", "humanoid-robotics-embeddings")
    TARGET_WEBSITE_URL = os.getenv("TARGET_WEBSITE_URL")

    # Initialize clients
    co = cohere.Client(CO_API_KEY)
    qdrant_client = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
    ```
  - **Acceptance**: FastAPI app initialized, environment variables loaded, clients configured

- [ ] T008 Configure logging for pipeline operations
  - **File**: `backend/main.py`
  - **Add after imports**:
    ```python
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    logger = logging.getLogger(__name__)
    ```
  - **Acceptance**: Logging configured with timestamp, level, and message format

- [ ] T009 Create Qdrant collection initialization function
  - **File**: `backend/main.py`
  - **Function**: `create_collection()`
  - **Content**:
    ```python
    def create_collection():
        """Create Qdrant collection if it doesn't exist"""
        try:
            collections = qdrant_client.get_collections().collections
            if COLLECTION_NAME not in [c.name for c in collections]:
                qdrant_client.create_collection(
                    collection_name=COLLECTION_NAME,
                    vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
                )
                logger.info(f"Created collection: {COLLECTION_NAME}")
            else:
                logger.info(f"Collection already exists: {COLLECTION_NAME}")
        except Exception as e:
            logger.error(f"Error creating collection: {str(e)}")
            raise
    ```
  - **Acceptance**: Function creates collection with 1024-dim vectors, cosine distance

- [ ] T010 Add main execution entry point
  - **File**: `backend/main.py`
  - **Add at end of file**:
    ```python
    if __name__ == "__main__":
        uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
    ```
  - **Acceptance**: FastAPI can run with `python main.py`

**Phase 1 Acceptance Criteria**:
- ✅ FastAPI application initialized
- ✅ Logging configured
- ✅ Qdrant collection creation function implemented
- ✅ Application can start successfully

---

## Phase 2: US1 - Automated Book Content Ingestion

**Goal**: Implement automated URL discovery and content extraction
**Duration**: 1 day
**Dependencies**: Phase 1
**User Story**: US1 (P1) - As a content manager, I want to automatically ingest all published book URLs

### Independent Test Criteria

**Test**: The system can fetch all book URLs from the deployed website, extract readable content from each page, and store the raw content for further processing.

**Validation**:
1. Run ingestion for target website
2. Verify all URLs from sitemap are discovered
3. Verify readable content extracted without HTML markup
4. Verify content stored with metadata (URL, title, timestamp)

### Tasks

- [ ] T011 [US1] Implement get_urls_from_sitemap function
  - **File**: `backend/main.py`
  - **Function**: `async def get_urls_from_sitemap(sitemap_url: str) -> List[str]`
  - **Logic**:
    - Fetch sitemap.xml using aiohttp
    - Parse XML with BeautifulSoup
    - Extract all <loc> tags containing URLs
    - Fix localhost URLs to use actual domain
    - Return list of discovered URLs
  - **Error Handling**: Log and return empty list if sitemap not found
  - **Acceptance**: Function fetches sitemap.xml and returns all URLs

- [ ] T012 [US1] Implement get_all_urls function with sitemap fallback
  - **File**: `backend/main.py`
  - **Function**: `async def get_all_urls(base_url: str) -> List[str]`
  - **Logic**:
    - Try get_urls_from_sitemap first
    - If sitemap fails, crawl website:
      - Fetch base_url
      - Parse HTML with BeautifulSoup
      - Extract all <a href> links
      - Filter to same domain only
    - Return unique list of URLs
  - **Acceptance**: Function returns all URLs from sitemap or crawl

- [ ] T013 [US1] Implement extract_text_from_url function
  - **File**: `backend/main.py`
  - **Function**: `async def extract_text_from_url(url: str) -> Dict[str, str]`
  - **Logic**:
    - Fetch URL using aiohttp
    - Parse HTML with BeautifulSoup
    - Extract title from <title> tag
    - Extract readable text from <article>, <main>, or <body>
    - Remove HTML tags, scripts, styles
    - Clean up whitespace
    - Return dict with url, title, text_content
  - **Error Handling**: Return None if fetch fails, log error
  - **Acceptance**: Function extracts clean text without HTML markup

- [ ] T014 [US1] Add retry mechanism for URL fetching
  - **File**: `backend/main.py`
  - **Modify**: extract_text_from_url
  - **Add**: Retry logic with exponential backoff
    ```python
    async def fetch_with_retry(session, url, max_retries=3):
        for attempt in range(max_retries):
            try:
                async with session.get(url) as response:
                    if response.status == 200:
                        return await response.text()
                    elif response.status == 404:
                        logger.warning(f"URL not found: {url}")
                        return None
            except Exception as e:
                if attempt == max_retries - 1:
                    logger.error(f"Failed after {max_retries} attempts: {url}")
                    raise
                await asyncio.sleep(2 ** attempt)
        return None
    ```
  - **Acceptance**: URL fetches retry up to 3 times with exponential backoff

- [ ] T015 [US1] Implement batch URL processing
  - **File**: `backend/main.py`
  - **Function**: `async def process_urls_batch(urls: List[str]) -> List[Dict[str, str]]`
  - **Logic**:
    - Process URLs in batches of 10 concurrently
    - Use asyncio.gather with return_exceptions=True
    - Extract text from each URL
    - Filter out None results (failed fetches)
    - Return list of extracted content dicts
  - **Acceptance**: Function processes URLs concurrently, handles failures gracefully

- [ ] T016 [US1] Add URL validation and filtering
  - **File**: `backend/main.py`
  - **Function**: `def filter_valid_urls(urls: List[str], base_domain: str) -> List[str]`
  - **Logic**:
    - Remove duplicate URLs
    - Remove non-HTTP(S) URLs
    - Remove URLs from different domains
    - Remove fragment-only URLs (#anchors)
    - Remove query-only variations
  - **Acceptance**: Function returns deduplicated, valid URLs only

- [ ] T017 [US1] Implement content validation
  - **File**: `backend/main.py`
  - **Function**: `def validate_extracted_content(content: Dict[str, str]) -> bool`
  - **Logic**:
    - Check URL is valid
    - Check title is not empty
    - Check text_content has at least 100 characters
    - Return True if valid, False otherwise
  - **Acceptance**: Function validates extracted content meets minimum requirements

- [ ] T018 [US1] Add ingestion logging for URL processing
  - **File**: `backend/main.py`
  - **Add throughout Phase 2 functions**:
    - Log start of URL discovery
    - Log number of URLs found
    - Log each URL fetch attempt
    - Log successful extractions vs failures
    - Log validation results
  - **Acceptance**: All URL processing activities logged with INFO level

**Phase 2 Acceptance Criteria**:
- ✅ System discovers all URLs from sitemap
- ✅ System extracts readable content from each URL
- ✅ System handles fetch errors gracefully
- ✅ System validates extracted content
- ✅ US1 independent test passes

---

## Phase 3: US2 - Embeddings Generation and Storage

**Goal**: Implement text chunking, embedding generation, and vector storage
**Duration**: 1 day
**Dependencies**: Phase 2
**User Story**: US2 (P2) - As a system administrator, I want content converted to embeddings and stored

### Independent Test Criteria

**Test**: The system can take extracted text content, break it into chunks, generate vector embeddings, and store them in a vector database with associated metadata.

**Validation**:
1. Provide extracted text content as input
2. Verify text is chunked into appropriate segments
3. Verify embeddings generated for each chunk
4. Verify vectors stored in Qdrant with metadata
5. Verify stored vectors can be retrieved

### Tasks

- [ ] T019 [US2] Implement chunk_text function
  - **File**: `backend/main.py`
  - **Function**: `def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]`
  - **Logic**:
    - Split text into chunks of ~chunk_size characters
    - Use overlap between chunks for context preservation
    - Split at sentence boundaries when possible
    - Return list of text chunks
  - **Acceptance**: Function chunks text with specified size and overlap

- [ ] T020 [US2] Implement embed function using Cohere
  - **File**: `backend/main.py`
  - **Function**: `def embed(texts: List[str]) -> List[List[float]]`
  - **Logic**:
    - Call Cohere embed API with text list
    - Use embed-english-v3.0 model
    - Input type: "search_document"
    - Return list of 1024-dim vectors
  - **Error Handling**: Retry on rate limit, log errors
  - **Acceptance**: Function generates 1024-dim embeddings using Cohere

- [ ] T021 [US2] Add batch embedding with rate limiting
  - **File**: `backend/main.py`
  - **Modify**: embed function
  - **Add**: Process in batches of 96 (Cohere limit)
  - **Add**: Rate limiting with asyncio.sleep between batches
  - **Logic**:
    ```python
    async def embed_with_rate_limit(texts: List[str], batch_size: int = 96) -> List[List[float]]:
        all_embeddings = []
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i+batch_size]
            response = co.embed(texts=batch, model='embed-english-v3.0', input_type='search_document')
            all_embeddings.extend(response.embeddings)
            if i + batch_size < len(texts):
                await asyncio.sleep(1)  # Rate limiting
        return all_embeddings
    ```
  - **Acceptance**: Function processes large text lists in batches with rate limiting

- [ ] T022 [US2] Implement save_chunk_to_qdrant function
  - **File**: `backend/main.py`
  - **Function**: `async def save_chunk_to_qdrant(chunk: str, embedding: List[float], metadata: Dict[str, Any])`
  - **Logic**:
    - Generate unique ID using uuid
    - Create PointStruct with id, vector, payload
    - Payload includes: text, source_url, source_title, chunk_index, created_at
    - Upsert point to Qdrant collection
  - **Error Handling**: Retry on connection errors
  - **Acceptance**: Function stores vector with metadata in Qdrant

- [ ] T023 [US2] Implement batch vector storage
  - **File**: `backend/main.py`
  - **Function**: `async def batch_save_to_qdrant(chunks: List[str], embeddings: List[List[float]], metadata_list: List[Dict])`
  - **Logic**:
    - Prepare list of PointStruct objects
    - Batch upsert to Qdrant (100 points per batch)
    - Log successful storage
  - **Acceptance**: Function stores multiple vectors in batches efficiently

- [ ] T024 [US2] Create process_content_to_vectors function
  - **File**: `backend/main.py`
  - **Function**: `async def process_content_to_vectors(content: Dict[str, str])`
  - **Logic**:
    - Extract text from content dict
    - Chunk text using chunk_text()
    - Generate embeddings for all chunks
    - Prepare metadata for each chunk
    - Batch save to Qdrant
    - Log completion with chunk count
  - **Acceptance**: Function orchestrates chunking → embedding → storage pipeline

- [ ] T025 [US2] Add vector storage validation
  - **File**: `backend/main.py`
  - **Function**: `async def validate_vector_storage(point_id: str) -> bool`
  - **Logic**:
    - Retrieve point from Qdrant by ID
    - Verify vector dimensions (1024)
    - Verify metadata exists
    - Return True if valid
  - **Acceptance**: Function validates stored vectors

- [ ] T026 [US2] Add embedding error handling and logging
  - **File**: `backend/main.py`
  - **Add throughout Phase 3 functions**:
    - Log chunk count
    - Log embedding generation start/completion
    - Log storage operations
    - Log errors with context (URL, chunk index)
  - **Acceptance**: All embedding operations logged appropriately

**Phase 3 Acceptance Criteria**:
- ✅ Text chunked into appropriate segments
- ✅ Embeddings generated using Cohere
- ✅ Vectors stored in Qdrant with metadata
- ✅ Batch processing implemented for efficiency
- ✅ US2 independent test passes

---

## Phase 4: US3 - Pipeline Monitoring and Reliability

**Goal**: Implement logging, error handling, and monitoring capabilities
**Duration**: 0.5 day
**Dependencies**: Phase 2, Phase 3
**User Story**: US3 (P3) - As an operations team member, I want reliable pipeline execution

### Independent Test Criteria

**Test**: The pipeline runs automatically, logs activities, and handles errors gracefully.

**Validation**:
1. Trigger pipeline execution
2. Verify all operations logged (INFO, WARNING, ERROR levels)
3. Verify errors handled without crashing
4. Verify pipeline completes successfully

### Tasks

- [ ] T027 [US3] Create main ingestion pipeline function
  - **File**: `backend/main.py`
  - **Function**: `async def run_ingestion_pipeline(target_url: str = None)`
  - **Logic**:
    - Use target_url or env variable
    - Create Qdrant collection if needed
    - Get all URLs from target site
    - Process URLs in batches
    - Extract content from each URL
    - Process content to vectors
    - Log summary statistics
    - Return status dict with processed count
  - **Acceptance**: Function orchestrates complete ingestion pipeline

- [ ] T028 [US3] Add comprehensive error handling to pipeline
  - **File**: `backend/main.py`
  - **Modify**: run_ingestion_pipeline
  - **Add**: Try-except blocks for each major step
  - **Logic**:
    - Catch and log URL fetch errors
    - Catch and log embedding generation errors
    - Catch and log storage errors
    - Continue processing on non-fatal errors
    - Return summary with success/failure counts
  - **Acceptance**: Pipeline continues despite individual failures

- [ ] T029 [US3] Implement pipeline execution logging
  - **File**: `backend/main.py`
  - **Add to run_ingestion_pipeline**:
    - Log pipeline start with timestamp
    - Log each phase (URL discovery, extraction, embedding, storage)
    - Log progress (X of Y URLs processed)
    - Log completion with summary stats
    - Log total duration
  - **Acceptance**: Pipeline execution fully logged with timestamps and progress

- [ ] T030 [US3] Add FastAPI endpoints for pipeline control
  - **File**: `backend/main.py`
  - **Endpoints**:
    ```python
    @app.post("/ingest")
    async def trigger_ingestion():
        """Trigger the ingestion pipeline"""
        try:
            result = await run_ingestion_pipeline(TARGET_WEBSITE_URL)
            return {"status": "success", "message": "Ingestion completed", "result": result}
        except Exception as e:
            logger.error(f"Ingestion failed: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))

    @app.get("/verify")
    async def verify_storage():
        """Verify stored embeddings in Qdrant"""
        try:
            collection_info = qdrant_client.get_collection(COLLECTION_NAME)
            return {
                "status": "success",
                "collection_name": COLLECTION_NAME,
                "vectors_count": collection_info.vectors_count,
                "points_count": collection_info.points_count
            }
        except Exception as e:
            logger.error(f"Verification failed: {str(e)}")
            raise HTTPException(status_code=500, detail=str(e))

    @app.get("/health")
    async def health_check():
        """Health check endpoint"""
        return {"status": "healthy", "service": "vector-ingestion-pipeline"}
    ```
  - **Acceptance**: POST /ingest triggers pipeline, GET /verify shows storage stats, GET /health responds

**Phase 4 Acceptance Criteria**:
- ✅ Pipeline runs end-to-end successfully
- ✅ All operations logged appropriately
- ✅ Errors handled gracefully
- ✅ FastAPI endpoints functional
- ✅ US3 independent test passes

---

## Phase 5: Polish & Integration

**Goal**: Finalize implementation, add documentation, and validate deployment readiness
**Duration**: 0.5 day
**Dependencies**: Phase 4

### Tasks

- [ ] T031 Add inline documentation and code comments
  - **File**: `backend/main.py`
  - **Add**:
    - Docstrings for all functions
    - Comments for complex logic
    - Type hints for function parameters
  - **Acceptance**: All functions documented with clear docstrings

- [ ] T032 Validate end-to-end pipeline execution
  - **Action**: Run complete pipeline on target website
  - **Verify**:
    - All URLs discovered from sitemap
    - Content extracted successfully
    - Embeddings generated
    - Vectors stored in Qdrant
    - API endpoints respond correctly
  - **Log**: Record execution time and success metrics
  - **Acceptance**: Pipeline completes successfully, all vectors stored

**Phase 5 Acceptance Criteria**:
- ✅ Code fully documented
- ✅ End-to-end execution validated
- ✅ Ready for production deployment

---

## Dependency Graph

```
Phase 0 (Setup)
  └─> Phase 1 (Foundational)
       ├─> Phase 2 (US1 - Content Ingestion)
       │    └─> Phase 3 (US2 - Embeddings & Storage)
       │         └─> Phase 4 (US3 - Monitoring)
       │              └─> Phase 5 (Polish)
       └─> Phase 3 (US2 - can start after Phase 1 if test data available)
```

**User Story Dependencies**:
- US1 (P1) is foundational - must complete first
- US2 (P2) depends on US1 (needs extracted content)
- US3 (P3) depends on US1 and US2 (orchestrates complete pipeline)

---

## Parallel Execution Opportunities

Tasks marked with **[P]** can be executed in parallel within their phase:
- Phase 0: T004, T005 (environment files)
- No other parallelization within phases due to sequential dependencies

**Between Phases**: Once US1 is complete, US2 can begin immediately

---

## Implementation Strategy

### MVP Scope (US1 Only)
**Duration**: 1.5 days
**Tasks**: T001-T018 (Phases 0-2)
**Deliverable**: System can discover URLs and extract content

### Incremental Delivery
1. **Week 1**: Complete US1 (Phases 0-2) - Content ingestion working
2. **Week 1-2**: Add US2 (Phase 3) - Embeddings and storage functional
3. **Week 2**: Add US3 (Phase 4) - Monitoring and API complete
4. **Week 2**: Polish (Phase 5) - Production ready

### Testing Strategy
- **US1 Test**: Run ingestion, verify content extraction
- **US2 Test**: Provide sample content, verify vectors in Qdrant
- **US3 Test**: Trigger via API, verify logs and error handling
- **Integration Test**: Run complete pipeline end-to-end

---

## Success Criteria

### Per User Story
- **US1**: ✅ All URLs discovered, content extracted cleanly
- **US2**: ✅ Text chunked, embeddings generated, vectors stored
- **US3**: ✅ Pipeline runs reliably, errors handled, operations logged

### Overall
- ✅ 32/32 tasks completed
- ✅ All 3 user stories independently testable
- ✅ Complete pipeline executes successfully
- ✅ API endpoints functional
- ✅ Code documented and production-ready

---

## File Structure After Completion

```
backend/
├── main.py                 # Complete implementation (T007-T032)
├── pyproject.toml          # Dependencies (T002)
├── requirements.txt        # Generated requirements (T003)
├── README.md              # Setup guide (T006)
├── .env.example           # Template (T004)
└── .env                   # Local config (T005, gitignored)
```

**Single File Implementation**: All code in `backend/main.py` as specified in plan.md

---

## Next Steps

1. **Execute tasks sequentially** by phase (T001 → T032)
2. **Test after each phase** using independent test criteria
3. **Mark tasks complete** with [x] in tasks.md
4. **Validate MVP** after Phase 2 (US1)
5. **Deploy to production** after Phase 5
