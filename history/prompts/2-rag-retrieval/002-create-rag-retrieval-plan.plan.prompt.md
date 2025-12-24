---
id: 002
title: Create RAG Retrieval Implementation Plan
stage: plan
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: 2-rag-retrieval
branch: 2-rag-retrieval
user: wania akram
command: /sp.plan
labels: ["rag", "retrieval", "planning", "architecture", "fastapi"]
links:
  spec: specs/2-rag-retrieval/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/2-rag-retrieval/plan.md
  - specs/2-rag-retrieval/research.md
  - specs/2-rag-retrieval/data-model.md
  - specs/2-rag-retrieval/contracts/api-contract.yaml
  - specs/2-rag-retrieval/quickstart.md
tests:
  - Constitution check: PASSED (no violations)
  - Research phase: 5 research questions resolved
  - Design phase: 8 Pydantic models + OpenAPI spec
---

## Prompt

/sp.plan
- Connect FastAPI backend to Qdrant Cloud collection
- Implement semantic vector search with configurable top-k
- Attach metadata filters for URL and section-based retrieval
- Validate retrieval accuracy using test queries
- Log retrieval results and performance metrics

## Response snapshot

Created comprehensive implementation plan for RAG retrieval pipeline feature with complete research, design, and architecture documentation.

**Planning Outputs (5 files created)**:

1. **research.md** (17,000 words)
   - Investigated existing backend infrastructure
   - Resolved 5 research questions about Qdrant search API, query embedding, performance optimization, error handling, and test design
   - Identified configuration issue (CO_API_KEY → COHERE_API_KEY mismatch)
   - Validated no new dependencies needed
   - Designed 20-query test suite across 4 categories

2. **data-model.md** (8,500 words)
   - Defined 8 Pydantic models (SearchRequest, SearchResponse, RetrievedChunk, TestRetrievalRequest, TestRetrievalResponse, TestResult, HealthResponse, ChunkMetadata)
   - Specified validation rules and constraints
   - Documented data flow and type aliases
   - Designed test query internal structure

3. **contracts/api-contract.yaml** (5,800 words)
   - OpenAPI 3.0 specification with 3 endpoints
   - Complete request/response schemas
   - Examples for all use cases
   - Error response documentation
   - Health check contract

4. **plan.md** (14,500 words)
   - Architecture decisions (6 ADs documented)
   - Phase 0 research summary
   - Phase 1 design artifacts
   - Implementation task breakdown preview
   - Risk assessment and mitigation strategies
   - Success metrics and validation checklist
   - Code examples and deployment guide

5. **quickstart.md** (6,000 words)
   - Setup instructions
   - Usage examples with curl commands
   - Troubleshooting guide
   - Performance expectations
   - Integration patterns for chatbot

**Key Architecture Decisions**:

AD-001: Extend existing FastAPI app (monolithic) vs separate microservice
- **Chosen**: Monolithic (shared clients/config, low traffic)

AD-002: Separate query embedding function with input_type="search_query"
- **Chosen**: New embed_query() function (optimized for queries)

AD-003: No embedding cache initially
- **Chosen**: Skip caching (premature optimization, <100 queries/day)

AD-004: Metadata filtering via Qdrant query_filter
- **Chosen**: Native Qdrant filtering (better performance)

AD-005: Test suite in-code as Python constant
- **Chosen**: 20 queries hardcoded in main.py (simple, version-controlled)

AD-006: Fix configuration naming (COHERE_API_KEY)
- **Chosen**: Standardize on COHERE_API_KEY (consistency)

**Technical Stack Validation**:
- ✅ Python 3.13+ (existing)
- ✅ FastAPI 0.104.0+ (existing)
- ✅ qdrant-client 1.8.0+ (existing)
- ✅ cohere 5.5.0+ (existing)
- ✅ No new dependencies required

**Implementation Approach**:
- Extend backend/main.py with 3 new endpoints
- Add 8 Pydantic models for validation
- Implement embed_query() function
- Add TEST_QUERIES constant (20 entries)
- Fix configuration naming
- ~500 LOC addition, ~11 implementation tasks

**Performance Targets**:
- <2s single query latency (expected: 300-800ms)
- <3s avg for 10 concurrent queries
- 85% test suite success rate (17+/20 queries)
- <500ms health check latency

**Risk Mitigation**:
- Cohere API failures: Retry logic, error handling (503)
- Qdrant connection failures: Health check, graceful degradation
- Low retrieval relevance: Test suite validation, threshold tuning
- Configuration breaking change: Document migration in quickstart

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation. All design decisions documented, no ambiguities remaining.
- 🧪 Tests: Constitution check passed (no violations), 5 research questions resolved, 20-query test suite designed
- 📁 Files: Created plan.md, research.md, data-model.md, api-contract.yaml, quickstart.md (51,800 words total)
- 🔁 Next prompts: `/sp.tasks` to generate detailed task breakdown with acceptance criteria
- 🧠 Reflection: Planning phase successfully bridges specification and implementation. Research validated technical feasibility, design provides clear contracts, architecture decisions are justified with tradeoffs documented.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed without blockers
- Graders run and results (PASS/FAIL): Constitution check PASSED, Research phase COMPLETED (5/5 RQs resolved), Design phase COMPLETED (8 models + OpenAPI spec)
- Prompt variant (if applicable): Standard /sp.plan workflow with user-specified implementation requirements
- Next experiment (smallest change to try): Consider adding architecture decision record (ADR) for significant decisions (AD-001 to AD-006) for long-term documentation
