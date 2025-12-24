# Task Breakdown: OpenAI Agents SDK RAG Agent

**Feature**: 3-openai-agent | **Branch**: `3-openai-agent` | **Date**: 2025-12-15
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

## Overview

This document breaks down the implementation of the OpenAI Agents SDK RAG Agent into executable tasks organized by user story. Each phase represents a complete, independently testable increment.

**User Stories**:
- **US1 (P1)**: Full-Book Question Answering - Core RAG with retrieval and citations
- **US2 (P2)**: Context-Constrained Answering - Selected text explanations without retrieval
- **US3 (P2)**: Multi-Turn Conversational Context - Follow-up questions with history
- **US4 (P1)**: Citation and Source Transparency - Verifiable sources for all claims
- **US5 (P3)**: Retrieval Quality Validation - Test suite for agent behavior

**Total Tasks**: 56 tasks across 9 phases
**Parallelizable Tasks**: 23 tasks marked with [P]
**MVP Scope**: Phase 1-4 (Setup + Foundational + US1 + US4) = 28 tasks

---

## Implementation Strategy

### MVP-First Approach (Recommended)

**MVP = US1 + US4** (Full-Book Question Answering + Citations)
- **Phases**: 1 (Setup) + 2 (Foundational) + 3 (US1) + 4 (US4)
- **Tasks**: 28 tasks
- **Deliverable**: Working agent that answers questions with cited sources
- **Timeline**: ~3-5 days of focused development

**After MVP**:
- Add US2 (Context-Constrained) - 6 tasks
- Add US3 (Multi-Turn Context) - 4 tasks (requires minimal work, mostly enabled by SQLiteSession)
- Add US5 (Validation Suite) - 6 tasks

### Incremental Delivery

Each user story is independently testable and can be delivered separately:

1. **Week 1**: MVP (US1 + US4) - Get basic agent working with citations
2. **Week 2**: US2 (Context-Constrained) - Add dual-mode support
3. **Week 3**: US3 (Multi-Turn) + US5 (Validation) - Polish and test

### Parallel Execution Opportunities

Tasks marked with **[P]** can be executed in parallel with other tasks in the same phase or across phases (if dependencies are met).

**Example Parallel Sets**:
- Phase 2 (Foundational): T009, T010, T011 can run in parallel (different files)
- Phase 3 (US1): T015, T016, T017 can run in parallel (models in backend/models.py)
- Phase 5 (US2): T031, T032 can run in parallel (different endpoint logic)

---

## Phase 1: Setup and Dependencies

**Objective**: Install dependencies, configure environment, and verify connectivity.

**Prerequisites**: None (start here)

**Acceptance Criteria**:
- ✅ All packages installed without conflicts
- ✅ OpenAI API key validated with test call
- ✅ Environment variables loaded correctly
- ✅ Project structure ready for implementation

### Tasks

- [ ] T001 Add openai-agents>=0.6.3 and httpx>=0.26.0 to backend/requirements.txt
- [ ] T002 Install new dependencies with pip install openai-agents httpx
- [ ] T003 Add OPENAI_API_KEY environment variable to backend/.env
- [ ] T004 Create backend/data/ directory for conversations database
- [ ] T005 [P] Verify OpenAI API connectivity with test script (create backend/scripts/test_openai.py)
- [ ] T006 [P] Verify httpx installation with import test

**Validation**:
```bash
# Test imports
python -c "import agents; import httpx; print('✓ Dependencies ready')"

# Test OpenAI API
python backend/scripts/test_openai.py
# Expected: "✓ OpenAI API connected"

# Verify environment
python -c "import os; from dotenv import load_dotenv; load_dotenv(); print('✓ OPENAI_API_KEY loaded' if os.getenv('OPENAI_API_KEY') else '✗ Missing API key')"
```

---

## Phase 2: Foundational Layer

**Objective**: Build core components needed by all user stories (retrieval tool, agent config, Pydantic models).

**Prerequisites**: Phase 1 complete

**Acceptance Criteria**:
- ✅ Retrieval tool calls /search endpoint successfully
- ✅ Tool formats results with [Source N] markers
- ✅ Agent initialized with grounding instructions
- ✅ All Pydantic models defined with validation

### Tasks

#### Retrieval Tool (US1, US4 dependency)

- [ ] T007 Create backend/tools.py module with imports (agents, httpx, typing)
- [ ] T008 [US1] Implement retrieve_book_content function with @function_tool decorator in backend/tools.py
- [ ] T009 [P] [US1] Add httpx POST call to http://localhost:8000/search in retrieve_book_content
- [ ] T010 [P] [US1] Implement format_retrieval_results helper function in backend/tools.py to format chunks with [Source N] markers
- [ ] T011 [P] [US1] Add custom error handler handle_retrieval_error function in backend/tools.py with failure_error_function parameter

#### Agent Configuration (US1, US4 dependency)

- [ ] T012 Create backend/agent_config.py module for agent setup
- [ ] T013 [US1] [US4] Define GROUNDING_INSTRUCTIONS string constant in backend/agent_config.py with full grounding policy (from research.md RQ3)
- [ ] T014 [US1] Initialize agent instance with instructions, tools=[retrieve_book_content], model="gpt-4o" in backend/agent_config.py

#### Pydantic Models (All user stories dependency)

- [ ] T015 [P] Create backend/models.py module with Pydantic imports
- [ ] T016 [P] Implement ChatRequest model in backend/models.py with message, session_id fields
- [ ] T017 [P] Implement ChatRequestWithContext model in backend/models.py extending ChatRequest with optional context_text field
- [ ] T018 [P] [US4] Implement Citation model in backend/models.py with source_number, url, score fields
- [ ] T019 [P] Implement ChatResponse model in backend/models.py with response, session_id, status fields
- [ ] T020 [P] [US4] Implement GroundedResponse model in backend/models.py with answer, sources, confidence, retrieval_mode fields
- [ ] T021 [P] Implement ConversationMessage model in backend/models.py with role, content, timestamp, tool_call_id fields
- [ ] T022 [P] Implement ErrorDetail and ErrorResponse models in backend/models.py
- [ ] T023 [P] Add field validators to models (session_id pattern, context_text non-empty check)

**Validation**:
```python
# Test retrieval tool
from backend.tools import retrieve_book_content
result = await retrieve_book_content("What is ZMP?", top_k=3)
assert "[Source 1]" in result
assert "URL:" in result

# Test agent initialization
from backend.agent_config import agent
assert agent.name == "Humanoid Robotics Assistant"
assert len(agent.tools) == 1

# Test models
from backend.models import ChatRequest
request = ChatRequest(message="test", session_id="test_123")
assert request.message == "test"
```

---

## Phase 3: User Story 1 - Full-Book Question Answering (P1)

**Goal**: Implement core RAG functionality with retrieval-first answering.

**Prerequisites**: Phase 2 complete

**Independent Test**:
```bash
# Test via FastAPI /docs
POST /agent/chat
{
  "message": "What is inverse kinematics?",
  "session_id": "test_us1",
  "context_text": null
}

# Verify response includes:
# 1. Agent calls retrieval tool (check logs)
# 2. Answer based on retrieved chunks
# 3. Inline citations [Source 1], [Source 2]
# 4. Sources section at end with URLs
```

**Acceptance Criteria**:
- ✅ Agent calls retrieval tool for every full-book query
- ✅ Responses grounded in retrieved content (no hallucination)
- ✅ Error handling graceful for retrieval failures
- ✅ Endpoint returns 200 with ChatResponse

### Tasks

- [ ] T024 [US1] Import agent, models, and SQLiteSession in backend/main.py
- [ ] T025 [US1] Implement POST /agent/chat endpoint in backend/main.py
- [ ] T026 [US1] Add SQLiteSession initialization with request.session_id and db_path="data/conversations.db"
- [ ] T027 [US1] Implement basic message passing (no dual-mode yet) to Runner.run with agent, message, session, max_turns=10
- [ ] T028 [US1] Add error handling try/except block for InputGuardrailTripwireTriggered, OutputGuardrailTripwireTriggered, Exception
- [ ] T029 [US1] Return ChatResponse with result.final_output, session_id, status="success"
- [ ] T030 [US1] Test endpoint via FastAPI /docs with sample question and verify retrieval tool called

**Code Reference**:
```python
# backend/main.py
from agents import SQLiteSession, Runner
from backend.agent_config import agent
from backend.models import ChatRequestWithContext, ChatResponse

@app.post("/agent/chat", response_model=ChatResponse)
async def chat(request: ChatRequestWithContext):
    try:
        session = SQLiteSession(request.session_id, "data/conversations.db")
        result = await Runner.run(agent, request.message, session=session, max_turns=10)
        return ChatResponse(
            response=result.final_output,
            session_id=request.session_id,
            status="success"
        )
    except Exception as e:
        # Error handling
        ...
```

---

## Phase 4: User Story 4 - Citation and Source Transparency (P1)

**Goal**: Ensure all responses include verifiable citations with URLs.

**Prerequisites**: Phase 2 + Phase 3 complete (US1 working)

**Independent Test**:
```bash
# Ask any question
POST /agent/chat
{
  "message": "How does ZMP work?",
  "session_id": "test_us4",
  "context_text": null
}

# Verify response format:
# 1. Inline citations: [Source 1], [Source 2]
# 2. Sources section at end:
#    Sources:
#    - [Source 1] https://example.com/chapter3
#    - [Source 2] https://example.com/chapter5
# 3. Every factual claim has citation
```

**Acceptance Criteria**:
- ✅ 100% of responses include inline citations
- ✅ Sources section lists all URLs
- ✅ Citations match retrieved chunk metadata
- ✅ No responses with uncited claims

### Tasks

- [ ] T031 [P] [US4] Verify GROUNDING_INSTRUCTIONS in backend/agent_config.py includes explicit citation requirements
- [ ] T032 [P] [US4] Verify format_retrieval_results in backend/tools.py includes "Instructions: Reference sources using [Source 1], [Source 2]" reminder
- [ ] T033 [US4] Test agent responses manually and verify citation format matches spec
- [ ] T034 [US4] Add optional output_guardrail for citation validation in backend/agent_config.py (create citation_guardrail function)
- [ ] T035 [US4] Test refusal behavior: ask off-topic question and verify agent states "I don't have information about this in the book"

**Validation**:
```python
# Test citation presence
response = client.post("/agent/chat", json={...})
data = response.json()
assert "[Source" in data["response"]
assert "Sources:" in data["response"]
assert "http" in data["response"]  # URL present

# Test refusal
response = client.post("/agent/chat", json={"message": "What is quantum entanglement?", ...})
data = response.json()
assert "don't have information" in data["response"].lower()
assert "[Source" not in data["response"]  # No citations for refusal
```

---

## Phase 5: User Story 2 - Context-Constrained Answering (P2)

**Goal**: Support selected-text explanations without calling retrieval tool.

**Prerequisites**: Phase 3 complete (US1 working)

**Independent Test**:
```bash
# Provide selected text
POST /agent/chat
{
  "message": "Explain this in simple terms",
  "session_id": "test_us2",
  "context_text": "The zero-moment point (ZMP) is the point on the ground where the total sum of inertial and gravitational forces equals zero. It is used in humanoid robot control to maintain balance during locomotion."
}

# Verify:
# 1. Agent does NOT call retrieval tool (check logs)
# 2. Response explains the provided text
# 3. No Sources section (no retrieval performed)
# 4. Simpler language used
```

**Acceptance Criteria**:
- ✅ Agent detects context_text presence correctly
- ✅ No retrieval tool calls when context_text provided
- ✅ Response references provided text
- ✅ Dual-mode behavior works independently

### Tasks

- [ ] T036 [US2] Implement dual-mode logic in /agent/chat endpoint: check if request.context_text is not None
- [ ] T037 [US2] Prepend context-constrained mode instruction to message when context_text provided: "[CONTEXT-CONSTRAINED MODE] Given text: {context_text}\\nUser question: {message}"
- [ ] T038 [US2] Test full-book mode (context_text=null) and verify retrieval tool called
- [ ] T039 [US2] Test context-constrained mode (context_text="...") and verify NO retrieval tool called
- [ ] T040 [US2] Verify agent responses differ appropriately between modes
- [ ] T041 [US2] Add validation: if context_text provided but user asks about unrelated topic, agent clarifies limitation

**Code Reference**:
```python
# backend/main.py - dual-mode logic
if request.context_text:
    # Context-constrained mode
    enhanced_message = (
        f"[CONTEXT-CONSTRAINED MODE]\n"
        f"Given text: {request.context_text}\n\n"
        f"User question: {request.message}"
    )
else:
    # Full-book mode
    enhanced_message = request.message

result = await Runner.run(agent, enhanced_message, session=session, max_turns=10)
```

---

## Phase 6: User Story 3 - Multi-Turn Conversational Context (P2)

**Goal**: Enable follow-up questions with maintained conversation history.

**Prerequisites**: Phase 3 complete (US1 with SQLiteSession already enables this)

**Independent Test**:
```bash
# Turn 1
POST /agent/chat
{
  "message": "What is the zero-moment point?",
  "session_id": "test_us3_conv1",
  "context_text": null
}

# Turn 2 (follow-up)
POST /agent/chat
{
  "message": "How is it calculated?",
  "session_id": "test_us3_conv1",  # Same session_id
  "context_text": null
}

# Verify:
# 1. Agent understands "it" refers to ZMP from turn 1
# 2. Response builds on previous context
# 3. Still includes citations
```

**Acceptance Criteria**:
- ✅ Agent maintains context across 5+ turns
- ✅ Follow-up questions understood correctly
- ✅ Conversation history retrieved automatically
- ✅ Each response still grounded in retrieved content

### Tasks

- [ ] T042 [US3] Verify SQLiteSession automatically saves/retrieves history (already implemented in Phase 3)
- [ ] T043 [US3] Test multi-turn conversation with 3-5 follow-up questions
- [ ] T044 [US3] Verify agent understands pronoun references ("it", "this", "that")
- [ ] T045 [US3] Test conversation context retention: ask related questions in sequence and verify coherence

**Note**: US3 is largely enabled by SQLiteSession in Phase 3. These tasks are validation-focused.

---

## Phase 7: Session Management Endpoints

**Goal**: Provide endpoints for viewing and clearing conversation history.

**Prerequisites**: Phase 3 complete (SQLiteSession in use)

**Acceptance Criteria**:
- ✅ GET /agent/session/{id} returns full history
- ✅ DELETE /agent/session/{id} clears session
- ✅ 404 returned for non-existent sessions
- ✅ Timestamps included for all messages

### Tasks

- [ ] T046 Implement GET /agent/session/{session_id} endpoint in backend/main.py
- [ ] T047 Retrieve session items with session.get_items(limit=50)
- [ ] T048 Format response with ConversationMessage list and total_count
- [ ] T049 Add error handling: return 404 if session not found
- [ ] T050 Implement DELETE /agent/session/{session_id} endpoint in backend/main.py
- [ ] T051 Call session.clear_session() and return 204 No Content
- [ ] T052 Test both endpoints via /docs interface

**Code Reference**:
```python
@app.get("/agent/session/{session_id}")
async def get_session(session_id: str):
    session = SQLiteSession(session_id, "data/conversations.db")
    history = await session.get_items(limit=50)
    return {
        "session_id": session_id,
        "messages": history,
        "total_count": len(history)
    }

@app.delete("/agent/session/{session_id}", status_code=204)
async def clear_session(session_id: str):
    session = SQLiteSession(session_id, "data/conversations.db")
    await session.clear_session()
```

---

## Phase 8: Health Check and Monitoring

**Goal**: Add service health endpoint for monitoring dependencies.

**Prerequisites**: Phase 3 complete (agent and /search dependency)

**Acceptance Criteria**:
- ✅ GET /health returns dependency statuses
- ✅ Returns 200 if all healthy, 503 if any unhealthy
- ✅ Individual checks for OpenAI API, /search, SQLite
- ✅ Useful for load balancer health checks

### Tasks

- [ ] T053 Create backend/models.py HealthResponse model with status, timestamp, checks, version fields
- [ ] T054 Implement GET /health endpoint in backend/main.py
- [ ] T055 Add health check functions: check_openai_api(), check_search_endpoint(), check_sqlite_database()
- [ ] T056 Return HealthResponse with 200 if all healthy, 503 if any unhealthy

**Code Reference**:
```python
@app.get("/health", response_model=HealthResponse)
async def health_check():
    checks = {
        "openai_api": await check_openai_api(),
        "search_endpoint": await check_search_endpoint(),
        "session_database": check_sqlite_database()
    }

    status = "healthy" if all(v == "healthy" for v in checks.values()) else "unhealthy"
    status_code = 200 if status == "healthy" else 503

    return Response(
        content=HealthResponse(
            status=status,
            timestamp=datetime.utcnow(),
            checks=checks,
            version="1.0.0"
        ).model_dump_json(),
        status_code=status_code,
        media_type="application/json"
    )
```

---

## Phase 9: User Story 5 - Retrieval Quality Validation (P3)

**Goal**: Create test suite to validate agent behavior and quality.

**Prerequisites**: All previous phases complete

**Independent Test**:
```bash
# Run test suite
pytest backend/tests/test_agent.py -v

# Expected: All tests pass
# - test_grounding_policy: 10/10 pass (no hallucination)
# - test_citation_requirements: 10/10 pass (all have citations)
# - test_dual_mode: 5/5 pass (correct mode selection)
# - test_error_handling: 5/5 pass (graceful failures)
# - test_multi_turn: 5/5 pass (context maintained)
```

**Acceptance Criteria**:
- ✅ 100% of test queries trigger retrieval (full-book mode)
- ✅ 100% of responses include citations
- ✅ No hallucination detected in responses
- ✅ Error messages user-friendly

### Tasks

- [ ] T057 [P] [US5] Create backend/tests/test_agent.py with pytest setup
- [ ] T058 [P] [US5] Implement test_grounding_policy: 10 questions, verify retrieval called and no hallucination
- [ ] T059 [P] [US5] Implement test_citation_requirements: verify all responses have [Source N] and Sources: section
- [ ] T060 [P] [US5] Implement test_dual_mode_behavior: test with and without context_text, verify correct behavior
- [ ] T061 [P] [US5] Implement test_error_handling: test retrieval failures, invalid inputs, off-topic questions
- [ ] T062 [P] [US5] Implement test_multi_turn_conversations: 5 turn conversation, verify context maintained

**Test Examples**:
```python
# backend/tests/test_agent.py
import pytest
from fastapi.testclient import TestClient
from backend.main import app

client = TestClient(app)

def test_grounding_policy():
    """Test agent refuses to answer off-topic questions"""
    response = client.post("/agent/chat", json={
        "message": "What is quantum entanglement?",
        "session_id": "test_grounding",
        "context_text": None
    })
    data = response.json()
    assert "don't have information" in data["response"].lower()
    assert "[Source" not in data["response"]

def test_citation_requirements():
    """Test all responses include citations"""
    response = client.post("/agent/chat", json={
        "message": "What is inverse kinematics?",
        "session_id": "test_citations",
        "context_text": None
    })
    data = response.json()
    assert "[Source" in data["response"]
    assert "Sources:" in data["response"]
    assert "http" in data["response"]  # URL present

# ... more tests
```

---

## Dependency Graph

### User Story Completion Order

```
Setup (Phase 1)
    ↓
Foundational (Phase 2) - Retrieval Tool, Agent Config, Models
    ↓
    ├─→ US1 (Phase 3) - Full-Book Question Answering [P1] [BLOCKING MVP]
    │       ↓
    │   US4 (Phase 4) - Citation Transparency [P1] [BLOCKING MVP]
    │       ↓
    │   ┌───┴────┐
    │   ↓        ↓
    │  US2      US3 (Phase 6) - Multi-Turn Context [P2]
    │  (Phase 5) [P2]
    │  Context-
    │  Constrained
    │       ↓
    └───────┴────→ Session Management (Phase 7)
                       ↓
                   Health Check (Phase 8)
                       ↓
                   US5 (Phase 9) - Validation Suite [P3]
```

**Critical Path**: Phase 1 → Phase 2 → Phase 3 (US1) → Phase 4 (US4)
**Parallel Opportunities**: US2 and US3 can be developed in parallel after US1

---

## Parallel Execution Examples

### By User Story

**US1 (Full-Book)** - Phase 3:
```
Parallel Group 1:
- T024 (import agent/models)
- T025 (implement endpoint)

Parallel Group 2 (after Group 1):
- T026 (SQLiteSession init)
- T027 (Runner.run logic)
- T028 (error handling)
```

**US2 (Context-Constrained)** - Phase 5:
```
Can run entirely in parallel with US3 (if US1 complete)
All tasks T036-T041 depend on US1 but not on each other
```

**US5 (Validation)** - Phase 9:
```
All test tasks can run in parallel:
- T058 (grounding tests)
- T059 (citation tests)
- T060 (dual-mode tests)
- T061 (error tests)
- T062 (multi-turn tests)
```

### By File

**backend/models.py**:
```
All model tasks can run in parallel:
- T016, T017, T018, T019, T020, T021, T022, T023
```

**backend/tools.py**:
```
Sequential within tool implementation:
T007 → T008 → (T009, T010, T011 in parallel)
```

**backend/main.py**:
```
Endpoints can be added in parallel:
- T025-T029 (chat endpoint)
- T046-T049 (session GET)
- T050-T051 (session DELETE)
- T054-T056 (health endpoint)
```

---

## File Structure After Completion

```
backend/
├── main.py                    # FastAPI app with all endpoints
├── agent_config.py            # Agent initialization and GROUNDING_INSTRUCTIONS
├── tools.py                   # retrieve_book_content tool
├── models.py                  # All Pydantic models
├── data/
│   └── conversations.db       # SQLite session storage (auto-created)
├── scripts/
│   └── test_openai.py         # OpenAI API connectivity test
├── tests/
│   └── test_agent.py          # Comprehensive test suite
├── requirements.txt           # Updated with openai-agents, httpx
└── .env                       # OPENAI_API_KEY added
```

---

## Task Completion Checklist

Use this checklist to track progress:

**Phase 1 (Setup)**: ☐ 6 tasks
**Phase 2 (Foundational)**: ☐ 17 tasks
**Phase 3 (US1)**: ☐ 7 tasks
**Phase 4 (US4)**: ☐ 5 tasks
**Phase 5 (US2)**: ☐ 6 tasks
**Phase 6 (US3)**: ☐ 4 tasks
**Phase 7 (Session Mgmt)**: ☐ 7 tasks
**Phase 8 (Health)**: ☐ 4 tasks
**Phase 9 (US5)**: ☐ 6 tasks

**Total**: ☐ 62 tasks

**MVP Milestone** (US1 + US4): ☐ 35 tasks (Phases 1-4)

---

## Success Metrics

After all tasks complete, verify:

- ✅ **SC-001**: Agent calls retrieval tool for 100% of full-book queries (validated through logging)
- ✅ **SC-002**: Agent responses include citations for 100% of answers (inline and source list)
- ✅ **SC-003**: Test suite of 10 questions achieves 100% grounded responses (no hallucinations detected by human review)
- ✅ **SC-004**: Agent correctly distinguishes between full-book and context-constrained modes with 100% accuracy
- ✅ **SC-005**: Multi-turn conversations maintain context for at least 5 consecutive turns without losing coherence
- ✅ **SC-006**: Agent refuses to answer (states "not in book") for 100% of queries with all retrieval scores <0.4
- ✅ **SC-007**: Agent response latency stays under 10 seconds for typical questions (single retrieval call)
- ✅ **SC-008**: Users can trace every factual claim in agent response to specific source URLs (100% verifiability)

---

**Tasks Generated**: 2025-12-15
**Ready for Implementation**: Use `/sp.implement` to execute tasks sequentially
**Next Step**: Start with Phase 1 (Setup) tasks T001-T006
