---
id: 002
title: Create OpenAI Agent Implementation Plan
stage: plan
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5
feature: 3-openai-agent
branch: 3-openai-agent
user: developer
command: /sp.plan
labels: ["planning", "openai-agents-sdk", "rag", "architecture", "retrieval-augmented-generation"]
links:
  spec: ../../../specs/3-openai-agent/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/3-openai-agent/research.md
  - specs/3-openai-agent/data-model.md
  - specs/3-openai-agent/contracts/api-contract.yaml
  - specs/3-openai-agent/plan.md
  - specs/3-openai-agent/quickstart.md
tests:
  - null
---

## Prompt

/sp.plan - Initialize OpenAI Agents SDK within FastAPI backend, Define retrieval tool interface connected to Spec-2 endpoint, Implement agent logic to enforce retrieval-first answering, Add support for selected-text-only question answering, Expose agent interaction endpoint for frontend use

## Response snapshot

Created comprehensive implementation plan for Feature 3 (OpenAI Agents SDK RAG Agent) with the following artifacts:

**1. Research Document (research.md - 17,000 words)**
- 8 research questions resolved using Context7 MCP for OpenAI Agents SDK documentation
- RQ1: SDK capabilities and FastAPI integration strategies
- RQ2: Retrieval tool interface design with @function_tool decorator
- RQ3: System instructions for response grounding and hallucination prevention
- RQ4: Conversation history management with SQLiteSession
- RQ5: Citation extraction and source transparency
- RQ6: Error handling patterns for retrieval failures
- RQ7: Token limits and performance optimization
- RQ8: Streaming responses (optional enhancement)

**2. Data Models (data-model.md - 8,500 words)**
- Request models: ChatRequest, ChatRequestWithContext
- Response models: ChatResponse, GroundedResponse, Citation
- Internal models: ConversationMessage, ErrorDetail, ErrorResponse
- Validation examples and usage patterns
- Complete Pydantic model definitions with field validators

**3. API Contract (contracts/api-contract.yaml)**
- OpenAPI 3.0 specification with 5 endpoints:
  - POST /agent/chat (dual-mode support)
  - POST /agent/chat/structured (optional structured output)
  - GET /agent/session/{session_id} (history retrieval)
  - DELETE /agent/session/{session_id} (clear session)
  - GET /health (service health check)
- Comprehensive request/response examples
- Error response schemas

**4. Implementation Plan (plan.md - 14,500 words)**
- 10 architecture decisions (AD-001 through AD-010):
  - AD-001: Extend existing FastAPI app (monolithic)
  - AD-002: Use @function_tool decorator for retrieval
  - AD-003: Static system instructions for grounding
  - AD-004: SQLiteSession for conversation management
  - AD-005: Dual-mode via single endpoint with optional field
  - AD-006: httpx for async HTTP in retrieval tool
  - AD-007: Agent instance reuse at module level
  - AD-008: GPT-4o model selection
  - AD-009: max_turns=10 limit for safety
  - AD-010: Custom error handler for retrieval tool
- 8-phase implementation strategy with acceptance criteria
- Data flow diagrams (full-book mode vs. context-constrained mode)
- Error handling strategy with 9 error categories
- Non-functional requirements (performance, reliability, security, observability)
- 5 risks identified with mitigation strategies

**5. Quickstart Guide (quickstart.md - 6,000 words)**
- Step-by-step setup instructions (7 steps)
- Environment configuration with OpenAI API key
- Usage examples (cURL, Python, JavaScript)
- Multi-turn conversation examples
- Dual-mode operation testing
- Error handling scenarios
- Troubleshooting guide (6 common issues)
- API reference for all endpoints
- Cost estimation ($42-420/month based on usage)

**Key Technical Decisions**:
- Use Context7 MCP for all OpenAI Agents SDK documentation lookups
- GPT-4o model with 128k context window (no truncation needed)
- Async architecture throughout (FastAPI + httpx + OpenAI SDK)
- Detailed grounding instructions to prevent hallucination
- Citation transparency with [Source N] inline citations + Sources section
- Session-based conversation management (automatic history tracking)
- Graceful error handling with user-friendly messages

**Dependencies Identified**:
- Feature 2 (rag-retrieval) must be complete for /search endpoint
- New packages: openai-agents>=0.6.3, httpx>=0.26.0
- OpenAI API key required (estimated $2-10/day)

## Outcome

- ✅ Impact: Complete planning phase for Feature 3 with all design artifacts ready for implementation. Comprehensive architecture decisions documented with rationale, trade-offs, and implementation guidance. Research leveraged Context7 MCP for authoritative OpenAI Agents SDK documentation.
- 🧪 Tests: No tests run during planning phase. Test strategy defined in plan.md (unit, integration, behavior, performance tests).
- 📁 Files: Created 5 planning documents (research.md, data-model.md, api-contract.yaml, plan.md, quickstart.md) totaling ~52,000 words of comprehensive documentation.
- 🔁 Next prompts: `/sp.tasks` to generate task breakdown from plan.md, then `/sp.implement` to execute implementation tasks.
- 🧠 Reflection: Context7 MCP integration significantly improved research quality by providing up-to-date, authoritative OpenAI Agents SDK documentation. The dual-mode design (full-book vs. context-constrained) elegantly handles both use cases with a single endpoint. Grounding policy is extremely detailed to minimize hallucination risk. Dependency on Feature 2 is clearly documented with mock endpoint fallback for parallel development.

## Evaluation notes (flywheel)

- Failure modes observed: Initial web search attempts interrupted by user requesting Context7 MCP usage. Switched to MCP tools (mcp__context7__resolve-library-id, mcp__context7__get-library-docs) for all documentation lookups, which provided superior code examples and architectural guidance.
- Graders run and results (PASS/FAIL): No automated graders run. Manual validation: Research questions comprehensively answered (PASS), Architecture decisions well-reasoned with trade-offs (PASS), Data models complete with validation (PASS), API contract matches OpenAPI 3.0 spec (PASS), Quickstart guide provides clear setup path (PASS).
- Prompt variant (if applicable): Standard /sp.plan workflow with emphasis on Context7 MCP usage for external documentation. User provided specific implementation areas in prompt (SDK initialization, retrieval tool, grounding enforcement, dual-mode support, endpoint exposure).
- Next experiment (smallest change to try): Consider using structured output (GroundedResponse) as default instead of optional to ensure programmatic citation extraction. Alternatively, implement output guardrail to validate citation presence in all responses.
