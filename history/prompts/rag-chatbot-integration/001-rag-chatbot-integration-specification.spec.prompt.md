---
id: 001
title: RAG Chatbot Integration Specification
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: rag-chatbot-integration
branch: 003-rag-chatbot-integration
user: wania akram
command: /sp.specify
labels: ["rag", "chatbot", "gemini", "qdrant", "neon", "better-auth", "chatkit", "personalization", "translation"]
links:
  spec: specs/003-rag-chatbot-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-rag-chatbot-integration/spec.md
  - specs/003-rag-chatbot-integration/checklists/requirements.md
tests:
  - N/A (specification phase - no tests yet)
---

## Prompt

Title: Integrate RAG + Chatbot into Humanoid Robotics Textbook

We are extending the already-published Physical AI & Humanoid Robotics textbook website by adding a full RAG-powered Chatbot layer embedded inside the existing "book UI".

Primary Goals:
1) Integrated RAG Chatbot Development that is embedded *inside the humanoid robotics book* pages.
2) Full Retrieval-Augmented Generation using existing Qdrant ingestion and retrieval code.
3) Chatbot must use Gemini (via OpenAI Agents SDK) and communicate through custom subagent tools.
4) Build a FastAPI backend exposing:
   - /ingest-book
   - /retrieve
   - /chat
   - /summarize
   - /select-text
   - /history
   - /translation/urdu
   and other agent tool routes.

5) Use ChatKit as the frontend UI widget that opens inside the published book page. (chatkit-js)
6) Use Neon serverless Postgres database for:
   - user profile data
   - software/hardware background
   - session history
   - personalized chapter selections
7) Better-Auth for signup & login:
   At signup ask the following onboarding questions:
   - hardware background
   - software background
   - experience in robotics
   - programming level
   This data lives in Neon and is used for personalizing agent responses.

8) Logged-in users will see two new buttons at the start of each chapter:
   - "Personalize this chapter"
   - "Translate in Urdu"
   When clicked, we call FastAPI endpoints which call Gemini agent tools that re-render that chapter content based on user background, or translate the chapter into Urdu.

9) User text selection inside the chapter:
   If a user selects text in the page UI and clicks "Summarize Selected Text", invoke a chat agent tool that retrieves only from selected text and returns a summary directly in chat.

10) Store RAG chunks in Qdrant Cloud Free Tier as already implemented using provided code.
    Use the existing:
       - agent.py
       - retrieval.py
       - main ingestion pipeline
    We will upgrade naming to one unified collection name "humanoid_ai_book".

11) All agent calls must FIRST call the retrieval tool, and are not allowed to hallucinate outside retrieved content. If the answer is not present, respond: "I don't know based on the book".

12) Create Subagents using Claude Code subagent functionality:
    - RAG Retrieval Agent
    - Summarization Agent
    - Text Selection Agent
    - Urdu Translation Agent
    - Personalization Agent
    - Chapter Assistant Agent
    Each subagent has isolated skills and tools, shared via the book runtime.

13) Implement persistent memory:
    - store conversations in Neon
    - use conversationId per user session
    - ability to view previous chat history inside ChatKit

14) ChatKit must appear floating on every chapter page with:
    - open window
    - minimize
    - history tab
    - "personalize" contextual actions
    - show retrieved paragraph sources

15) Backend Stack Required:
   - FastAPI app
   - Better-Auth integration
   - Neon Postgres Python driver
   - Qdrant client

## Response snapshot

Created comprehensive specification for RAG-powered chatbot integration with the following structure:

**User Stories (6 total, prioritized P1-P4)**:
1. P1 - Interactive Chapter Q&A: Core chatbot functionality with RAG retrieval and source citations
2. P2 - Personalized Chapter Content: Background-based content adaptation
3. P3 - Text Selection Summarization: In-context summarization tool
4. P4 - Urdu Chapter Translation: Accessibility feature for Urdu speakers
5. P1 - User Onboarding and Profile Management: Foundation for personalization
6. P2 - Conversation History and Continuity: Long-term learning support

**Functional Requirements**: 34 requirements across categories:
- Core RAG & Chatbot (6 requirements)
- User Authentication & Profiles (5 requirements)
- Personalization Features (4 requirements)
- Translation Features (3 requirements)
- Text Selection & Summarization (3 requirements)
- Conversation History (4 requirements)
- Backend API Requirements (3 requirements)
- Data Storage & Persistence (4 requirements)
- Agent Architecture (2 requirements)

**Success Criteria**: 12 measurable, technology-agnostic outcomes including response times (3s for chatbot, 30s for personalization), accuracy (98% anti-hallucination, 100% source citations), and scale (100 concurrent users).

**Key Entities**: 8 data models (User Profile, Conversation, Message, Book Content Chunk, Chapter, Retrieved Context, Personalization Request, Translation Request)

**Additional Sections**:
- 7 comprehensive edge cases
- 15 assumptions documented
- 15 out-of-scope items for clear boundaries
- 12 dependencies identified
- 10 risks with mitigation strategies

**Validation**: Created requirements.md checklist - all 14 quality criteria passed. Specification is complete and ready for `/sp.plan` phase.

## Outcome

- ✅ Impact: Created production-ready specification for RAG chatbot integration, defining 6 prioritized user stories, 34 functional requirements, and 12 measurable success criteria. Specification provides clear foundation for implementation planning.

- 🧪 Tests: No tests in specification phase. User stories include 15+ acceptance scenarios with Given-When-Then format that will guide test creation in implementation phase.

- 📁 Files: Created 2 files:
  - specs/003-rag-chatbot-integration/spec.md (comprehensive specification)
  - specs/003-rag-chatbot-integration/checklists/requirements.md (quality validation checklist)

- 🔁 Next prompts:
  - `/sp.clarify` - Ask targeted clarification questions if any requirements need refinement
  - `/sp.plan` - Generate architectural plan and implementation strategy
  - `/sp.tasks` - Break down into actionable development tasks with test cases

- 🧠 Reflection: Specification successfully balances technical requirements (RAG, agents, APIs) with user-focused outcomes. P1 stories (Interactive Q&A + User Onboarding) form a viable MVP that delivers core value. Strong emphasis on anti-hallucination (FR-004, SC-002) and source citation (FR-003, SC-012) ensures chatbot trustworthiness. Personalization and translation features (P2-P4) provide differentiation but are correctly deprioritized below core functionality.

## Evaluation notes (flywheel)

- Failure modes observed: None - specification passed all 14 checklist criteria without requiring revisions or clarifications.

- Graders run and results (PASS/FAIL):
  - Content Quality: PASS (user-focused, no implementation details)
  - Requirement Completeness: PASS (34 testable requirements, 0 clarifications needed)
  - Success Criteria: PASS (12 measurable, technology-agnostic outcomes)
  - Feature Readiness: PASS (ready for planning phase)

- Prompt variant (if applicable): Standard /sp.specify flow - no variants tested

- Next experiment (smallest change to try): Consider adding performance benchmarks for vector database queries in success criteria (e.g., "Qdrant similarity search returns top 5 chunks in < 500ms") to ensure SC-001 (3-second chatbot response) is achievable. Current spec assumes retrieval speed is acceptable but doesn't explicitly measure it.
