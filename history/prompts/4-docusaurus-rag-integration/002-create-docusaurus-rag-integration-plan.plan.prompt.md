---
id: 002
title: Create Docusaurus RAG Integration Implementation Plan
stage: plan
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5
feature: 4-docusaurus-rag-integration
branch: 4-docusaurus-rag-integration
user: developer
command: /sp.plan
labels: ["planning", "frontend-backend-integration", "react", "docusaurus", "chat-ui", "rag"]
links:
  spec: ../../../specs/4-docusaurus-rag-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/4-docusaurus-rag-integration/research.md
  - specs/4-docusaurus-rag-integration/data-model.md
  - specs/4-docusaurus-rag-integration/contracts/frontend-backend-api.md
  - specs/4-docusaurus-rag-integration/plan.md
  - specs/4-docusaurus-rag-integration/quickstart.md
tests:
  - null
---

## Prompt

/sp.plan - Add chat UI component to Docusaurus frontend, Connect frontend to FastAPI agent endpoint, Send user queries and selected text to backend, Render agent responses with source references, Configure local and production environment variables

## Response snapshot

Created comprehensive implementation plan for Feature 4 (Docusaurus RAG Frontend Integration) with 9 architectural decisions and 10 implementation phases:

**Architecture Decisions (ADRs)**:
1. **AD-001**: Use Docusaurus Root Theme Component (`@theme/Root` swizzling) for global chat integration - single integration point, no MDX changes
2. **AD-002**: Use Shadcn/ui + Tailwind CSS for component library - copy-paste components, accessible by default, full customization
3. **AD-003**: Use React Context API for state management - built-in, sufficient for simple state (session, messages, UI flags)
4. **AD-004**: Use native fetch API with custom wrapper - zero dependencies, full control over timeout/retries
5. **AD-005**: Use window.getSelection() for text capture - native browser API, supports keyboard selection
6. **AD-006**: Use customFields in docusaurus.config.js for environment variables - official Docusaurus pattern, build-time replacement
7. **AD-007**: Use environment-based explicit CORS origin list - secure, no wildcard in production
8. **AD-008**: Use UUID v4 + sessionStorage for session management - privacy-preserving (per-tab), cleared on tab close
9. **AD-009**: Use regex parsing for citation rendering - simple, fast, predictable format

**Implementation Phases** (10.75 days total, ~5 days MVP):

**Phase 0: Setup** (0.5 day)
- Install Tailwind CSS, Shadcn/ui CLI
- Configure environment variables (.env.example, .env.local)

**Phase 1: Core Infrastructure** (1.5 days)
- Sub-Phase 1.1: API Client (apiClient.ts with fetch, timeout, retry logic)
- Sub-Phase 1.2: Session Management (generateSessionId, sessionStorage persistence)
- Sub-Phase 1.3: Citation Parsing (regex extraction of inline + source list)

**Phase 2: Chat Context** (1 day)
- ChatContext.tsx with ChatProvider
- State management: session (messages, mode, context), uiState (isOpen, isLoading, error)
- Actions: sendMessage, selectText, clearContext, resetConversation, toggleChat

**Phase 3: UI Components** (0.5 day)
- Install Shadcn/ui components (button, textarea, scroll-area, tooltip, badge, card)
- Install lucide-react for icons

**Phase 4: Chat UI Components** (2.5 days)
- ChatButton (floating button bottom-right)
- ChatWindow (container with conditional render)
- ChatHeader (title, context badge, new conversation, close)
- MessageList (scrollable, auto-scroll to bottom)
- ChatMessage (user/agent messages with citations)
- ChatInput (textarea, send button, validation, character counter)
- SelectedContextBadge (context preview + clear button)

**Phase 5: Text Selection Integration** (1 day)
- useTextSelection hook (selectionchange event, window.getSelection())
- TextSelectionTooltip ("Ask about this" floating button)
- Integration with ChatWidget

**Phase 6: Root Component Integration** (0.5 day)
- Swizzle @theme/Root component
- Wrap with ChatProvider
- Render ChatWidget globally

**Phase 7: Error Handling** (0.5 day)
- Error boundary for chat components
- Loading states (spinner, "Thinking...", "Taking longer than usual...")
- Error messages (network, timeout, validation, server) with retry button

**Phase 8: Backend CORS** (0.25 day)
- Update backend/main.py with CORSMiddleware
- Configure CORS_ORIGINS environment variable

**Phase 9: Testing** (2 days)
- Unit tests (apiClient, session, citations, validation)
- Integration tests (ChatContext, components)
- E2E tests (full conversation, context-constrained, error handling, session management)

**Phase 10: Documentation** (0.5 day)
- Update README
- Create quickstart.md (user guide)
- Test production deployment
- Create PHR

**File Structure After Implementation**:
```
website/
├── src/
│   ├── components/
│   │   ├── ui/                      # Shadcn/ui components
│   │   └── ChatKit/                 # Chat UI components (13 files)
│   ├── contexts/
│   │   └── ChatContext.tsx
│   ├── hooks/
│   │   ├── useChatContext.ts
│   │   └── useTextSelection.ts
│   ├── lib/
│   │   ├── apiClient.ts
│   │   ├── config.ts
│   │   ├── session.ts
│   │   ├── citations.ts
│   │   └── utils.ts
│   ├── types/
│   │   ├── api.ts
│   │   └── chat.ts
│   └── theme/
│       └── Root.tsx
└── tailwind.config.js

backend/
├── main.py                          # Updated with CORS
└── .env.example
```

**Design Artifacts Created**:
1. **research.md** (14,000+ words) - 8 research questions with technical decisions, dependency list, project structure, open questions
2. **data-model.md** (7,000+ words) - 6 core entities (ChatMessage, Session, SelectedTextContext, SourceCitation, UIState, UIError), DTOs (ChatRequest, ChatResponse), state management strategy, session persistence schema, validation utilities, complete TypeScript definitions
3. **contracts/frontend-backend-api.md** (6,000+ words) - API contract for /agent/chat endpoint, request/response schemas, examples (full-book, context-constrained, multi-turn, guardrail, errors), citation format, session lifecycle, timeout/retry, CORS security, testing compliance
4. **plan.md** (18,000+ words) - Architecture overview, 9 ADRs, 10 implementation phases, component hierarchy, state transitions, API integration flow, testing strategy, deployment guide, risk analysis, timeline estimate
5. **quickstart.md** (3,000+ words) - User-facing guide covering features, tips, troubleshooting, keyboard shortcuts, privacy, FAQ, browser compatibility

**Technology Stack**:
- Frontend: React, TypeScript, Docusaurus, Shadcn/ui, Tailwind CSS
- State: React Context API, useState
- HTTP: Native fetch API with AbortController
- Session: UUID v4 via crypto.randomUUID(), sessionStorage
- Text Selection: window.getSelection(), Range API
- Icons: lucide-react

**MVP Scope** (US1 + US2 + US4): ~7 days
- Phases 0-6 (setup through root integration)
- Phase 8 (backend CORS)
- Phase 9 (testing)

**Risk Mitigation**:
- CORS configuration errors: Test locally first, use environment-specific origins, add debug logging
- Session expiry confusion: Display banner when session changes, extend timeout to 60 minutes
- Mobile UX issues: Full-screen on mobile, test iOS/Android, handle keyboard viewport
- Large bundle size: Tailwind purge, lazy load components, monitor with webpack-bundle-analyzer
- SSR/hydration errors: Use useEffect for browser-only code, test build output, avoid window during render

## Outcome

- ✅ Impact: Complete implementation plan ready for task generation. All 9 architectural decisions documented with rationale and alternatives. 10 phases with detailed acceptance criteria and testing strategy. MVP clearly scoped (7 days, US1+US2+US4). Risk analysis identifies 5 risks with mitigations. Component hierarchy and state management fully defined.
- 🧪 Tests: Testing strategy spans 3 levels: unit (apiClient, session, citations), integration (ChatContext, components), E2E (full conversations, context modes, errors, sessions). Each phase includes acceptance criteria. Total 2 days allocated for testing (Phase 9).
- 📁 Files: Created research.md (8 RQs, 9 decisions), data-model.md (6 entities + DTOs), contracts/frontend-backend-api.md (API contract + examples), plan.md (9 ADRs + 10 phases), quickstart.md (user guide). All design artifacts total ~48,000 words.
- 🔁 Next prompts: `/sp.tasks` to generate task breakdown based on 10 implementation phases. Tasks should reference ADRs (AD-001 through AD-009) and phases for traceability. Each task should specify exact file paths (website/src/components/ChatKit/..., website/src/lib/..., etc.).
- 🧠 Reflection: Planning phase successfully translated specification (6 user stories, 15 FRs) into concrete technical plan. Key insight: Docusaurus Root theme component (AD-001) is critical enabler - single integration point avoids fragile MDX imports. Shadcn/ui (AD-002) chosen over full libraries to control bundle size. Session management (AD-008) balances privacy (per-tab) with UX (maintains context within session). Text selection (AD-005) uses native APIs to avoid dependencies. All decisions favor simplicity over features (fetch over axios, Context over Zustand, regex over parser). MVP focus on US1+US2+US4 delivers core value (question answering + context mode + deployment) before enhancing with US3 (multi-turn) and US5 (error handling). Risk analysis preemptively addresses CORS, mobile UX, and bundle size. User-facing quickstart.md completes the picture with comprehensive guide for end users.

## Evaluation notes (flywheel)

- Failure modes observed: None. All design artifacts created successfully. Research questions comprehensively answered. No blockers or ambiguities encountered. File structure mirrors plan exactly.
- Graders run and results (PASS/FAIL): Manual validation: ADR completeness (PASS - 9 decisions with status/context/rationale/alternatives), Phase definition (PASS - 10 phases with tasks/acceptance/duration), Component hierarchy (PASS - full tree from Root to leaf components), State management (PASS - context structure + transitions defined), API integration (PASS - request/response flows + error handling), Testing strategy (PASS - 3 levels with examples), File structure (PASS - matches conventions), Risk analysis (PASS - 5 risks with mitigations).
- Prompt variant (if applicable): Standard /sp.plan workflow with user-provided focus areas: "Add chat UI component, Connect to agent endpoint, Send queries and selected text, Render responses with citations, Configure environment variables". All areas mapped to specific phases and ADRs. Research phase (Phase 0 in workflow) produced comprehensive findings that informed all subsequent design decisions.
- Next experiment (smallest change to try): Consider adding WebSocket support as alternative to HTTP polling for real-time response streaming (currently deferred to future). Would require: AD-010 (WebSocket vs HTTP decision), Phase 11 (WebSocket integration), updated contracts (WebSocket events), state management for streaming chunks. Trade-off: improved UX for long responses vs increased complexity (connection management, reconnection logic, fallback to HTTP). Current HTTP approach simpler and sufficient for MVP, but WebSocket could differentiate UX for follow-up questions.
