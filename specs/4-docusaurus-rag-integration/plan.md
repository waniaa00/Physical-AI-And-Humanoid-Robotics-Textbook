# Implementation Plan: Docusaurus RAG Frontend Integration

**Feature**: 4-docusaurus-rag-integration
**Branch**: `4-docusaurus-rag-integration`
**Created**: 2025-12-15
**Status**: Draft

## Executive Summary

This plan details the implementation of an integrated chat UI in the Docusaurus-based humanoid robotics book, connecting the frontend to the FastAPI RAG backend (Feature 3). The implementation will enable readers to ask questions within the book interface and receive grounded answers with source citations.

**Scope**: Frontend-only implementation connecting to existing backend
**Timeline**: 3-5 days focused development
**Risk Level**: Medium (new UI integration, CORS configuration, environment setup)

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Technical Decisions](#technical-decisions)
3. [Implementation Phases](#implementation-phases)
4. [File Structure](#file-structure)
5. [Component Hierarchy](#component-hierarchy)
6. [State Management](#state-management)
7. [API Integration](#api-integration)
8. [Testing Strategy](#testing-strategy)
9. [Deployment](#deployment)
10. [Risk Analysis](#risk-analysis)

---

## Architecture Overview

### System Context

```
┌─────────────────────────────────────────────────────────┐
│                    User's Browser                        │
│  ┌───────────────────────────────────────────────────┐  │
│  │         Docusaurus Book Frontend                   │  │
│  │  ┌─────────────┐  ┌──────────────────────────┐    │  │
│  │  │   Book      │  │    Chat UI Widget        │    │  │
│  │  │   Content   │  │  (React Components)      │    │  │
│  │  │   (MDX)     │  │  - ChatButton            │    │  │
│  │  │             │  │  - ChatWindow            │    │  │
│  │  │             │  │  - MessageList           │    │  │
│  │  │             │  │  - ChatInput             │    │  │
│  │  └─────────────┘  └──────────────────────────┘    │  │
│  │                            │                        │  │
│  │                            │ HTTP POST              │  │
│  │                            ▼                        │  │
│  │                   ┌─────────────────┐               │  │
│  │                   │   API Client    │               │  │
│  │                   │  (fetch wrapper)│               │  │
│  │                   └─────────────────┘               │  │
│  └────────────────────────────┼──────────────────────  │
│                                │                        │
└────────────────────────────────┼────────────────────────┘
                                 │
                                 │ /agent/chat
                                 │ (CORS enabled)
                                 ▼
┌─────────────────────────────────────────────────────────┐
│              FastAPI Backend (Feature 3)                 │
│  ┌────────────────────────────────────────────────┐    │
│  │  POST /agent/chat                              │    │
│  │  - Validate request                            │    │
│  │  - Call OpenAI Agents SDK                      │    │
│  │  - Retrieve from vector DB                     │    │
│  │  - Generate grounded response                  │    │
│  │  - Return with citations                       │    │
│  └────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────┘
```

### Component Architecture

```
Root (@theme/Root)
└── ChatProvider (Context)
    └── ChatWidget
        ├── ChatButton (floating button)
        └── ChatWindow (conditional render)
            ├── ChatHeader
            │   ├── Title ("Ask about the book")
            │   ├── ContextBadge (if context-constrained)
            │   └── CloseButton
            ├── MessageList
            │   └── ChatMessage[] (map)
            │       ├── UserMessage
            │       └── AgentMessage
            │           ├── MessageText (with citations)
            │           └── CitationList
            ├── SelectedContextBadge (if text selected)
            │   ├── ContextPreview
            │   └── ClearButton
            └── ChatInput
                ├── TextArea
                ├── SendButton
                └── ErrorMessage (if validation error)
```

---

## Technical Decisions

### Architecture Decision Records (ADRs)

#### AD-001: Use Docusaurus Root Theme Component for Global Chat Integration

**Status**: Accepted

**Context**:
Need to integrate chat UI across all book pages without modifying individual MDX files. Evaluated three approaches:
1. Root theme component (swizzling `@theme/Root`)
2. MDX import in every page
3. Docusaurus plugin

**Decision**: Use Root theme component (`@theme/Root`)

**Rationale**:
- ✅ Single integration point - wraps entire site automatically
- ✅ No changes to MDX content files
- ✅ Native Docusaurus pattern (recommended by docs)
- ✅ Minimal maintenance burden
- ❌ Requires swizzling (one-time setup)

**Alternatives Considered**:
- MDX import: Rejected - requires editing all pages, fragile
- Plugin: Rejected - overkill for simple UI injection

**Implementation**:
```bash
npm run swizzle @docusaurus/theme-classic Root -- --wrap
```

Creates `website/src/theme/Root.tsx` which wraps all pages.

---

#### AD-002: Use Shadcn/ui + Tailwind CSS for Component Library

**Status**: Accepted

**Context**:
Need UI components for chat interface (buttons, inputs, scrollable lists, tooltips). Evaluated:
1. Shadcn/ui (copy-paste components + Tailwind)
2. Vanilla React + CSS modules
3. Chakra UI
4. Material-UI

**Decision**: Shadcn/ui + Tailwind CSS

**Rationale**:
- ✅ Copy-paste components (no dependency bloat)
- ✅ Full customization control
- ✅ Accessible by default (built on Radix UI)
- ✅ Tailwind already common in React ecosystem
- ✅ Modern DX with utility classes
- ❌ Requires Tailwind CSS setup in Docusaurus

**Alternatives Considered**:
- Chakra UI: Rejected - adds 200KB+ to bundle
- MUI: Rejected - opinionated styling, larger bundle
- Vanilla: Rejected - reinventing accessibility

**Dependencies**:
```json
{
  "tailwindcss": "^3.4.0",
  "@tailwindcss/typography": "^0.5.10",
  "class-variance-authority": "^0.7.0",
  "clsx": "^2.1.0",
  "tailwind-merge": "^2.2.0"
}
```

**Components to Install**:
- Button
- Textarea
- Scroll Area
- Tooltip
- Badge
- Card

---

#### AD-003: Use React Context API for State Management

**Status**: Accepted

**Context**:
Need to manage chat state (messages, session, UI state) across multiple components. Evaluated:
1. React Context API + useState
2. Zustand
3. Redux Toolkit
4. Jotai

**Decision**: React Context API + useState

**Rationale**:
- ✅ No additional dependencies
- ✅ Built into React
- ✅ Sufficient for simple state (session, messages, UI flags)
- ✅ Easy to reason about
- ✅ Docusaurus already uses React
- ❌ Potential re-render issues (mitigated with useMemo)

**Alternatives Considered**:
- Zustand: Rejected - overkill for simple state
- Redux: Rejected - too heavy for this use case

**State Structure**:
```typescript
ChatContext = {
  session: Session,           // Current conversation
  uiState: UIState,           // UI flags (isOpen, isLoading, error)
  sendMessage: (msg) => {},   // Send user message
  selectText: (ctx) => {},    // Set selected context
  clearContext: () => {},     // Clear context mode
  resetConversation: () => {},// Start new session
  toggleChat: () => {}        // Open/close chat
}
```

---

#### AD-004: Use Native fetch API with Custom Wrapper

**Status**: Accepted

**Context**:
Need HTTP client to call `/agent/chat` endpoint. Evaluated:
1. Native fetch with custom wrapper
2. Axios
3. TanStack Query (React Query)

**Decision**: Native fetch with custom wrapper

**Rationale**:
- ✅ Zero dependencies (fetch built into browsers)
- ✅ Full control over timeout, retries, error handling
- ✅ Lighter bundle size
- ✅ Simple use case (one endpoint)
- ❌ Manual timeout implementation (using AbortController)

**Alternatives Considered**:
- Axios: Rejected - adds dependency for features we don't need
- React Query: Rejected - overkill (no caching/invalidation needed)

**Implementation**:
```typescript
// website/src/lib/apiClient.ts
async function sendMessage(request: ChatRequest): Promise<ChatResponse> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 30000);

  const response = await fetch(`${API_BASE_URL}/agent/chat`, {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(request),
    signal: controller.signal,
  });

  clearTimeout(timeoutId);
  return await response.json();
}
```

---

#### AD-005: Use window.getSelection() for Text Capture

**Status**: Accepted

**Context**:
Need to capture user-selected text from book content. Evaluated:
1. window.getSelection() (native browser API)
2. react-use-text-selection library
3. Manual mouse event tracking

**Decision**: window.getSelection() with custom hook

**Rationale**:
- ✅ Native browser API (zero dependencies)
- ✅ Works across all book content
- ✅ Supports keyboard selection (Shift+Arrow)
- ✅ Access to Range API for precise offsets
- ❌ Requires selectionchange event listener

**Alternatives Considered**:
- react-use-text-selection: Rejected - adds dependency for simple task
- Manual tracking: Rejected - complex, fragile

**Implementation**:
```typescript
// website/src/hooks/useTextSelection.ts
function useTextSelection() {
  const [selection, setSelection] = useState({text: '', range: null});

  useEffect(() => {
    function handleSelectionChange() {
      const sel = window.getSelection();
      if (sel && sel.toString().trim().length > 0) {
        setSelection({text: sel.toString().trim(), range: sel.getRangeAt(0)});
      }
    }

    document.addEventListener('selectionchange', handleSelectionChange);
    return () => document.removeEventListener('selectionchange', handleSelectionChange);
  }, []);

  return selection;
}
```

---

#### AD-006: Use customFields in docusaurus.config.js for Environment Variables

**Status**: Accepted

**Context**:
Need to configure backend URL differently for local dev and production. Evaluated:
1. customFields in docusaurus.config.js (build-time)
2. Runtime environment variables (window.ENV)
3. .env files with DefinePlugin

**Decision**: customFields in docusaurus.config.js with process.env

**Rationale**:
- ✅ Official Docusaurus pattern
- ✅ Build-time replacement (no runtime overhead)
- ✅ Type-safe access via useDocusaurusContext()
- ✅ Works with Netlify/Vercel environment variables
- ❌ Requires rebuild for changes (acceptable tradeoff)

**Alternatives Considered**:
- Runtime ENV: Rejected - requires separate config endpoint
- DefinePlugin only: Rejected - not idiomatic Docusaurus

**Implementation**:
```javascript
// website/docusaurus.config.js
module.exports = {
  customFields: {
    backendUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',
  },
};
```

```typescript
// website/src/lib/config.ts
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export function useBackendUrl(): string {
  const {siteConfig} = useDocusaurusContext();
  return siteConfig.customFields.backendUrl as string;
}
```

---

#### AD-007: Use Environment-Based Explicit CORS Origin List

**Status**: Accepted

**Context**:
Need to configure CORS to allow frontend requests. Evaluated:
1. Explicit origin list (environment-based)
2. Wildcard `*` for all origins
3. Dynamic origin validation

**Decision**: Environment-based explicit origin list

**Rationale**:
- ✅ Secure (no wildcard in production)
- ✅ Environment-specific (dev vs prod origins)
- ✅ Simple to configure
- ❌ Requires environment variable configuration

**Alternatives Considered**:
- Wildcard: Rejected - insecure, allows any origin
- Dynamic validation: Rejected - overkill for known origins

**Implementation**:
```python
# backend/main.py
from fastapi.middleware.cors import CORSMiddleware
import os

origins = os.getenv(
    'CORS_ORIGINS',
    'http://localhost:3000'  # Default for local dev
).split(',')

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=['POST', 'OPTIONS'],
    allow_headers=['Content-Type'],
)
```

**Environment Variables**:
```bash
# Local dev (.env)
CORS_ORIGINS=http://localhost:3000

# Production (Netlify/Railway env)
CORS_ORIGINS=https://humanoid-robotics.netlify.app
```

---

#### AD-008: Use UUID v4 with sessionStorage for Session Management

**Status**: Accepted

**Context**:
Need to generate and persist session IDs for conversation continuity. Evaluated:
1. UUID v4 + sessionStorage (per-tab)
2. UUID v4 + localStorage (cross-tab)
3. URL parameter (session_id in URL)
4. Server-generated session ID

**Decision**: UUID v4 via crypto.randomUUID() + sessionStorage

**Rationale**:
- ✅ Privacy-preserving (per-tab, cleared on tab close)
- ✅ Native browser API (crypto.randomUUID())
- ✅ No cross-tab pollution
- ✅ Prevents session hijacking (each tab independent)
- ❌ Doesn't persist across browser sessions (acceptable tradeoff)

**Alternatives Considered**:
- localStorage: Rejected - cross-tab sessions confusing
- URL param: Rejected - pollutes URL, hard to manage
- Server-generated: Rejected - requires roundtrip before first question

**Implementation**:
```typescript
// Generate session ID
function generateSessionId(): string {
  return crypto.randomUUID(); // Native browser API
}

// Persist in sessionStorage
function getOrCreateSessionId(): string {
  let sessionId = sessionStorage.getItem('chat_session_id');
  if (!sessionId) {
    sessionId = crypto.randomUUID();
    sessionStorage.setItem('chat_session_id', sessionId);
  }
  return sessionId;
}
```

---

#### AD-009: Use Regex Parsing for Citation Rendering

**Status**: Accepted

**Context**:
Need to parse agent responses to extract and render citations. Evaluated:
1. Regex pattern matching
2. Markdown parser library
3. Custom lexer/parser

**Decision**: Regex pattern matching with React components

**Rationale**:
- ✅ Simple and fast
- ✅ Agent response format is predictable
- ✅ No dependencies
- ✅ Easy to test
- ❌ Fragile if format changes (mitigated by backend contract)

**Alternatives Considered**:
- Markdown parser: Rejected - overkill, citations aren't standard markdown
- Custom parser: Rejected - unnecessary complexity

**Implementation**:
```typescript
function parseCitations(response: string) {
  // Match inline citations: [Source 1], [Source 2]
  const citationRegex = /\[Source (\d+)\]/g;
  const inlineCitations = [...response.matchAll(citationRegex)].map(m => ({
    number: parseInt(m[1]),
    placeholder: m[0],
  }));

  // Match source list: - [Source 1] <url>
  const sourcesRegex = /^- \[Source (\d+)\] (.+)$/gm;
  const sources = [...response.matchAll(sourcesRegex)].map(m => ({
    number: parseInt(m[1]),
    url: m[2].trim(),
  }));

  return {inlineCitations, sources};
}
```

---

## Implementation Phases

### Phase 0: Setup and Configuration ✅

**Goal**: Prepare development environment with required dependencies

**Tasks**:
1. Install Tailwind CSS in Docusaurus
2. Configure Tailwind to scan src/ directory
3. Install Shadcn/ui CLI and initialize
4. Set up custom theme directory structure
5. Configure environment variables (.env.example, .env.local)

**Acceptance Criteria**:
- ✅ Tailwind CSS compiles successfully
- ✅ Shadcn/ui CLI generates components correctly
- ✅ Environment variables accessible via customFields

**Duration**: 0.5 day

---

### Phase 1: Core Infrastructure

**Goal**: Implement foundational components and utilities

**Sub-Phase 1.1: API Client**

**Tasks**:
1. Create `website/src/lib/apiClient.ts`
   - Implement `sendMessage()` function with fetch
   - Add AbortController for timeout (30 seconds)
   - Implement error handling and APIError class
   - Add retry logic for network errors
2. Create `website/src/lib/config.ts`
   - Export `useBackendUrl()` hook
   - Read from `siteConfig.customFields.backendUrl`
3. Create `website/src/types/api.ts`
   - Define `ChatRequest`, `ChatResponse`, `APIError` types

**Acceptance Criteria**:
- ✅ `sendMessage()` successfully sends request to backend
- ✅ Timeout aborts request after 30 seconds
- ✅ Network errors throw `APIError` with type='network'
- ✅ 400 errors throw `APIError` with type='validation'
- ✅ 500 errors throw `APIError` with type='server'
- ✅ `useBackendUrl()` returns correct URL for environment

**Testing**:
```typescript
// Mock fetch and test error handling
describe('apiClient', () => {
  it('should timeout after 30 seconds', async () => {
    // Mock slow response
    global.fetch = jest.fn(() => new Promise(resolve => setTimeout(resolve, 40000)));

    await expect(sendMessage(validRequest)).rejects.toThrow(APIError);
  });

  it('should retry on network error', async () => {
    // Mock network failure then success
    global.fetch = jest.fn()
      .mockRejectedValueOnce(new TypeError('Network error'))
      .mockResolvedValueOnce({ok: true, json: async () => validResponse});

    const result = await sendMessage(validRequest);
    expect(result).toEqual(validResponse);
    expect(global.fetch).toHaveBeenCalledTimes(2);
  });
});
```

**Duration**: 0.5 day

---

**Sub-Phase 1.2: Session Management**

**Tasks**:
1. Create `website/src/lib/session.ts`
   - Implement `generateSessionId()` using crypto.randomUUID()
   - Implement `getOrCreateSessionId()` with sessionStorage
   - Implement `persistSession()`, `loadSession()`, `clearSession()`
   - Add session validation (UUID v4 regex)
2. Create `website/src/types/chat.ts`
   - Define all data model types (Session, ChatMessage, etc.)

**Acceptance Criteria**:
- ✅ `generateSessionId()` returns valid UUID v4
- ✅ `getOrCreateSessionId()` creates new ID on first call
- ✅ `getOrCreateSessionId()` returns same ID on subsequent calls
- ✅ Session persists across page navigations within same tab
- ✅ New tab gets new session ID
- ✅ Session clears on tab close

**Testing**:
```typescript
describe('session', () => {
  beforeEach(() => sessionStorage.clear());

  it('should generate valid UUID v4', () => {
    const id = generateSessionId();
    expect(UUID_V4_REGEX.test(id)).toBe(true);
  });

  it('should return same ID on subsequent calls', () => {
    const id1 = getOrCreateSessionId();
    const id2 = getOrCreateSessionId();
    expect(id1).toBe(id2);
  });

  it('should persist session to sessionStorage', () => {
    const session = createNewSession();
    persistSession(session);

    const loaded = loadSession();
    expect(loaded.sessionId).toBe(session.sessionId);
  });
});
```

**Duration**: 0.5 day

---

**Sub-Phase 1.3: Citation Parsing**

**Tasks**:
1. Create `website/src/lib/citations.ts`
   - Implement `parseCitations(response: string)`
   - Extract inline citations with regex
   - Extract source list with regex
   - Match citations by number
2. Create `website/src/lib/citations.test.ts`
   - Test various citation formats
   - Test edge cases (missing sources, unmatched numbers)

**Acceptance Criteria**:
- ✅ Correctly parses inline citations `[Source N]`
- ✅ Correctly parses source list `- [Source N] <url>`
- ✅ Matches citations by number
- ✅ Handles missing sources gracefully
- ✅ Handles multiple citations in same sentence

**Testing**:
```typescript
describe('parseCitations', () => {
  it('should parse inline citations and source list', () => {
    const response = `
      This is a fact [Source 1] and another [Source 2].

      Sources:
      - [Source 1] /docs/page1
      - [Source 2] /docs/page2
    `;

    const {inlineCitations, sources} = parseCitations(response);

    expect(inlineCitations).toHaveLength(2);
    expect(sources).toHaveLength(2);
    expect(sources[0].url).toBe('/docs/page1');
  });

  it('should handle missing source list', () => {
    const response = `This is a fact [Source 1].`;
    const {inlineCitations, sources} = parseCitations(response);

    expect(inlineCitations).toHaveLength(1);
    expect(sources).toHaveLength(0);
  });
});
```

**Duration**: 0.5 day

---

### Phase 2: Chat Context and State Management

**Goal**: Implement global chat state with React Context

**Tasks**:
1. Create `website/src/contexts/ChatContext.tsx`
   - Define `ChatContextValue` interface
   - Implement `ChatProvider` component
   - Implement state management:
     - `session: Session` (useState)
     - `uiState: UIState` (useState)
   - Implement actions:
     - `sendMessage(message: string): Promise<void>`
     - `selectText(context: SelectedTextContext): void`
     - `clearContext(): void`
     - `resetConversation(): void`
     - `toggleChat(): void`
2. Implement `sendMessage` logic:
   - Add optimistic user message to state
   - Add loading message for assistant
   - Call `apiClient.sendMessage()`
   - On success: Update assistant message with response
   - On error: Set error state, remove loading message
   - Parse citations from response
   - Update session in sessionStorage
3. Create `website/src/hooks/useChatContext.ts`
   - Export `useChatContext()` hook
   - Throw error if used outside provider

**Acceptance Criteria**:
- ✅ ChatProvider wraps app and provides context
- ✅ `sendMessage()` adds user message optimistically
- ✅ `sendMessage()` shows loading state while waiting
- ✅ `sendMessage()` updates with assistant response on success
- ✅ `sendMessage()` sets error state on failure
- ✅ `selectText()` switches to context-constrained mode
- ✅ `clearContext()` switches back to full-book mode
- ✅ `resetConversation()` generates new session ID
- ✅ `toggleChat()` opens/closes chat window

**Testing**:
```typescript
describe('ChatContext', () => {
  it('should send message and update state', async () => {
    const {result} = renderHook(() => useChatContext(), {
      wrapper: ChatProvider,
    });

    // Mock API response
    jest.spyOn(apiClient, 'sendMessage').mockResolvedValue({
      response: 'Test response [Source 1]\n\nSources:\n- [Source 1] /docs/test',
      session_id: result.current.session.sessionId,
      status: 'success',
    });

    await act(async () => {
      await result.current.sendMessage('Test question');
    });

    expect(result.current.session.messages).toHaveLength(2);
    expect(result.current.session.messages[0].role).toBe('user');
    expect(result.current.session.messages[1].role).toBe('assistant');
    expect(result.current.session.messages[1].citations).toHaveLength(1);
  });

  it('should handle context selection', () => {
    const {result} = renderHook(() => useChatContext(), {
      wrapper: ChatProvider,
    });

    const context = {
      text: 'Test selected text',
      sourceSection: 'Chapter 1',
      startOffset: 0,
      endOffset: 18,
      selectedAt: new Date(),
    };

    act(() => {
      result.current.selectText(context);
    });

    expect(result.current.session.mode).toBe('context-constrained');
    expect(result.current.session.selectedContext).toEqual(context);
    expect(result.current.uiState.showContextBadge).toBe(true);
  });
});
```

**Duration**: 1 day

---

### Phase 3: UI Components (Shadcn/ui)

**Goal**: Install and customize Shadcn/ui components

**Tasks**:
1. Install Shadcn/ui components:
   ```bash
   npx shadcn-ui@latest add button
   npx shadcn-ui@latest add textarea
   npx shadcn-ui@latest add scroll-area
   npx shadcn-ui@latest add tooltip
   npx shadcn-ui@latest add badge
   npx shadcn-ui@latest add card
   ```
2. Customize component variants as needed:
   - Button: Add `floating` variant for chat button
   - Badge: Add `context` variant for context indicator
3. Install lucide-react for icons:
   ```bash
   npm install lucide-react
   ```

**Acceptance Criteria**:
- ✅ All Shadcn/ui components installed in `website/src/components/ui/`
- ✅ Components styled with Tailwind
- ✅ Accessible (keyboard navigation, ARIA labels)
- ✅ Icons imported from lucide-react

**Duration**: 0.5 day

---

### Phase 4: Chat UI Components

**Goal**: Build chat interface components

**Sub-Phase 4.1: ChatButton (Floating Button)**

**Tasks**:
1. Create `website/src/components/ChatKit/ChatButton.tsx`
   - Floating button (fixed position bottom-right)
   - Icon: MessageSquare from lucide-react
   - Click handler: calls `toggleChat()`
   - Badge indicator if unread messages (future enhancement)

**Component Structure**:
```typescript
export function ChatButton() {
  const {toggleChat, uiState} = useChatContext();

  return (
    <Tooltip>
      <TooltipTrigger asChild>
        <Button
          variant="floating"
          onClick={toggleChat}
          className="fixed bottom-6 right-6 h-14 w-14 rounded-full shadow-lg"
          aria-label="Open chat"
        >
          <MessageSquare className="h-6 w-6" />
        </Button>
      </TooltipTrigger>
      <TooltipContent>Ask a question about the book</TooltipContent>
    </Tooltip>
  );
}
```

**Acceptance Criteria**:
- ✅ Button appears in bottom-right corner on all pages
- ✅ Clicking button opens chat window
- ✅ Tooltip shows on hover
- ✅ Accessible via keyboard (Tab to focus, Enter to activate)

**Duration**: 0.25 day

---

**Sub-Phase 4.2: ChatWindow (Container)**

**Tasks**:
1. Create `website/src/components/ChatKit/ChatWindow.tsx`
   - Conditional render based on `uiState.isOpen`
   - Fixed position (bottom-right, above ChatButton)
   - Dimensions: 400px wide, 600px tall (desktop)
   - Responsive: Full screen on mobile (<640px)
   - Contains: ChatHeader, MessageList, SelectedContextBadge, ChatInput

**Component Structure**:
```typescript
export function ChatWindow() {
  const {uiState} = useChatContext();

  if (!uiState.isOpen) return null;

  return (
    <Card className="fixed bottom-24 right-6 w-[400px] h-[600px] flex flex-col shadow-2xl sm:w-full sm:h-full sm:bottom-0 sm:right-0">
      <ChatHeader />
      <MessageList />
      {uiState.showContextBadge && <SelectedContextBadge />}
      <ChatInput />
    </Card>
  );
}
```

**Acceptance Criteria**:
- ✅ Window appears when `isOpen=true`
- ✅ Window hidden when `isOpen=false`
- ✅ Positioned correctly on desktop (bottom-right)
- ✅ Full screen on mobile
- ✅ Box shadow for depth

**Duration**: 0.25 day

---

**Sub-Phase 4.3: ChatHeader**

**Tasks**:
1. Create `website/src/components/ChatKit/ChatHeader.tsx`
   - Title: "Ask about the book"
   - Context badge (if context-constrained mode)
   - Close button (X icon)
   - New conversation button (RotateCcw icon)

**Component Structure**:
```typescript
export function ChatHeader() {
  const {toggleChat, resetConversation, session, uiState} = useChatContext();

  return (
    <div className="flex items-center justify-between p-4 border-b">
      <div className="flex items-center gap-2">
        <h2 className="text-lg font-semibold">Ask about the book</h2>
        {uiState.mode === 'context-constrained' && (
          <Badge variant="context">Based on selection</Badge>
        )}
      </div>
      <div className="flex gap-2">
        <Tooltip>
          <TooltipTrigger asChild>
            <Button variant="ghost" size="icon" onClick={resetConversation}>
              <RotateCcw className="h-4 w-4" />
            </Button>
          </TooltipTrigger>
          <TooltipContent>New conversation</TooltipContent>
        </Tooltip>
        <Button variant="ghost" size="icon" onClick={toggleChat}>
          <X className="h-4 w-4" />
        </Button>
      </div>
    </div>
  );
}
```

**Acceptance Criteria**:
- ✅ Title displays correctly
- ✅ Context badge appears only in context-constrained mode
- ✅ Close button closes chat window
- ✅ New conversation button resets session
- ✅ Icons render correctly

**Duration**: 0.25 day

---

**Sub-Phase 4.4: MessageList**

**Tasks**:
1. Create `website/src/components/ChatKit/MessageList.tsx`
   - Scrollable container (ScrollArea from Shadcn/ui)
   - Map over `session.messages`
   - Render UserMessage or AgentMessage based on role
   - Auto-scroll to bottom on new message
   - Loading indicator for assistant response

**Component Structure**:
```typescript
export function MessageList() {
  const {session, uiState} = useChatContext();
  const messagesEndRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({behavior: 'smooth'});
  }, [session.messages]);

  return (
    <ScrollArea className="flex-1 p-4">
      {session.messages.length === 0 && (
        <div className="text-center text-muted-foreground py-8">
          Ask a question to get started
        </div>
      )}
      {session.messages.map((message) => (
        <ChatMessage key={message.id} message={message} />
      ))}
      {uiState.isLoading && <LoadingMessage />}
      <div ref={messagesEndRef} />
    </ScrollArea>
  );
}
```

**Acceptance Criteria**:
- ✅ Messages display in chronological order
- ✅ Auto-scrolls to bottom on new message
- ✅ Empty state shows placeholder text
- ✅ Loading indicator appears while waiting for response
- ✅ Scrollable when messages exceed viewport

**Duration**: 0.5 day

---

**Sub-Phase 4.5: ChatMessage (User + Agent)**

**Tasks**:
1. Create `website/src/components/ChatKit/ChatMessage.tsx`
   - Conditional render based on `message.role`
   - UserMessage: Right-aligned, blue background
   - AgentMessage: Left-aligned, gray background, includes citations
2. Create `website/src/components/ChatKit/CitationList.tsx`
   - Render parsed citations as clickable links
   - Click handler navigates to source URL

**UserMessage Component**:
```typescript
function UserMessage({message}: {message: ChatMessage}) {
  return (
    <div className="flex justify-end mb-4">
      <div className="bg-primary text-primary-foreground rounded-lg p-3 max-w-[80%]">
        <p className="text-sm">{message.content}</p>
        <span className="text-xs opacity-70">
          {format(message.timestamp, 'HH:mm')}
        </span>
      </div>
    </div>
  );
}
```

**AgentMessage Component**:
```typescript
function AgentMessage({message}: {message: ChatMessage}) {
  const {inlineCitations, sources} = parseCitations(message.content);

  // Split response into main text and sources section
  const [mainText, sourcesText] = message.content.split('\n\nSources:\n');

  return (
    <div className="flex justify-start mb-4">
      <div className="bg-muted rounded-lg p-3 max-w-[80%]">
        <MessageText text={mainText} citations={inlineCitations} />
        {sources.length > 0 && <CitationList sources={sources} />}
        <span className="text-xs text-muted-foreground">
          {format(message.timestamp, 'HH:mm')}
        </span>
      </div>
    </div>
  );
}
```

**CitationList Component**:
```typescript
function CitationList({sources}: {sources: SourceCitation[]}) {
  return (
    <div className="mt-2 pt-2 border-t">
      <p className="text-xs font-semibold mb-1">Sources:</p>
      <ul className="text-xs space-y-1">
        {sources.map((source) => (
          <li key={source.number}>
            <a
              href={source.url}
              target="_blank"
              rel="noopener noreferrer"
              className="text-primary hover:underline flex items-center gap-1"
            >
              <ExternalLink className="h-3 w-3" />
              [Source {source.number}] {source.title || source.url}
            </a>
          </li>
        ))}
      </ul>
    </div>
  );
}
```

**Acceptance Criteria**:
- ✅ User messages right-aligned with blue background
- ✅ Agent messages left-aligned with gray background
- ✅ Citations parsed and rendered as clickable links
- ✅ Clicking citation opens URL in new tab
- ✅ Timestamps display correctly
- ✅ Error messages display in red (if message.error !== null)

**Duration**: 0.75 day

---

**Sub-Phase 4.6: ChatInput**

**Tasks**:
1. Create `website/src/components/ChatKit/ChatInput.tsx`
   - Textarea for user input
   - Send button (Send icon)
   - Enter to send, Shift+Enter for newline
   - Disable while loading
   - Show validation errors
   - Character counter (0/5000)

**Component Structure**:
```typescript
export function ChatInput() {
  const {sendMessage, uiState} = useChatContext();
  const [input, setInput] = useState('');
  const [error, setError] = useState<string | null>(null);

  const handleSubmit = async () => {
    const validation = validateMessage(input);
    if (!validation.valid) {
      setError(validation.error);
      return;
    }

    setError(null);
    await sendMessage(input);
    setInput(''); // Clear input on success
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit();
    }
  };

  return (
    <div className="p-4 border-t">
      {error && <p className="text-sm text-destructive mb-2">{error}</p>}
      <div className="flex gap-2">
        <Textarea
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder="Ask a question..."
          disabled={uiState.isLoading}
          className="flex-1 min-h-[60px] max-h-[120px]"
        />
        <Button
          onClick={handleSubmit}
          disabled={uiState.isLoading || input.trim().length === 0}
        >
          <Send className="h-4 w-4" />
        </Button>
      </div>
      <p className="text-xs text-muted-foreground mt-1">
        {input.length}/5000 characters
      </p>
    </div>
  );
}
```

**Acceptance Criteria**:
- ✅ Textarea accepts user input
- ✅ Enter key sends message
- ✅ Shift+Enter adds newline
- ✅ Send button disabled while loading
- ✅ Send button disabled if input empty
- ✅ Validation errors display above input
- ✅ Character counter updates on input
- ✅ Input clears after successful send

**Duration**: 0.5 day

---

**Sub-Phase 4.7: SelectedContextBadge**

**Tasks**:
1. Create `website/src/components/ChatKit/SelectedContextBadge.tsx`
   - Shows preview of selected text
   - "Clear" button to exit context mode
   - Appears above ChatInput when context selected

**Component Structure**:
```typescript
export function SelectedContextBadge() {
  const {session, clearContext} = useChatContext();

  if (!session.selectedContext) return null;

  const preview = session.selectedContext.text.slice(0, 100) + '...';

  return (
    <div className="px-4 py-2 bg-accent border-t border-b">
      <div className="flex items-start justify-between gap-2">
        <div className="flex-1">
          <p className="text-xs font-semibold mb-1">Asking about:</p>
          <p className="text-sm italic text-muted-foreground">{preview}</p>
        </div>
        <Button variant="ghost" size="sm" onClick={clearContext}>
          Clear
        </Button>
      </div>
    </div>
  );
}
```

**Acceptance Criteria**:
- ✅ Badge appears when context selected
- ✅ Shows preview of selected text (max 100 chars)
- ✅ Clear button removes context and switches to full-book mode
- ✅ Badge hidden when no context

**Duration**: 0.25 day

---

### Phase 5: Text Selection Integration

**Goal**: Enable users to select book text and ask questions about it

**Tasks**:
1. Create `website/src/hooks/useTextSelection.ts`
   - Listen to `selectionchange` event
   - Extract selected text using `window.getSelection()`
   - Extract Range for offset calculation
   - Detect source section from DOM (nearest heading)
2. Create `website/src/components/ChatKit/TextSelectionTooltip.tsx`
   - Floating tooltip appears above selection
   - "Ask about this" button
   - Click handler calls `selectText()` and opens chat
3. Integrate into ChatWidget:
   - Use `useTextSelection()` hook
   - Show tooltip when text selected
   - Hide tooltip when selection cleared

**useTextSelection Hook**:
```typescript
function useTextSelection() {
  const [selection, setSelection] = useState<SelectionState>({
    text: '',
    range: null,
  });

  useEffect(() => {
    function handleSelectionChange() {
      const sel = window.getSelection();
      if (sel && sel.toString().trim().length >= 10) {
        const range = sel.getRangeAt(0);
        const sourceSection = getSourceSection(range.startContainer);

        setSelection({
          text: sel.toString().trim(),
          range,
          sourceSection,
          startOffset: range.startOffset,
          endOffset: range.endOffset,
        });
      } else {
        setSelection({text: '', range: null});
      }
    }

    document.addEventListener('selectionchange', handleSelectionChange);
    return () => document.removeEventListener('selectionchange', handleSelectionChange);
  }, []);

  return selection;
}

function getSourceSection(node: Node): string {
  // Traverse up DOM to find nearest heading (h1, h2, h3)
  let current = node.parentElement;
  while (current) {
    const heading = current.querySelector('h1, h2, h3');
    if (heading) return heading.textContent || 'Unknown Section';
    current = current.parentElement;
  }
  return 'Unknown Section';
}
```

**TextSelectionTooltip Component**:
```typescript
export function TextSelectionTooltip({selection}: {selection: SelectionState}) {
  const {selectText, toggleChat} = useChatContext();

  if (!selection.text) return null;

  const handleAskAbout = () => {
    selectText({
      text: selection.text,
      sourceSection: selection.sourceSection,
      startOffset: selection.startOffset,
      endOffset: selection.endOffset,
      selectedAt: new Date(),
    });
    toggleChat(); // Open chat if closed
  };

  return (
    <div className="absolute z-50 p-2 bg-background border rounded-lg shadow-lg">
      <Button onClick={handleAskAbout} size="sm">
        <MessageSquare className="h-4 w-4 mr-1" />
        Ask about this
      </Button>
    </div>
  );
}
```

**Acceptance Criteria**:
- ✅ Selecting text (>=10 chars) shows tooltip
- ✅ Tooltip positioned near selection
- ✅ Clicking "Ask about this" opens chat
- ✅ Chat switches to context-constrained mode
- ✅ Selected text preview appears in chat
- ✅ Clearing selection hides tooltip
- ✅ Works across all book content pages

**Duration**: 1 day

---

### Phase 6: Root Component Integration

**Goal**: Integrate chat widget globally across all pages

**Tasks**:
1. Swizzle Docusaurus Root component:
   ```bash
   npm run swizzle @docusaurus/theme-classic Root -- --wrap
   ```
2. Edit `website/src/theme/Root.tsx`:
   - Import ChatProvider and ChatWidget
   - Wrap children with ChatProvider
   - Render ChatWidget after children
3. Verify chat appears on all pages

**Root Component**:
```typescript
// website/src/theme/Root.tsx
import React from 'react';
import {ChatProvider} from '@site/src/contexts/ChatContext';
import {ChatWidget} from '@site/src/components/ChatKit/ChatWidget';

export default function Root({children}) {
  return (
    <ChatProvider>
      {children}
      <ChatWidget />
    </ChatProvider>
  );
}
```

**ChatWidget Component**:
```typescript
// website/src/components/ChatKit/ChatWidget.tsx
import React from 'react';
import {ChatButton} from './ChatButton';
import {ChatWindow} from './ChatWindow';
import {TooltipProvider} from '@/components/ui/tooltip';

export function ChatWidget() {
  return (
    <TooltipProvider>
      <ChatButton />
      <ChatWindow />
    </TooltipProvider>
  );
}
```

**Acceptance Criteria**:
- ✅ Chat button appears on all pages
- ✅ Chat window opens/closes correctly
- ✅ State persists across page navigations
- ✅ No hydration errors (SSR compatibility)

**Duration**: 0.5 day

---

### Phase 7: Error Handling and Loading States

**Goal**: Improve UX with error handling and loading indicators

**Tasks**:
1. Implement error boundary for chat components
2. Add loading spinner to AgentMessage while streaming
3. Add error message display (network, timeout, validation, server)
4. Add retry button for retryable errors
5. Add "Taking longer than usual..." message after 10 seconds

**Error Display Component**:
```typescript
function ErrorMessage({error}: {error: UIError}) {
  const {sendMessage, session} = useChatContext();

  const handleRetry = async () => {
    const lastUserMessage = [...session.messages]
      .reverse()
      .find((m) => m.role === 'user');

    if (lastUserMessage) {
      await sendMessage(lastUserMessage.content);
    }
  };

  return (
    <div className="p-4 bg-destructive/10 border border-destructive rounded-lg">
      <p className="text-sm text-destructive mb-2">{error.message}</p>
      {error.retryable && (
        <Button onClick={handleRetry} variant="outline" size="sm">
          Retry
        </Button>
      )}
    </div>
  );
}
```

**LoadingMessage Component**:
```typescript
function LoadingMessage() {
  const [showSlowWarning, setShowSlowWarning] = useState(false);

  useEffect(() => {
    const timer = setTimeout(() => setShowSlowWarning(true), 10000);
    return () => clearTimeout(timer);
  }, []);

  return (
    <div className="flex justify-start mb-4">
      <div className="bg-muted rounded-lg p-3 max-w-[80%]">
        <div className="flex items-center gap-2">
          <Loader2 className="h-4 w-4 animate-spin" />
          <span className="text-sm">Thinking...</span>
        </div>
        {showSlowWarning && (
          <p className="text-xs text-muted-foreground mt-2">
            This is taking longer than usual...
          </p>
        )}
      </div>
    </div>
  );
}
```

**Acceptance Criteria**:
- ✅ Loading spinner appears while waiting for response
- ✅ Error messages display with correct type-specific text
- ✅ Retry button appears for retryable errors
- ✅ Retry button resends last user message
- ✅ Slow warning appears after 10 seconds
- ✅ Error boundary catches component errors

**Duration**: 0.5 day

---

### Phase 8: Backend CORS Configuration

**Goal**: Configure backend to allow frontend requests

**Tasks**:
1. Update `backend/main.py`:
   - Add CORSMiddleware
   - Read allowed origins from environment variable
   - Set `allow_credentials=True`
2. Create `backend/.env.example`:
   - Document CORS_ORIGINS variable
3. Update production environment variables (Railway/Render):
   - Set CORS_ORIGINS to production frontend URL

**Backend Code**:
```python
# backend/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import os

app = FastAPI()

# Parse comma-separated origins from environment
origins = os.getenv(
    'CORS_ORIGINS',
    'http://localhost:3000'  # Default for local dev
).split(',')

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=['POST', 'OPTIONS'],
    allow_headers=['Content-Type', 'Authorization'],
)
```

**Environment Variables**:
```bash
# backend/.env (local dev)
CORS_ORIGINS=http://localhost:3000

# Production (Railway env)
CORS_ORIGINS=https://humanoid-robotics.netlify.app
```

**Acceptance Criteria**:
- ✅ Local frontend (localhost:3000) can call backend (localhost:8000)
- ✅ Production frontend can call production backend
- ✅ CORS preflight requests succeed
- ✅ No CORS errors in browser console

**Duration**: 0.25 day

---

### Phase 9: Testing

**Goal**: Ensure all features work correctly

**Sub-Phase 9.1: Unit Tests**

**Tasks**:
1. Test API client (timeout, retries, error handling)
2. Test session management (generation, persistence)
3. Test citation parsing (various formats, edge cases)
4. Test validation utilities (message, context, session ID)

**Duration**: 0.5 day

---

**Sub-Phase 9.2: Integration Tests**

**Tasks**:
1. Test ChatContext (sendMessage flow end-to-end)
2. Test component rendering (ChatButton, ChatWindow, MessageList)
3. Test text selection (useTextSelection hook)

**Duration**: 0.5 day

---

**Sub-Phase 9.3: End-to-End Tests**

**Tasks**:
1. Test full conversation flow:
   - Open chat
   - Send question
   - Receive answer with citations
   - Click citation (verify navigation)
   - Ask follow-up question
   - Verify context maintained
2. Test context-constrained flow:
   - Select text
   - Click "Ask about this"
   - Verify chat opens
   - Verify context badge appears
   - Send question
   - Verify response based on context (no retrieval)
   - Clear context
   - Verify full-book mode restored
3. Test error handling:
   - Stop backend
   - Send question
   - Verify error message
   - Start backend
   - Click retry
   - Verify success
4. Test session management:
   - Send messages
   - Navigate to different page
   - Verify conversation persists
   - Open new tab
   - Verify new session created

**Duration**: 1 day

---

### Phase 10: Documentation and Deployment

**Goal**: Document setup and deploy to production

**Tasks**:
1. Update README with:
   - Chat UI features
   - Setup instructions
   - Environment variables
   - Local development guide
2. Create quickstart.md (user-facing)
3. Test production deployment:
   - Deploy frontend to Netlify
   - Configure CORS_ORIGINS on backend
   - Verify chat works in production
4. Create PHR for planning phase

**Acceptance Criteria**:
- ✅ README documents chat UI setup
- ✅ quickstart.md provides user guide
- ✅ Production deployment successful
- ✅ Chat works in production environment
- ✅ PHR created with full context

**Duration**: 0.5 day

---

## File Structure

After implementation, the project structure will be:

```
website/
├── src/
│   ├── components/
│   │   ├── ui/                      # Shadcn/ui components
│   │   │   ├── button.tsx
│   │   │   ├── textarea.tsx
│   │   │   ├── scroll-area.tsx
│   │   │   ├── tooltip.tsx
│   │   │   ├── badge.tsx
│   │   │   └── card.tsx
│   │   └── ChatKit/                 # Chat UI components
│   │       ├── ChatWidget.tsx       # Root chat component
│   │       ├── ChatButton.tsx       # Floating button
│   │       ├── ChatWindow.tsx       # Chat container
│   │       ├── ChatHeader.tsx       # Header with title/actions
│   │       ├── MessageList.tsx      # Scrollable message list
│   │       ├── ChatMessage.tsx      # User/Agent messages
│   │       ├── CitationList.tsx     # Citation rendering
│   │       ├── ChatInput.tsx        # Input field + send button
│   │       ├── SelectedContextBadge.tsx  # Context indicator
│   │       ├── TextSelectionTooltip.tsx  # "Ask about this" tooltip
│   │       ├── ErrorMessage.tsx     # Error display
│   │       └── LoadingMessage.tsx   # Loading indicator
│   ├── contexts/
│   │   └── ChatContext.tsx          # Global chat state
│   ├── hooks/
│   │   ├── useChatContext.ts        # Chat context hook
│   │   └── useTextSelection.ts      # Text selection hook
│   ├── lib/
│   │   ├── apiClient.ts             # HTTP client
│   │   ├── config.ts                # Environment config
│   │   ├── session.ts               # Session management
│   │   ├── citations.ts             # Citation parsing
│   │   └── utils.ts                 # Utility functions
│   ├── types/
│   │   ├── api.ts                   # API types (Request/Response)
│   │   └── chat.ts                  # Chat types (entities)
│   └── theme/
│       └── Root.tsx                 # Docusaurus root wrapper
├── tailwind.config.js               # Tailwind configuration
├── components.json                  # Shadcn/ui config
├── .env.example                     # Environment variables template
└── .env.local                       # Local environment variables (gitignored)

backend/
├── main.py                          # FastAPI app (updated with CORS)
├── .env.example                     # Backend env template
└── .env                             # Backend env (gitignored)
```

---

## Component Hierarchy

```
Root (@theme/Root)
│
└── ChatProvider (Context)
    │
    └── ChatWidget
        │
        ├── ChatButton (floating, fixed bottom-right)
        │   └── Tooltip ("Ask a question about the book")
        │
        └── ChatWindow (conditional: uiState.isOpen)
            │
            ├── ChatHeader
            │   ├── Title ("Ask about the book")
            │   ├── ContextBadge (if mode='context-constrained')
            │   ├── NewConversationButton (RotateCcw icon)
            │   └── CloseButton (X icon)
            │
            ├── MessageList (ScrollArea)
            │   ├── EmptyState (if messages.length === 0)
            │   ├── ChatMessage[] (map over session.messages)
            │   │   ├── UserMessage
            │   │   │   ├── Content (text)
            │   │   │   └── Timestamp
            │   │   └── AgentMessage
            │   │       ├── MessageText (with inline citations)
            │   │       ├── CitationList (clickable sources)
            │   │       ├── Timestamp
            │   │       └── ErrorIndicator (if message.error)
            │   ├── LoadingMessage (if uiState.isLoading)
            │   │   ├── Spinner
            │   │   ├── "Thinking..." text
            │   │   └── SlowWarning (if >10 seconds)
            │   └── AutoScrollAnchor (ref for scroll-to-bottom)
            │
            ├── SelectedContextBadge (if session.selectedContext)
            │   ├── Label ("Asking about:")
            │   ├── ContextPreview (truncated text)
            │   └── ClearButton
            │
            ├── ErrorMessage (if uiState.error)
            │   ├── ErrorText (type-specific message)
            │   └── RetryButton (if error.retryable)
            │
            └── ChatInput
                ├── ValidationError (if local validation fails)
                ├── Textarea (multi-line input)
                ├── SendButton (Send icon, disabled while loading)
                └── CharacterCounter (0/5000)
```

---

## State Management

### ChatContext State

```typescript
interface ChatContextState {
  // Session state
  session: Session;                    // Current conversation
  // {
  //   sessionId: string,
  //   messages: ChatMessage[],
  //   mode: 'full-book' | 'context-constrained',
  //   selectedContext: SelectedTextContext | null,
  //   createdAt: Date,
  //   lastActivityAt: Date
  // }

  // UI state
  uiState: UIState;                    // UI flags
  // {
  //   isOpen: boolean,
  //   isLoading: boolean,
  //   error: UIError | null,
  //   mode: 'full-book' | 'context-constrained',
  //   showContextBadge: boolean
  // }
}
```

### State Transitions

```
Initial State:
{
  session: { sessionId: <uuid>, messages: [], mode: 'full-book', selectedContext: null, ... },
  uiState: { isOpen: false, isLoading: false, error: null, mode: 'full-book', showContextBadge: false }
}

User clicks chat button:
  toggleChat() → uiState.isOpen = true

User submits question "What is IK?":
  sendMessage() →
    1. Add user message to session.messages (optimistic update)
    2. Set uiState.isLoading = true
    3. Call apiClient.sendMessage()
    4. On success:
       - Add assistant message to session.messages
       - Parse citations
       - Set uiState.isLoading = false
       - Persist session to sessionStorage
    5. On error:
       - Set uiState.error = {...}
       - Set uiState.isLoading = false

User selects text:
  selectText() →
    session.mode = 'context-constrained'
    session.selectedContext = {...}
    uiState.mode = 'context-constrained'
    uiState.showContextBadge = true

User clicks "Clear context":
  clearContext() →
    session.mode = 'full-book'
    session.selectedContext = null
    uiState.mode = 'full-book'
    uiState.showContextBadge = false

User clicks "New conversation":
  resetConversation() →
    session = createNewSession()  // New UUID, empty messages
    uiState.error = null
    uiState.mode = 'full-book'
    uiState.showContextBadge = false
    Persist new session to sessionStorage
```

---

## API Integration

### Request Flow

```
User submits question
  ↓
ChatInput calls sendMessage()
  ↓
ChatContext.sendMessage()
  1. Validate message
  2. Add user message to state (optimistic)
  3. Set isLoading = true
  4. Call apiClient.sendMessage()
  ↓
apiClient.sendMessage()
  1. Get backend URL from config
  2. Create AbortController for timeout
  3. fetch() POST to /agent/chat
  4. Handle timeout (abort after 30s)
  5. Check response.ok
  6. Parse JSON
  7. Return ChatResponse
  ↓
ChatContext receives response
  1. Parse citations from response.response
  2. Create assistant message with citations
  3. Add to session.messages
  4. Set isLoading = false
  5. Persist session to sessionStorage
  ↓
MessageList re-renders with new message
```

### Error Flow

```
apiClient.sendMessage() throws error
  ↓
ChatContext catch block
  1. Determine error type:
     - TypeError (network) → type='network'
     - AbortError → type='timeout'
     - 400 status → type='validation'
     - 500+ status → type='server'
  2. Create UIError object
  3. Set uiState.error = UIError
  4. Set isLoading = false
  ↓
ErrorMessage component renders
  1. Display user-friendly message
  2. Show Retry button if retryable
  ↓
User clicks Retry
  ↓
ErrorMessage calls sendMessage() with last user message
  (Flow starts over from beginning)
```

---

## Testing Strategy

### Unit Tests

**Utilities** (`lib/`):
- `apiClient.ts`:
  - ✅ Timeout after 30 seconds
  - ✅ Retry on network error (max 2 retries)
  - ✅ Throw APIError on 400/500
  - ✅ Parse response correctly
- `session.ts`:
  - ✅ Generate valid UUID v4
  - ✅ Return same session ID on repeated calls
  - ✅ Persist to sessionStorage
  - ✅ Load from sessionStorage on init
- `citations.ts`:
  - ✅ Parse inline citations
  - ✅ Parse source list
  - ✅ Handle missing sources
  - ✅ Handle multiple citations in sentence

**Hooks** (`hooks/`):
- `useTextSelection.ts`:
  - ✅ Extract text on selectionchange
  - ✅ Ignore selections <10 characters
  - ✅ Extract Range and offsets
  - ✅ Detect source section from DOM

### Integration Tests

**Context** (`contexts/ChatContext.tsx`):
- ✅ sendMessage adds user message optimistically
- ✅ sendMessage calls API and updates with response
- ✅ sendMessage handles errors correctly
- ✅ selectText switches to context-constrained mode
- ✅ clearContext switches back to full-book mode
- ✅ resetConversation generates new session ID
- ✅ toggleChat opens/closes window

**Components** (`components/ChatKit/`):
- ✅ ChatButton opens chat on click
- ✅ ChatWindow renders when open
- ✅ MessageList displays messages in order
- ✅ ChatInput validates and sends message
- ✅ CitationList renders clickable links

### End-to-End Tests (Playwright/Cypress)

**Full Conversation Flow**:
1. Open book page
2. Click chat button → verify chat opens
3. Type question "What is inverse kinematics?" → click Send
4. Wait for response → verify answer appears with citations
5. Click citation → verify navigation to source page
6. Return to original page → verify chat still open
7. Type follow-up "How does it work?" → click Send
8. Verify response understands context from first question

**Context-Constrained Flow**:
1. Open book page
2. Select text (drag to highlight paragraph)
3. Click "Ask about this" tooltip → verify chat opens
4. Verify context badge appears
5. Verify selected text preview shown
6. Type question "Explain this simply" → click Send
7. Verify response explains selected text (no broad retrieval)
8. Click "Clear" on context badge
9. Verify full-book mode restored

**Error Handling Flow**:
1. Open chat
2. Stop backend server (simulate network error)
3. Type question → click Send
4. Verify error message "Unable to connect to server"
5. Verify Retry button appears
6. Start backend
7. Click Retry → verify success

---

## Deployment

### Local Development

**Setup**:
```bash
# Frontend
cd website
npm install
npm install -D tailwindcss @tailwindcss/typography
npx shadcn-ui@latest init
npx shadcn-ui@latest add button textarea scroll-area tooltip badge card
npm run swizzle @docusaurus/theme-classic Root -- --wrap

# Create .env.local
echo "REACT_APP_BACKEND_URL=http://localhost:8000" > .env.local

# Start dev server
npm start  # Runs on http://localhost:3000

# Backend (separate terminal)
cd ../backend
pip install -r requirements.txt

# Create .env
echo "CORS_ORIGINS=http://localhost:3000" > .env

# Start FastAPI
uvicorn main:app --reload  # Runs on http://localhost:8000
```

**Verification**:
1. Open http://localhost:3000
2. Click chat button (bottom-right)
3. Type question → verify response with citations
4. Select text → click "Ask about this" → verify context mode

---

### Production Deployment

**Frontend (Netlify)**:

1. **Build Settings**:
   - Build command: `cd website && npm run build`
   - Publish directory: `website/build`

2. **Environment Variables** (Netlify dashboard):
   ```
   REACT_APP_BACKEND_URL=https://api.humanoid-robotics.com
   ```

3. **Deploy**:
   ```bash
   git push origin 4-docusaurus-rag-integration
   # Netlify auto-deploys on push
   ```

**Backend (Railway/Render)**:

1. **Environment Variables**:
   ```
   CORS_ORIGINS=https://humanoid-robotics.netlify.app
   ```

2. **Deploy**:
   ```bash
   # Railway auto-deploys from main branch
   git checkout main
   git merge 4-docusaurus-rag-integration
   git push origin main
   ```

**Verification**:
1. Open production frontend URL
2. Click chat button
3. Send question
4. Verify response (check browser console for CORS errors)

---

## Risk Analysis

### High Priority Risks

#### Risk 1: CORS Configuration Errors

**Probability**: Medium
**Impact**: High (blocks all frontend requests)

**Mitigation**:
- Test CORS locally before production
- Use environment-specific origin lists
- Verify preflight requests succeed
- Add CORS debug logging on backend

**Contingency**:
- If CORS fails in production, add temporary wildcard (`*`) to unblock
- Then debug and fix with proper origin list

---

#### Risk 2: Session Expiry Confusion

**Probability**: Medium
**Impact**: Medium (users lose context unexpectedly)

**Mitigation**:
- Display banner when backend creates new session (session ID changes)
- Consider extending backend session timeout from 30 minutes to 60 minutes
- Document session behavior in user guide

**Contingency**:
- If users complain about lost sessions, add localStorage persistence for conversation history (view-only)

---

#### Risk 3: Mobile UX Issues

**Probability**: High
**Impact**: Medium (chat hard to use on mobile)

**Mitigation**:
- Make chat window full-screen on mobile (<640px)
- Test on iOS Safari and Android Chrome
- Ensure keyboard doesn't cover input field (viewport height handling)
- Test text selection on touch devices

**Contingency**:
- If mobile UX poor, add "Mobile optimizations coming soon" banner
- Prioritize desktop experience for MVP

---

### Medium Priority Risks

#### Risk 4: Large Bundle Size (Tailwind + Shadcn/ui)

**Probability**: Low
**Impact**: Medium (slow page load)

**Mitigation**:
- Use Tailwind purge to remove unused styles
- Lazy load chat components (React.lazy)
- Monitor bundle size with webpack-bundle-analyzer

**Contingency**:
- If bundle >500KB, remove Shadcn/ui and use vanilla CSS

---

#### Risk 5: SSR/Hydration Errors (Docusaurus)

**Probability**: Low
**Impact**: Medium (chat doesn't render)

**Mitigation**:
- Use `useEffect` for browser-only code (window.getSelection, sessionStorage)
- Test build output (npm run build && npm run serve)
- Avoid accessing `window` during render

**Contingency**:
- If hydration errors persist, wrap ChatWidget in `<BrowserOnly>` component

---

## Open Questions

1. **Q**: Should we implement conversation history export (download as JSON/PDF)?
   **A**: Out of scope for MVP. Add to future enhancements.

2. **Q**: Should we add keyboard shortcuts (e.g., Cmd+K to open chat)?
   **A**: Nice-to-have for P2. MVP uses click interaction only.

3. **Q**: How to handle very long responses (>2000 characters)?
   **A**: ScrollArea handles vertical scroll automatically. No truncation needed.

4. **Q**: Should we show typing indicators when agent is "thinking"?
   **A**: Yes - LoadingMessage with spinner + "Thinking..." text.

5. **Q**: Should we persist conversation history across browser sessions?
   **A**: No (per spec - sessions ephemeral). Use sessionStorage only.

---

## Success Metrics

### User Story Acceptance

- ✅ **US1**: Users can ask questions and receive cited answers
- ✅ **US2**: Users can select text and get context-constrained explanations
- ✅ **US3**: Users can have multi-turn conversations with context
- ✅ **US4**: Developers can deploy to local/production without code changes
- ✅ **US5**: Users see loading/error states appropriately
- ✅ **US6**: Users can click citations to navigate to sources

### Technical Acceptance

- ✅ Response time <5 seconds (95th percentile)
- ✅ Works on Chrome, Firefox, Safari, Edge (latest versions)
- ✅ Works on iOS Safari, Android Chrome
- ✅ 100% of citations rendered as clickable links
- ✅ 5-turn conversations succeed without context loss
- ✅ Environment switching works (local → production)
- ✅ Errors display within 2 seconds
- ✅ No CORS errors in browser console
- ✅ Conversation persists across page navigations (same tab)
- ✅ New tabs get new sessions

---

## Timeline Estimate

| Phase | Duration | Dependencies |
|-------|----------|--------------|
| Phase 0: Setup | 0.5 day | None |
| Phase 1: Infrastructure | 1.5 days | Phase 0 |
| Phase 2: Context & State | 1 day | Phase 1 |
| Phase 3: Shadcn/ui Setup | 0.5 day | Phase 0 |
| Phase 4: UI Components | 2.5 days | Phase 2, Phase 3 |
| Phase 5: Text Selection | 1 day | Phase 2, Phase 4 |
| Phase 6: Root Integration | 0.5 day | Phase 4, Phase 5 |
| Phase 7: Error Handling | 0.5 day | Phase 4 |
| Phase 8: Backend CORS | 0.25 day | None (backend) |
| Phase 9: Testing | 2 days | Phase 6, Phase 7 |
| Phase 10: Documentation | 0.5 day | Phase 9 |

**Total**: ~10.75 days (~2 weeks)

**MVP** (US1 + US2 + US4): ~7 days (Phases 0-6, 8-9)

**With Accelerators** (parallel work, simplified testing): ~5 days

---

## Next Steps

1. **Immediate**: Create quickstart.md for user documentation
2. **After Plan Approval**: Run `/sp.tasks` to generate task breakdown
3. **Implementation**: Execute tasks sequentially by phase
4. **Testing**: Validate each user story independently
5. **Deployment**: Deploy to production and verify CORS configuration
6. **Documentation**: Create PHR for planning session

---

## Appendix: Architectural Decision Summary

| ID | Decision | Status | Rationale |
|----|----------|--------|-----------|
| AD-001 | Root Theme Component | ✅ Accepted | Single integration point, no MDX changes |
| AD-002 | Shadcn/ui + Tailwind CSS | ✅ Accepted | Lightweight, accessible, customizable |
| AD-003 | React Context API | ✅ Accepted | Built-in, sufficient for simple state |
| AD-004 | Native fetch with wrapper | ✅ Accepted | Zero dependencies, full control |
| AD-005 | window.getSelection() | ✅ Accepted | Native API, works everywhere |
| AD-006 | customFields for env vars | ✅ Accepted | Official Docusaurus pattern |
| AD-007 | Explicit CORS origins | ✅ Accepted | Secure, environment-specific |
| AD-008 | UUID v4 + sessionStorage | ✅ Accepted | Privacy-preserving, per-tab |
| AD-009 | Regex citation parsing | ✅ Accepted | Simple, fast, predictable format |
