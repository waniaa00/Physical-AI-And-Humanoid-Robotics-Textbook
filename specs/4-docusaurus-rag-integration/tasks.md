# Task Breakdown: Docusaurus RAG Frontend Integration

**Feature**: 4-docusaurus-rag-integration
**Branch**: `4-docusaurus-rag-integration`
**Created**: 2025-12-16
**Status**: Ready for Implementation

## Overview

This document breaks down Feature 4 (Docusaurus RAG Frontend Integration) into 87 actionable tasks across 11 phases. Tasks reference user stories (US1-US6), architectural decisions (AD-001 through AD-009), and include parallelization markers ([P]) where applicable.

**Total Tasks**: 87
**MVP Tasks**: 62 (Phases 0-6, 8-9)
**Estimated Duration**: 10.75 days (MVP: ~7 days)

---

## Task Organization by Phase

- **Phase 0**: Setup and Configuration (6 tasks)
- **Phase 1**: Core Infrastructure (15 tasks)
- **Phase 2**: Chat Context and State Management (8 tasks)
- **Phase 3**: Shadcn/ui Components Installation (7 tasks)
- **Phase 4**: Chat UI Components (21 tasks)
- **Phase 5**: Text Selection Integration (6 tasks)
- **Phase 6**: Root Component Integration (4 tasks)
- **Phase 7**: Error Handling and Loading States (6 tasks)
- **Phase 8**: Backend CORS Configuration (3 tasks)
- **Phase 9**: Testing (9 tasks)
- **Phase 10**: Documentation and Deployment (2 tasks)

---

## User Story Mapping

- **US1 (P1)**: Basic Question Answering - Phases 0-4, 6-9
- **US2 (P1)**: Selected Text Context - Phase 5, portions of Phase 4
- **US3 (P2)**: Multi-Turn Conversations - Covered by Phase 2 (ChatContext)
- **US4 (P1)**: Environment Configuration - Phase 0, Phase 6, Phase 8
- **US5 (P2)**: Error Handling and Loading States - Phase 7
- **US6 (P3)**: Source Citation Navigation - Phase 4 (CitationList component)

---

## Phase 0: Setup and Configuration

**Goal**: Prepare development environment with required dependencies
**Duration**: 0.5 day
**Dependencies**: None

### Tasks

- [x] **T001** [US4] Install Tailwind CSS in Docusaurus project
  - **File**: `website/package.json`, `website/tailwind.config.js`
  - **Command**: `cd website && npm install -D tailwindcss postcss autoprefixer @tailwindcss/typography`
  - **Then**: `npx tailwindcss init -p`
  - **Acceptance**: Tailwind config file created, postcss.config.js exists
  - **Reference**: AD-002 (Shadcn/ui + Tailwind CSS)

- [x] **T002** [US4] Configure Tailwind to scan src/ directory
  - **File**: `website/tailwind.config.js`
  - **Edit**: Set `content: ['./src/**/*.{js,jsx,ts,tsx}', './docs/**/*.{md,mdx}']`
  - **Add**: Extend theme with custom colors if needed
  - **Acceptance**: Tailwind scans all source files

- [x] **T003** [US4] Initialize Shadcn/ui CLI and configure components.json
  - **File**: `website/components.json`
  - **Command**: `cd website && npx shadcn-ui@latest init`
  - **Interactive prompts**:
    - TypeScript: Yes
    - Style: Default
    - Base color: Slate
    - CSS variables: Yes
    - Import alias: @/components
  - **Acceptance**: components.json created with correct config
  - **Reference**: AD-002 (Shadcn/ui)

- [x] **T004** [P] [US4] Create .env.example file for frontend
  - **File**: `website/.env.example`
  - **Content**:
    ```
    # Backend API URL
    REACT_APP_BACKEND_URL=http://localhost:8000
    ```
  - **Acceptance**: Template file committed to repo

- [x] **T005** [P] [US4] Create .env.local file for local development (gitignored)
  - **File**: `website/.env.local`
  - **Content**:
    ```
    REACT_APP_BACKEND_URL=http://localhost:8000
    ```
  - **Add to**: `website/.gitignore` - add line `.env.local`
  - **Acceptance**: Local env file created, gitignored

- [x] **T006** [US4] Configure customFields in docusaurus.config.js
  - **File**: `website/docusaurus.config.ts` or `website/docusaurus.config.js`
  - **Edit**: Add to module.exports:
    ```javascript
    customFields: {
      backendUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',
    },
    ```
  - **Acceptance**: customFields accessible via useDocusaurusContext()
  - **Reference**: AD-006 (customFields for env vars)

**Phase 0 Acceptance Criteria**:
- ✅ Tailwind CSS compiles without errors
- ✅ Shadcn/ui CLI can generate components
- ✅ Environment variables load correctly
- ✅ `npm start` runs without errors

---

## Phase 1: Core Infrastructure

**Goal**: Implement foundational utilities and services
**Duration**: 1.5 days
**Dependencies**: Phase 0

### Sub-Phase 1.1: Type Definitions (Foundational)

- [ ] **T007** [P] Create api.ts with DTOs
  - **File**: `website/src/types/api.ts`
  - **Content**: Define `ChatRequest`, `ChatResponse`, `APIError` interfaces
  - **Acceptance**: All API types defined, exported
  - **Reference**: data-model.md (DTOs)

- [ ] **T008** [P] Create chat.ts with entity types
  - **File**: `website/src/types/chat.ts`
  - **Content**: Define `ChatMessage`, `Session`, `SelectedTextContext`, `SourceCitation`, `UIState`, `UIError`, `ChatContextValue`
  - **Acceptance**: All entity types defined, exported
  - **Reference**: data-model.md (Core Entities)

### Sub-Phase 1.2: Configuration

- [ ] **T009** Create config.ts with useBackendUrl hook
  - **File**: `website/src/lib/config.ts`
  - **Content**:
    ```typescript
    import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

    export function useBackendUrl(): string {
      const {siteConfig} = useDocusaurusContext();
      return siteConfig.customFields.backendUrl as string;
    }
    ```
  - **Acceptance**: Hook returns correct URL for environment
  - **Reference**: AD-006 (customFields)

### Sub-Phase 1.3: API Client

- [ ] **T010** [US1] Create apiClient.ts with sendMessage function
  - **File**: `website/src/lib/apiClient.ts`
  - **Content**:
    - Import types from `@/types/api`
    - Define `API_BASE_URL` (read from env or default)
    - Define `REQUEST_TIMEOUT = 30000`
    - Implement `APIError` class extending Error
    - Implement `sendMessage(request: ChatRequest): Promise<ChatResponse>`
  - **Logic**:
    - Create AbortController for timeout
    - fetch() POST to `${API_BASE_URL}/agent/chat`
    - Set signal for abort
    - setTimeout to abort after 30 seconds
    - Check response.ok
    - Parse JSON
    - Handle errors (network, timeout, 400, 500)
  - **Acceptance**: Function sends POST request, handles timeout
  - **Reference**: AD-004 (native fetch), contracts/frontend-backend-api.md

- [ ] **T011** [US1] Add timeout handling to apiClient
  - **File**: `website/src/lib/apiClient.ts`
  - **Edit**: Wrap fetch in try-catch, catch AbortError
  - **Throw**: `APIError(408, 'TIMEOUT', 'Request timed out')`
  - **Acceptance**: Request aborts after 30 seconds, throws timeout error

- [ ] **T012** [US1] Add error handling for different HTTP status codes
  - **File**: `website/src/lib/apiClient.ts`
  - **Edit**: Check response.status
    - 400 → `APIError(400, 'VALIDATION_ERROR', 'Invalid request')`
    - 500+ → `APIError(500, 'SERVER_ERROR', 'Server error occurred')`
    - Other → `APIError(status, 'UNKNOWN_ERROR', 'Request failed')`
  - **Acceptance**: Different error types thrown for different statuses

- [ ] **T013** [P] [US1] Add retry logic for network errors
  - **File**: `website/src/lib/apiClient.ts`
  - **Create**: `sendMessageWithRetry(request, maxRetries=2)` wrapper
  - **Logic**: Loop up to maxRetries, catch errors, wait with exponential backoff (1s, 2s), don't retry on 400
  - **Acceptance**: Retries on network/timeout, not on validation error

### Sub-Phase 1.4: Session Management

- [ ] **T014** Create session.ts with UUID generation
  - **File**: `website/src/lib/session.ts`
  - **Content**:
    - `generateSessionId(): string` using `crypto.randomUUID()`
    - `isValidSessionId(id: string): boolean` using UUID v4 regex
  - **Acceptance**: Generates valid UUID v4 format
  - **Reference**: AD-008 (UUID v4 + sessionStorage)

- [ ] **T015** [US3] Add sessionStorage persistence to session.ts
  - **File**: `website/src/lib/session.ts`
  - **Add**:
    - `getOrCreateSessionId(): string` - check sessionStorage, create if missing
    - `persistSession(session: Session): void` - save to sessionStorage
    - `loadSession(): Session | null` - load from sessionStorage, rehydrate Dates
    - `clearSession(): void` - remove from sessionStorage
  - **Keys**: `'chat_session_id'`, `'chat_session'`
  - **Acceptance**: Session persists across page navigations within tab

- [ ] **T016** Create createNewSession() helper
  - **File**: `website/src/lib/session.ts`
  - **Add**:
    ```typescript
    function createNewSession(): Session {
      return {
        sessionId: crypto.randomUUID(),
        messages: [],
        mode: 'full-book',
        selectedContext: null,
        createdAt: new Date(),
        lastActivityAt: new Date(),
      };
    }
    ```
  - **Export**: `createNewSession`
  - **Acceptance**: Returns valid new session object

### Sub-Phase 1.5: Citation Parsing

- [ ] **T017** [US1] [US6] Create citations.ts with parseCitations function
  - **File**: `website/src/lib/citations.ts`
  - **Content**:
    ```typescript
    interface ParsedCitations {
      inlineCitations: Array<{number: number; placeholder: string}>;
      sources: Array<{number: number; url: string}>;
    }

    export function parseCitations(response: string): ParsedCitations {
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
  - **Acceptance**: Correctly parses inline and source list citations
  - **Reference**: AD-009 (regex parsing), contracts/frontend-backend-api.md (citation format)

### Sub-Phase 1.6: Validation Utilities

- [ ] **T018** [P] Create utils.ts with validation functions
  - **File**: `website/src/lib/utils.ts`
  - **Content**:
    - `validateMessage(message: string): {valid: boolean; error?: string}`
    - `validateSelectedContext(context: SelectedTextContext): {valid: boolean; error?: string}`
    - `cn(...classes)` for className merging (from Shadcn/ui setup)
  - **Logic**:
    - Message: non-empty after trim, 1-5000 chars
    - Context: text 10-10,000 chars, endOffset > startOffset
  - **Acceptance**: Validation functions return correct results

**Phase 1 Acceptance Criteria**:
- ✅ API client sends POST requests to backend
- ✅ Timeout aborts requests after 30 seconds
- ✅ Session ID generates and persists in sessionStorage
- ✅ Citation parsing extracts inline and source list
- ✅ Validation functions prevent invalid inputs

---

## Phase 2: Chat Context and State Management

**Goal**: Implement global chat state with React Context
**Duration**: 1 day
**Dependencies**: Phase 1

### Tasks

- [ ] **T019** [US1] [US3] Create ChatContext.tsx with provider skeleton
  - **File**: `website/src/contexts/ChatContext.tsx`
  - **Content**:
    - Import types from `@/types/chat`
    - Define `ChatContext = createContext<ChatContextValue | undefined>(undefined)`
    - Create `ChatProvider` component with children prop
    - Create `useChatContext()` hook that throws if used outside provider
  - **Acceptance**: Context and provider structure in place

- [ ] **T020** [US1] [US3] Add session state to ChatProvider
  - **File**: `website/src/contexts/ChatContext.tsx`
  - **Add**:
    - `const [session, setSession] = useState<Session>(() => initializeSession())`
    - Implement `initializeSession()` using `loadSession() || createNewSession()`
  - **Acceptance**: Session state initialized from sessionStorage or fresh

- [ ] **T021** [US1] [US5] Add UI state to ChatProvider
  - **File**: `website/src/contexts/ChatContext.tsx`
  - **Add**:
    - `const [uiState, setUIState] = useState<UIState>({ isOpen: false, isLoading: false, error: null, mode: 'full-book', showContextBadge: false })`
  - **Acceptance**: UI state initialized correctly

- [ ] **T022** [US1] Implement toggleChat action
  - **File**: `website/src/contexts/ChatContext.tsx`
  - **Add**:
    ```typescript
    const toggleChat = useCallback(() => {
      setUIState(prev => ({...prev, isOpen: !prev.isOpen}));
    }, []);
    ```
  - **Acceptance**: Toggles isOpen flag

- [ ] **T023** [US3] Implement resetConversation action
  - **File**: `website/src/contexts/ChatContext.tsx`
  - **Add**:
    ```typescript
    const resetConversation = useCallback(() => {
      const newSession = createNewSession();
      setSession(newSession);
      persistSession(newSession);
      setUIState(prev => ({
        ...prev,
        error: null,
        mode: 'full-book',
        showContextBadge: false,
      }));
    }, []);
    ```
  - **Acceptance**: Creates new session, clears UI state

- [ ] **T024** [US2] Implement selectText and clearContext actions
  - **File**: `website/src/contexts/ChatContext.tsx`
  - **Add**:
    - `selectText(context: SelectedTextContext)` - sets session.mode='context-constrained', session.selectedContext=context, uiState updates
    - `clearContext()` - sets session.mode='full-book', session.selectedContext=null, uiState updates
  - **Acceptance**: Mode switches between full-book and context-constrained

- [ ] **T025** [US1] [US3] Implement sendMessage action (core logic)
  - **File**: `website/src/contexts/ChatContext.tsx`
  - **Add**: `sendMessage(message: string): Promise<void>`
  - **Logic**:
    1. Validate message
    2. Create user ChatMessage (optimistic update)
    3. Add to session.messages
    4. Set uiState.isLoading = true
    5. Build ChatRequest with session.sessionId, context_text from session.selectedContext
    6. Call apiClient.sendMessage()
    7. On success:
       - Parse citations from response.response
       - Create assistant ChatMessage with citations
       - Add to session.messages
       - Update session.lastActivityAt
       - persistSession()
       - Set uiState.isLoading = false
    8. On error:
       - Set uiState.error with appropriate UIError
       - Set uiState.isLoading = false
  - **Acceptance**: Sends message, updates state, handles errors
  - **Reference**: plan.md (State Transitions)

- [ ] **T026** [US1] Wire all actions and state to ChatContext.Provider value
  - **File**: `website/src/contexts/ChatContext.tsx`
  - **Edit**: Return `<ChatContext.Provider value={{session, uiState, sendMessage, selectText, clearContext, resetConversation, toggleChat}}>`
  - **Acceptance**: All state and actions accessible via useChatContext()

**Phase 2 Acceptance Criteria**:
- ✅ ChatProvider provides session and uiState
- ✅ sendMessage sends request and updates state
- ✅ selectText/clearContext switch modes correctly
- ✅ resetConversation generates new session ID
- ✅ toggleChat opens/closes UI

---

## Phase 3: Shadcn/ui Components Installation

**Goal**: Install and customize UI component primitives
**Duration**: 0.5 day
**Dependencies**: Phase 0

### Tasks

- [ ] **T027** [P] Install button component
  - **Command**: `cd website && npx shadcn-ui@latest add button`
  - **File**: `website/src/components/ui/button.tsx`
  - **Acceptance**: Button component generated

- [ ] **T028** [P] Install textarea component
  - **Command**: `cd website && npx shadcn-ui@latest add textarea`
  - **File**: `website/src/components/ui/textarea.tsx`
  - **Acceptance**: Textarea component generated

- [ ] **T029** [P] Install scroll-area component
  - **Command**: `cd website && npx shadcn-ui@latest add scroll-area`
  - **File**: `website/src/components/ui/scroll-area.tsx`
  - **Acceptance**: ScrollArea component generated

- [ ] **T030** [P] Install tooltip component
  - **Command**: `cd website && npx shadcn-ui@latest add tooltip`
  - **File**: `website/src/components/ui/tooltip.tsx`
  - **Acceptance**: Tooltip component generated

- [ ] **T031** [P] Install badge component
  - **Command**: `cd website && npx shadcn-ui@latest add badge`
  - **File**: `website/src/components/ui/badge.tsx`
  - **Acceptance**: Badge component generated

- [ ] **T032** [P] Install card component
  - **Command**: `cd website && npx shadcn-ui@latest add card`
  - **File**: `website/src/components/ui/card.tsx`
  - **Acceptance**: Card component generated

- [ ] **T033** Install lucide-react for icons
  - **Command**: `cd website && npm install lucide-react`
  - **Acceptance**: lucide-react in package.json, icons importable

**Phase 3 Acceptance Criteria**:
- ✅ All Shadcn/ui components in `website/src/components/ui/`
- ✅ lucide-react installed
- ✅ Components render without errors

---

## Phase 4: Chat UI Components

**Goal**: Build all chat interface React components
**Duration**: 2.5 days
**Dependencies**: Phase 2, Phase 3

### Sub-Phase 4.1: ChatButton (Floating Button)

- [ ] **T034** [US1] Create ChatButton.tsx component
  - **File**: `website/src/components/ChatKit/ChatButton.tsx`
  - **Content**:
    - Import Button, Tooltip from ui components
    - Import MessageSquare icon from lucide-react
    - Import useChatContext
    - Render floating button (fixed bottom-6 right-6, h-14 w-14, rounded-full, shadow-lg)
    - onClick calls toggleChat()
    - Wrap in Tooltip with "Ask a question about the book"
  - **Acceptance**: Button appears bottom-right, opens chat
  - **Reference**: plan.md (Sub-Phase 4.1)

### Sub-Phase 4.2: ChatWindow (Container)

- [ ] **T035** [US1] Create ChatWindow.tsx skeleton
  - **File**: `website/src/components/ChatKit/ChatWindow.tsx`
  - **Content**:
    - Import Card from ui components
    - Import useChatContext
    - Conditional render: `if (!uiState.isOpen) return null`
    - Return Card with fixed position (bottom-24 right-6, w-[400px] h-[600px], flex flex-col, shadow-2xl)
    - Add responsive: `sm:w-full sm:h-full sm:bottom-0 sm:right-0` (full screen on mobile)
    - Placeholder children: "ChatHeader", "MessageList", "ChatInput"
  - **Acceptance**: Window appears when isOpen=true, positioned correctly

### Sub-Phase 4.3: ChatHeader

- [ ] **T036** [US1] [US2] Create ChatHeader.tsx component
  - **File**: `website/src/components/ChatKit/ChatHeader.tsx`
  - **Content**:
    - Import Button, Badge from ui components
    - Import X, RotateCcw icons from lucide-react
    - Import useChatContext
    - Render header div (flex items-center justify-between p-4 border-b)
    - Left: Title "Ask about the book"
    - If uiState.mode === 'context-constrained': Badge with "Based on selection"
    - Right: New conversation button (RotateCcw icon, calls resetConversation), Close button (X icon, calls toggleChat)
  - **Acceptance**: Header displays title, mode badge, action buttons

### Sub-Phase 4.4: MessageList

- [ ] **T037** [US1] [US3] Create MessageList.tsx skeleton
  - **File**: `website/src/components/ChatKit/MessageList.tsx`
  - **Content**:
    - Import ScrollArea from ui components
    - Import useChatContext
    - Import useRef, useEffect for auto-scroll
    - Render ScrollArea (flex-1 p-4)
    - If session.messages.length === 0: Empty state "Ask a question to get started"
    - Map over session.messages, render ChatMessage component
    - If uiState.isLoading: Render LoadingMessage component
    - Auto-scroll to bottom: useRef + scrollIntoView on messages change
  - **Acceptance**: Messages display chronologically, auto-scroll works

### Sub-Phase 4.5: ChatMessage (User + Agent)

- [ ] **T038** [US1] Create ChatMessage.tsx with role-based rendering
  - **File**: `website/src/components/ChatKit/ChatMessage.tsx`
  - **Content**:
    - Import types ChatMessage
    - Prop: `message: ChatMessage`
    - Conditional render:
      - If message.role === 'user': Render UserMessage
      - If message.role === 'assistant': Render AgentMessage
  - **Acceptance**: Routes to correct message component

- [ ] **T039** [US1] Implement UserMessage component
  - **File**: `website/src/components/ChatKit/ChatMessage.tsx`
  - **Content**:
    - Render div (flex justify-end mb-4)
    - Inner div (bg-primary text-primary-foreground rounded-lg p-3 max-w-[80%])
    - Display message.content
    - Display timestamp (format with date-fns or Intl.DateTimeFormat)
  - **Acceptance**: User messages right-aligned, blue background

- [ ] **T040** [US1] [US6] Implement AgentMessage component
  - **File**: `website/src/components/ChatKit/ChatMessage.tsx`
  - **Content**:
    - Render div (flex justify-start mb-4)
    - Inner div (bg-muted rounded-lg p-3 max-w-[80%])
    - Parse citations from message.content using parseCitations()
    - Split content into mainText (before "Sources:") and sources section
    - Display mainText
    - If message.citations.length > 0: Render CitationList component
    - Display timestamp
  - **Acceptance**: Agent messages left-aligned, gray background, citations shown

### Sub-Phase 4.6: CitationList

- [ ] **T041** [US1] [US6] Create CitationList.tsx component
  - **File**: `website/src/components/ChatKit/CitationList.tsx`
  - **Content**:
    - Import ExternalLink icon from lucide-react
    - Prop: `sources: SourceCitation[]`
    - Render div (mt-2 pt-2 border-t)
    - Label: "Sources:" (text-xs font-semibold)
    - Map over sources, render list items
    - Each item: Link with href={source.url}, target="_blank", rel="noopener noreferrer"
    - Display: "[Source {number}] {title || url}"
    - Icon: ExternalLink
  - **Acceptance**: Citations clickable, open in new tab

### Sub-Phase 4.7: ChatInput

- [ ] **T042** [US1] Create ChatInput.tsx skeleton
  - **File**: `website/src/components/ChatKit/ChatInput.tsx`
  - **Content**:
    - Import Textarea, Button from ui components
    - Import Send icon from lucide-react
    - Import useChatContext, useState
    - State: `input` (string), `error` (string | null)
    - Render div (p-4 border-t)
    - If error: Display error message (text-sm text-destructive)
    - Render flex container with Textarea and Send Button
  - **Acceptance**: Input field and send button render

- [ ] **T043** [US1] Implement input handling and validation
  - **File**: `website/src/components/ChatKit/ChatInput.tsx`
  - **Add**:
    - onChange handler: `setInput(e.target.value)`
    - onKeyDown handler: If Enter && !Shift: e.preventDefault(), handleSubmit()
    - handleSubmit function:
      - Validate message using validateMessage()
      - If invalid: setError()
      - If valid: await sendMessage(input), setInput(''), setError(null)
    - Disable textarea and button while uiState.isLoading
  - **Acceptance**: Enter sends, Shift+Enter newline, validation works

- [ ] **T044** [US1] Add character counter to ChatInput
  - **File**: `website/src/components/ChatKit/ChatInput.tsx`
  - **Add**: Below textarea: `<p className="text-xs text-muted-foreground mt-1">{input.length}/5000 characters</p>`
  - **Acceptance**: Counter updates on input

### Sub-Phase 4.8: SelectedContextBadge

- [ ] **T045** [US2] Create SelectedContextBadge.tsx component
  - **File**: `website/src/components/ChatKit/SelectedContextBadge.tsx`
  - **Content**:
    - Import Button from ui components
    - Import useChatContext
    - If !session.selectedContext: return null
    - Render div (px-4 py-2 bg-accent border-t border-b)
    - Label: "Asking about:" (text-xs font-semibold)
    - Preview: Truncate selectedContext.text to 100 chars + "..."
    - Clear button: calls clearContext()
  - **Acceptance**: Badge shows when context selected, clear button works

### Sub-Phase 4.9: LoadingMessage

- [ ] **T046** [US5] Create LoadingMessage.tsx component
  - **File**: `website/src/components/ChatKit/LoadingMessage.tsx`
  - **Content**:
    - Import Loader2 icon from lucide-react
    - State: `showSlowWarning` (boolean)
    - useEffect: setTimeout 10 seconds, set showSlowWarning=true
    - Render div (flex justify-start mb-4)
    - Inner div (bg-muted rounded-lg p-3 max-w-[80%])
    - Display: Loader2 icon (animate-spin), "Thinking..." text
    - If showSlowWarning: Display "This is taking longer than usual..." (text-xs text-muted-foreground mt-2)
  - **Acceptance**: Loading spinner shows, slow warning after 10s

### Sub-Phase 4.10: ErrorMessage

- [ ] **T047** [US5] Create ErrorMessage.tsx component
  - **File**: `website/src/components/ChatKit/ErrorMessage.tsx`
  - **Content**:
    - Import Button from ui components
    - Import useChatContext
    - Prop: `error: UIError`
    - Render div (p-4 bg-destructive/10 border border-destructive rounded-lg)
    - Display error.message
    - If error.retryable: Render Retry button
      - onClick: Get last user message from session.messages
      - Call sendMessage() with last user message.content
  - **Acceptance**: Error displays with retry button when retryable

### Sub-Phase 4.11: ChatWidget (Root Assembly)

- [ ] **T048** [US1] Create ChatWidget.tsx to assemble all components
  - **File**: `website/src/components/ChatKit/ChatWidget.tsx`
  - **Content**:
    - Import ChatButton, ChatWindow
    - Import TooltipProvider from ui components
    - Render:
      ```tsx
      <TooltipProvider>
        <ChatButton />
        <ChatWindow />
      </TooltipProvider>
      ```
  - **Acceptance**: ChatWidget renders button and window

- [ ] **T049** [US1] Update ChatWindow to include all child components
  - **File**: `website/src/components/ChatKit/ChatWindow.tsx`
  - **Edit**: Replace placeholder children with:
    - `<ChatHeader />`
    - `<MessageList />`
    - `{uiState.showContextBadge && <SelectedContextBadge />}`
    - `{uiState.error && <ErrorMessage error={uiState.error} />}`
    - `<ChatInput />`
  - **Acceptance**: All components render in correct order

**Phase 4 Acceptance Criteria**:
- ✅ Chat button opens/closes window
- ✅ Messages display chronologically
- ✅ User can send messages via input
- ✅ Loading indicator appears while waiting
- ✅ Error messages display with retry option
- ✅ Citations render as clickable links
- ✅ Context badge shows when text selected

---

## Phase 5: Text Selection Integration

**Goal**: Enable users to select book text and ask questions about it
**Duration**: 1 day
**Dependencies**: Phase 2, Phase 4

### Tasks

- [ ] **T050** [US2] Create useTextSelection.ts hook
  - **File**: `website/src/hooks/useTextSelection.ts`
  - **Content**:
    - Import useState, useEffect
    - Interface: `SelectionState = {text: string; range: Range | null; sourceSection: string; startOffset: number; endOffset: number}`
    - State: `selection` (SelectionState)
    - useEffect: Listen to 'selectionchange' event
      - On change: Get window.getSelection()
      - If selection exists && text.trim().length >= 10:
        - Extract text, range (getRangeAt(0)), offsets
        - Find sourceSection by traversing DOM for nearest heading (h1/h2/h3)
        - setSelection()
      - Else: setSelection({text: '', range: null, ...})
    - Return selection
  - **Acceptance**: Hook captures text selection, extracts range and source
  - **Reference**: AD-005 (window.getSelection), plan.md (useTextSelection Hook)

- [ ] **T051** [US2] Add getSourceSection helper function
  - **File**: `website/src/hooks/useTextSelection.ts`
  - **Add**:
    ```typescript
    function getSourceSection(node: Node): string {
      let current = node.parentElement;
      while (current) {
        const heading = current.querySelector('h1, h2, h3');
        if (heading) return heading.textContent || 'Unknown Section';
        current = current.parentElement;
      }
      return 'Unknown Section';
    }
    ```
  - **Acceptance**: Finds nearest heading in DOM hierarchy

- [ ] **T052** [US2] Create TextSelectionTooltip.tsx component
  - **File**: `website/src/components/ChatKit/TextSelectionTooltip.tsx`
  - **Content**:
    - Import Button from ui components
    - Import MessageSquare icon from lucide-react
    - Import useChatContext
    - Prop: `selection: SelectionState`
    - If !selection.text: return null
    - Render div (absolute z-50 p-2 bg-background border rounded-lg shadow-lg)
      - Position near selection (use getBoundingClientRect if needed, or simple fixed position)
    - Button "Ask about this" with MessageSquare icon
    - onClick:
      - Build SelectedTextContext from selection
      - Call selectText()
      - Call toggleChat() to open chat if closed
  - **Acceptance**: Tooltip appears when text selected, click opens chat in context mode

- [ ] **T053** [US2] Integrate useTextSelection into ChatWidget
  - **File**: `website/src/components/ChatKit/ChatWidget.tsx`
  - **Edit**:
    - Import useTextSelection
    - Call hook: `const selection = useTextSelection()`
    - Pass to TextSelectionTooltip: `<TextSelectionTooltip selection={selection} />`
  - **Acceptance**: Text selection triggers tooltip

- [ ] **T054** [US2] Add CSS for tooltip positioning
  - **File**: `website/src/components/ChatKit/TextSelectionTooltip.tsx`
  - **Edit**: Add portal rendering or calculate position based on selection.range.getBoundingClientRect()
  - **Acceptance**: Tooltip positioned near selected text (above or below)

- [ ] **T055** [US2] Handle selection clearing on chat close or context clear
  - **File**: `website/src/components/ChatKit/TextSelectionTooltip.tsx`
  - **Edit**: Hide tooltip when uiState.mode === 'full-book' or !uiState.showContextBadge
  - **Acceptance**: Tooltip hides when context cleared

**Phase 5 Acceptance Criteria**:
- ✅ Selecting text (>=10 chars) shows tooltip
- ✅ Clicking "Ask about this" opens chat
- ✅ Chat switches to context-constrained mode
- ✅ Selected text preview appears in chat
- ✅ Clearing context hides tooltip

---

## Phase 6: Root Component Integration

**Goal**: Integrate chat widget globally across all pages
**Duration**: 0.5 day
**Dependencies**: Phase 4, Phase 5

### Tasks

- [ ] **T056** [US1] [US4] Swizzle Docusaurus Root component
  - **Command**: `cd website && npm run swizzle @docusaurus/theme-classic Root -- --wrap`
  - **File**: `website/src/theme/Root.tsx` (or .js)
  - **Acceptance**: Root.tsx file created
  - **Reference**: AD-001 (Root theme component)

- [ ] **T057** [US1] Edit Root.tsx to wrap with ChatProvider
  - **File**: `website/src/theme/Root.tsx`
  - **Content**:
    ```tsx
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
  - **Acceptance**: ChatProvider wraps all pages, ChatWidget rendered globally

- [ ] **T058** [US1] Test chat widget appears on all book pages
  - **Action**: Navigate to multiple pages (docs, home, different chapters)
  - **Verify**: Chat button appears on all pages
  - **Verify**: State persists across navigation within same tab
  - **Acceptance**: Chat works universally

- [ ] **T059** [US4] Verify no SSR/hydration errors
  - **Action**: Run `npm run build && npm run serve`
  - **Check**: Build succeeds, no hydration warnings in browser console
  - **Fix if needed**: Wrap browser-only code (window.getSelection, sessionStorage) in useEffect
  - **Acceptance**: Production build works without errors
  - **Reference**: plan.md (Risk 5 - SSR/Hydration Errors)

**Phase 6 Acceptance Criteria**:
- ✅ Chat button appears on all pages
- ✅ Chat window opens/closes correctly
- ✅ State persists across page navigations (same tab)
- ✅ No hydration errors

---

## Phase 7: Error Handling and Loading States

**Goal**: Improve UX with comprehensive error handling
**Duration**: 0.5 day
**Dependencies**: Phase 4

### Tasks

- [ ] **T060** [US5] Add error boundary for chat components
  - **File**: `website/src/components/ChatKit/ErrorBoundary.tsx`
  - **Content**:
    - Create class component extending React.Component
    - Implement componentDidCatch
    - Render fallback UI with error message
    - Wrap ChatWidget in ErrorBoundary
  - **Acceptance**: Component errors don't crash entire app

- [ ] **T061** [US5] Enhance ErrorMessage with type-specific messages
  - **File**: `website/src/components/ChatKit/ErrorMessage.tsx`
  - **Edit**: Map error.type to user-friendly messages:
    - 'network' → "Unable to connect to server. Please check your connection."
    - 'timeout' → "Request timed out. Please try again."
    - 'validation' → error.message (specific from backend)
    - 'server' → "Something went wrong. Please try again in a moment."
    - 'unknown' → "An unexpected error occurred."
  - **Acceptance**: Correct message for each error type
  - **Reference**: data-model.md (UIError type mapping)

- [ ] **T062** [US5] Add optimistic update rollback on error
  - **File**: `website/src/contexts/ChatContext.tsx`
  - **Edit**: In sendMessage catch block:
    - Remove optimistic user message from session.messages
    - Set uiState.error
  - **Acceptance**: Failed message removed from chat

- [ ] **T063** [US5] Add loading state transitions
  - **File**: `website/src/contexts/ChatContext.tsx`
  - **Verify**:
    - isLoading = true immediately after sendMessage starts
    - isLoading = false after response received or error
  - **Acceptance**: Loading indicator shows/hides correctly

- [ ] **T064** [US5] Test error scenarios
  - **Test**: Stop backend, send message
  - **Verify**: "Unable to connect to server" error displays
  - **Test**: Click Retry button
  - **Verify**: Request sent again
  - **Acceptance**: All error types tested

- [ ] **T065** [US5] Add timeout warning enhancement
  - **File**: `website/src/components/ChatKit/LoadingMessage.tsx`
  - **Verify**: Slow warning appears after 10 seconds
  - **Acceptance**: Warning displays for long requests

**Phase 7 Acceptance Criteria**:
- ✅ Error boundary catches component errors
- ✅ Error messages type-specific and user-friendly
- ✅ Retry button resends last message
- ✅ Loading indicators show/hide correctly
- ✅ Slow warning displays after 10 seconds

---

## Phase 8: Backend CORS Configuration

**Goal**: Configure backend to allow frontend requests
**Duration**: 0.25 day
**Dependencies**: None (backend work)

### Tasks

- [ ] **T066** [US4] Update backend/main.py with CORS middleware
  - **File**: `backend/main.py`
  - **Edit**:
    ```python
    from fastapi.middleware.cors import CORSMiddleware
    import os

    app = FastAPI()

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
  - **Acceptance**: CORS middleware added
  - **Reference**: AD-007 (explicit CORS origins), plan.md (Phase 8)

- [ ] **T067** [US4] Create backend/.env.example with CORS_ORIGINS
  - **File**: `backend/.env.example`
  - **Content**:
    ```
    CORS_ORIGINS=http://localhost:3000
    ```
  - **Acceptance**: Template file committed

- [ ] **T068** [US4] Test CORS in local development
  - **Action**: Start backend (uvicorn main:app --reload)
  - **Action**: Start frontend (npm start)
  - **Action**: Send chat message
  - **Verify**: No CORS errors in browser console
  - **Verify**: Request succeeds
  - **Acceptance**: CORS works locally

**Phase 8 Acceptance Criteria**:
- ✅ CORS middleware configured
- ✅ Local frontend can call local backend
- ✅ No CORS errors in console

---

## Phase 9: Testing

**Goal**: Validate all features with comprehensive tests
**Duration**: 2 days
**Dependencies**: Phase 6, Phase 7, Phase 8

### Sub-Phase 9.1: Unit Tests

- [ ] **T069** [P] Write unit tests for apiClient.ts
  - **File**: `website/src/lib/__tests__/apiClient.test.ts`
  - **Tests**:
    - Timeout after 30 seconds
    - Retry on network error (max 2 retries)
    - Throw APIError(400) on validation error
    - Throw APIError(500) on server error
    - Parse response correctly on success
  - **Framework**: Jest or Vitest
  - **Acceptance**: All apiClient tests pass

- [ ] **T070** [P] Write unit tests for session.ts
  - **File**: `website/src/lib/__tests__/session.test.ts`
  - **Tests**:
    - generateSessionId returns valid UUID v4
    - getOrCreateSessionId returns same ID on repeated calls
    - persistSession saves to sessionStorage
    - loadSession restores from sessionStorage
    - Dates rehydrated correctly
  - **Acceptance**: All session tests pass

- [ ] **T071** [P] Write unit tests for citations.ts
  - **File**: `website/src/lib/__tests__/citations.test.ts`
  - **Tests**:
    - Parse inline citations [Source 1], [Source 2]
    - Parse source list "- [Source 1] <url>"
    - Handle missing sources gracefully
    - Handle multiple citations in same sentence
  - **Acceptance**: All citation tests pass

### Sub-Phase 9.2: Integration Tests

- [ ] **T072** [P] Write integration tests for ChatContext
  - **File**: `website/src/contexts/__tests__/ChatContext.test.tsx`
  - **Tests**:
    - sendMessage adds user message optimistically
    - sendMessage calls API and updates with response
    - sendMessage handles errors correctly
    - selectText switches to context-constrained mode
    - clearContext switches back to full-book mode
    - resetConversation generates new session ID
  - **Framework**: React Testing Library
  - **Acceptance**: All context tests pass

- [ ] **T073** [P] Write component tests for ChatButton, ChatInput
  - **File**: `website/src/components/ChatKit/__tests__/Chat.test.tsx`
  - **Tests**:
    - ChatButton opens chat on click
    - ChatInput validates message
    - ChatInput sends message on Enter
    - ChatInput adds newline on Shift+Enter
  - **Acceptance**: All component tests pass

### Sub-Phase 9.3: End-to-End Tests

- [ ] **T074** Write E2E test for full conversation flow
  - **File**: `website/e2e/chat.spec.ts` (Playwright or Cypress)
  - **Test**:
    1. Open book page
    2. Click chat button
    3. Verify chat window opens
    4. Type question "What is inverse kinematics?"
    5. Click Send
    6. Wait for response
    7. Verify answer appears with citations
    8. Click citation
    9. Verify navigation to source page
  - **Acceptance**: E2E test passes

- [ ] **T075** Write E2E test for context-constrained flow
  - **File**: `website/e2e/chat.spec.ts`
  - **Test**:
    1. Open book page
    2. Select text (simulate drag)
    3. Click "Ask about this" tooltip
    4. Verify chat opens
    5. Verify context badge appears
    6. Type question "Explain this simply"
    7. Send message
    8. Verify response
    9. Click "Clear" on context badge
    10. Verify full-book mode restored
  - **Acceptance**: E2E test passes

- [ ] **T076** Write E2E test for error handling
  - **File**: `website/e2e/chat.spec.ts`
  - **Test**:
    1. Mock backend failure (network error)
    2. Send question
    3. Verify error message displays
    4. Verify Retry button appears
    5. Mock backend success
    6. Click Retry
    7. Verify success
  - **Acceptance**: E2E test passes

- [ ] **T077** Write E2E test for session persistence
  - **File**: `website/e2e/chat.spec.ts`
  - **Test**:
    1. Send message
    2. Navigate to different page
    3. Reopen chat
    4. Verify conversation history persists
    5. Open new tab
    6. Verify new session created
  - **Acceptance**: E2E test passes

**Phase 9 Acceptance Criteria**:
- ✅ All unit tests pass (apiClient, session, citations)
- ✅ All integration tests pass (ChatContext, components)
- ✅ All E2E tests pass (full flow, context mode, errors, sessions)

---

## Phase 10: Documentation and Deployment

**Goal**: Document setup and deploy to production
**Duration**: 0.5 day
**Dependencies**: Phase 9

### Tasks

- [ ] **T078** [US4] Update README.md with chat UI setup
  - **File**: `website/README.md` or root `README.md`
  - **Add sections**:
    - Chat UI Features
    - Setup Instructions (install Tailwind, Shadcn/ui, swizzle Root)
    - Environment Variables (REACT_APP_BACKEND_URL)
    - Local Development Guide
  - **Acceptance**: README documents chat UI

- [ ] **T079** [US4] Test production deployment
  - **Action**: Deploy frontend to Netlify
  - **Config**: Set environment variable REACT_APP_BACKEND_URL to production backend URL
  - **Action**: Deploy backend with CORS_ORIGINS set to production frontend URL
  - **Test**: Open production frontend, send chat message
  - **Verify**: Chat works, no CORS errors
  - **Acceptance**: Production deployment successful

**Phase 10 Acceptance Criteria**:
- ✅ README documents setup
- ✅ Production deployment works
- ✅ Chat functional in production

---

## MVP Task Summary

**MVP Scope** (US1 + US2 + US4): Phases 0-6, 8-9

**Total MVP Tasks**: 62 tasks
- Phase 0: 6 tasks
- Phase 1: 15 tasks
- Phase 2: 8 tasks
- Phase 3: 7 tasks
- Phase 4: 21 tasks
- Phase 5: 6 tasks
- Phase 6: 4 tasks
- Phase 8: 3 tasks
- Phase 9: 9 tasks

**Non-MVP Tasks** (US3, US5, US6 enhancements): 25 tasks
- Phase 7: 6 tasks (US5 - Error Handling)
- Phase 10: 2 tasks (Documentation)
- US3 and US6 functionality included in core phases but enhanced in later phases

---

## Parallelization Opportunities

Tasks marked with **[P]** can be executed in parallel:

**Phase 0**: T004, T005 (env files)
**Phase 1**: T007, T008 (type files), T018 (utils)
**Phase 3**: T027-T032 (all Shadcn/ui components)
**Phase 9**: T069-T073 (all test files)

**Total Parallel Tasks**: 13 tasks

---

## Dependency Graph

```
Phase 0 (Setup)
  └─> Phase 1 (Infrastructure)
       ├─> Phase 2 (Context)
       │    └─> Phase 4 (UI Components)
       │         ├─> Phase 5 (Text Selection)
       │         │    └─> Phase 6 (Root Integration)
       │         │         └─> Phase 9 (Testing)
       │         │              └─> Phase 10 (Docs)
       │         └─> Phase 7 (Error Handling)
       │              └─> Phase 9 (Testing)
       └─> Phase 3 (Shadcn/ui)
            └─> Phase 4 (UI Components)

Phase 8 (Backend CORS) - Independent, can run anytime
```

---

## Critical Path

**MVP Critical Path** (longest dependency chain):
Phase 0 → Phase 1 → Phase 2 → Phase 3 → Phase 4 → Phase 5 → Phase 6 → Phase 9

**Duration**: 7.25 days

With parallel execution of Phase 8 (Backend CORS) alongside frontend work, total MVP time: **~7 days**

---

## Independent Test Criteria

### US1 - Basic Question Answering
- ✅ T034-T049 completed: Chat UI renders
- ✅ T010-T013 completed: API client sends requests
- ✅ T025 completed: sendMessage() works
- ✅ T074 E2E test passes: Full conversation flow validated

**Test**: Click chat button, send "What is inverse kinematics?", verify answer with citations appears.

### US2 - Selected Text Context
- ✅ T050-T055 completed: Text selection integration works
- ✅ T045 completed: Context badge displays
- ✅ T075 E2E test passes: Context-constrained flow validated

**Test**: Select text, click "Ask about this", send question, verify context badge and context-based response.

### US3 - Multi-Turn Conversations
- ✅ T025 completed: sendMessage maintains session
- ✅ T037-T040 completed: Message history displays
- ✅ T074 E2E test passes (extended): Follow-up questions work

**Test**: Send "What is FK?", then "How does it differ from IK?", verify second response understands context.

### US4 - Environment Configuration
- ✅ T001-T006 completed: Env vars configured
- ✅ T066-T068 completed: CORS configured
- ✅ T079 completed: Production deployment tested

**Test**: Run locally (localhost backend), deploy to production (production backend), verify both work without code changes.

### US5 - Error Handling and Loading States
- ✅ T046-T047 completed: Loading and error components
- ✅ T060-T065 completed: Comprehensive error handling
- ✅ T076 E2E test passes: Error scenarios validated

**Test**: Stop backend, send message, verify error + retry button, restart backend, click retry, verify success.

### US6 - Source Citation Navigation
- ✅ T041 completed: CitationList component
- ✅ T040 completed: Citations parsed and rendered

**Test**: Send question, verify citations clickable, click citation, verify navigation to source page.

---

## File Structure After Completion

```
website/
├── src/
│   ├── components/
│   │   ├── ui/                          # Shadcn/ui components (T027-T032)
│   │   │   ├── button.tsx
│   │   │   ├── textarea.tsx
│   │   │   ├── scroll-area.tsx
│   │   │   ├── tooltip.tsx
│   │   │   ├── badge.tsx
│   │   │   └── card.tsx
│   │   └── ChatKit/                     # Chat UI components (T034-T049)
│   │       ├── ChatWidget.tsx           # T048
│   │       ├── ChatButton.tsx           # T034
│   │       ├── ChatWindow.tsx           # T035, T049
│   │       ├── ChatHeader.tsx           # T036
│   │       ├── MessageList.tsx          # T037
│   │       ├── ChatMessage.tsx          # T038-T040
│   │       ├── CitationList.tsx         # T041
│   │       ├── ChatInput.tsx            # T042-T044
│   │       ├── SelectedContextBadge.tsx # T045
│   │       ├── TextSelectionTooltip.tsx # T052
│   │       ├── LoadingMessage.tsx       # T046
│   │       ├── ErrorMessage.tsx         # T047, T061
│   │       └── ErrorBoundary.tsx        # T060
│   ├── contexts/
│   │   └── ChatContext.tsx              # T019-T026
│   ├── hooks/
│   │   ├── useChatContext.ts            # T019
│   │   └── useTextSelection.ts          # T050-T051
│   ├── lib/
│   │   ├── apiClient.ts                 # T010-T013
│   │   ├── config.ts                    # T009
│   │   ├── session.ts                   # T014-T016
│   │   ├── citations.ts                 # T017
│   │   └── utils.ts                     # T018
│   ├── types/
│   │   ├── api.ts                       # T007
│   │   └── chat.ts                      # T008
│   └── theme/
│       └── Root.tsx                     # T056-T057
├── tailwind.config.js                   # T001-T002
├── components.json                      # T003
├── .env.example                         # T004
├── .env.local                           # T005 (gitignored)
└── package.json                         # Updated with dependencies

backend/
├── main.py                              # T066 (CORS added)
└── .env.example                         # T067
```

---

## Next Steps

1. **Start Implementation**: Begin with Phase 0 (Setup)
2. **Sequential Execution**: Follow phase order, mark tasks complete
3. **Parallel Execution**: Execute [P] tasks concurrently where possible
4. **Test Incrementally**: Run tests after each phase to catch issues early
5. **Deploy Early**: Test production deployment after Phase 6 to validate env config

---

## References

- **Spec**: `specs/4-docusaurus-rag-integration/spec.md`
- **Plan**: `specs/4-docusaurus-rag-integration/plan.md`
- **Data Model**: `specs/4-docusaurus-rag-integration/data-model.md`
- **API Contract**: `specs/4-docusaurus-rag-integration/contracts/frontend-backend-api.md`
- **Quickstart**: `specs/4-docusaurus-rag-integration/quickstart.md`
