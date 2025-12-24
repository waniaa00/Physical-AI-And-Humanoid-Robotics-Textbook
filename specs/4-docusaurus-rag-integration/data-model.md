# Data Model: Docusaurus RAG Frontend Integration

**Feature**: 4-docusaurus-rag-integration
**Created**: 2025-12-15
**Purpose**: Define frontend entities and state structures for chat UI integration

## Overview

This document defines the data structures used in the Docusaurus frontend for RAG integration. These entities represent the client-side state and data transfer objects (DTOs) for communicating with the FastAPI backend.

## Entity Relationships

```
Session (1) ──── (n) ChatMessage
Session (1) ──── (0..1) SelectedTextContext
ChatMessage (1) ──── (n) SourceCitation
UIState (1) ──── (1) Session
```

## Core Entities

### 1. ChatMessage

Represents a single message in the conversation thread (user question or agent response).

**Attributes**:
- `id`: string - Unique identifier for the message (UUID v4)
- `role`: 'user' | 'assistant' - Who sent the message
- `content`: string - The message text (user question or agent response)
- `timestamp`: Date - When the message was created
- `citations`: SourceCitation[] - List of source references (empty for user messages)
- `isLoading`: boolean - Whether this message is currently being generated (streaming support)
- `error`: string | null - Error message if message failed to send/receive

**Usage**:
```typescript
interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  citations: SourceCitation[];
  isLoading: boolean;
  error: string | null;
}
```

**Example**:
```typescript
{
  id: "550e8400-e29b-41d4-a716-446655440000",
  role: "user",
  content: "What is inverse kinematics?",
  timestamp: new Date("2025-12-15T10:30:00Z"),
  citations: [],
  isLoading: false,
  error: null
}
```

**Validation Rules**:
- `id` must be valid UUID v4 format
- `role` must be exactly 'user' or 'assistant'
- `content` must be non-empty string (trim whitespace)
- `timestamp` must be valid Date object
- `citations` must be empty array for user messages
- `isLoading` must be false for user messages
- `error` is null for successful messages

---

### 2. Session

Represents an ongoing conversation between the user and the agent.

**Attributes**:
- `sessionId`: string - Unique session identifier (UUID v4, persisted in sessionStorage)
- `messages`: ChatMessage[] - Chronologically ordered list of conversation messages
- `mode`: 'full-book' | 'context-constrained' - Current query mode
- `selectedContext`: SelectedTextContext | null - Active text selection context (null in full-book mode)
- `createdAt`: Date - When the session was initialized
- `lastActivityAt`: Date - Timestamp of last user interaction

**Usage**:
```typescript
interface Session {
  sessionId: string;
  messages: ChatMessage[];
  mode: 'full-book' | 'context-constrained';
  selectedContext: SelectedTextContext | null;
  createdAt: Date;
  lastActivityAt: Date;
}
```

**Example**:
```typescript
{
  sessionId: "7f3a9b2c-4d1e-4f8a-9c5b-2e6f1a3d4b5c",
  messages: [
    { id: "msg-1", role: "user", content: "What is inverse kinematics?", ... },
    { id: "msg-2", role: "assistant", content: "Inverse kinematics (IK) is...", ... }
  ],
  mode: "full-book",
  selectedContext: null,
  createdAt: new Date("2025-12-15T10:00:00Z"),
  lastActivityAt: new Date("2025-12-15T10:35:00Z")
}
```

**Validation Rules**:
- `sessionId` must be valid UUID v4 format
- `messages` must be chronologically ordered (ascending by timestamp)
- `mode` must be exactly 'full-book' or 'context-constrained'
- `selectedContext` must be null when mode is 'full-book'
- `selectedContext` must be non-null when mode is 'context-constrained'
- `lastActivityAt` must be >= `createdAt`

**Lifecycle**:
1. Created when chat widget first opened (generates new UUID v4)
2. Persisted in sessionStorage for current browser tab only
3. Restored on page navigation within same tab
4. Destroyed when user clicks "New Conversation" or tab is closed
5. Session expires on backend after 30 minutes of inactivity (backend enforces)

---

### 3. SelectedTextContext

Represents user-highlighted text from the book used for context-constrained answering.

**Attributes**:
- `text`: string - The selected text content (trimmed)
- `sourceSection`: string - Book section where text was selected (e.g., "Chapter 3: Kinematics")
- `startOffset`: number - Character offset where selection starts
- `endOffset`: number - Character offset where selection ends
- `selectedAt`: Date - When the text was selected

**Usage**:
```typescript
interface SelectedTextContext {
  text: string;
  sourceSection: string;
  startOffset: number;
  endOffset: number;
  selectedAt: Date;
}
```

**Example**:
```typescript
{
  text: "The zero-moment point (ZMP) is the point on the ground where the sum of all moments equals zero.",
  sourceSection: "Chapter 4: Balance and Stability",
  startOffset: 1250,
  endOffset: 1348,
  selectedAt: new Date("2025-12-15T10:32:00Z")
}
```

**Validation Rules**:
- `text` must be non-empty after trimming whitespace
- `text` length must be >= 10 characters (minimum meaningful selection)
- `text` length must be <= 10,000 characters (prevent oversized context)
- `sourceSection` must be non-empty string
- `startOffset` must be >= 0
- `endOffset` must be > `startOffset`
- `selectedAt` must be valid Date object

**Capture Mechanism**:
- Triggered by `window.getSelection()` API when user finishes text selection
- Extracted from DOM using Range API (`sel.getRangeAt(0)`)
- Source section derived from nearest heading element in DOM hierarchy

---

### 4. SourceCitation

Represents a reference to a book section cited in an agent response.

**Attributes**:
- `number`: number - Citation number as referenced in response text (e.g., [Source 1])
- `url`: string - URL to the book section (absolute or relative path)
- `title`: string - Chapter or section title
- `preview`: string | null - Optional preview text from the source (first 100 chars)

**Usage**:
```typescript
interface SourceCitation {
  number: number;
  url: string;
  title: string;
  preview: string | null;
}
```

**Example**:
```typescript
{
  number: 1,
  url: "/docs/kinematics/inverse-kinematics",
  title: "Chapter 3: Inverse Kinematics",
  preview: "Inverse kinematics (IK) involves calculating the joint angles required to position the end-effector..."
}
```

**Validation Rules**:
- `number` must be positive integer (>= 1)
- `url` must be non-empty string
- `url` should be valid URL format (relative or absolute)
- `title` must be non-empty string
- `preview` is optional (can be null)
- `preview` length should be <= 200 characters if present

**Parsing Logic**:
- Extracted from agent response using regex patterns:
  - Inline citations: `\[Source (\d+)\]`
  - Source list: `^- \[Source (\d+)\] (.+)$`
- Citations are matched by number across inline and list sections
- Unmatched citation numbers result in warning (but not error)

---

### 5. UIState

Represents the current state of the chat UI widget.

**Attributes**:
- `isOpen`: boolean - Whether the chat window is currently visible
- `isLoading`: boolean - Whether a request is currently in flight
- `error`: UIError | null - Current error state (null if no error)
- `mode`: 'full-book' | 'context-constrained' - Current query mode (mirrors Session.mode)
- `showContextBadge`: boolean - Whether to display "Based on selected text" indicator

**Usage**:
```typescript
interface UIState {
  isOpen: boolean;
  isLoading: boolean;
  error: UIError | null;
  mode: 'full-book' | 'context-constrained';
  showContextBadge: boolean;
}
```

**Example**:
```typescript
{
  isOpen: true,
  isLoading: false,
  error: null,
  mode: "full-book",
  showContextBadge: false
}
```

**State Transitions**:
```
Initial: { isOpen: false, isLoading: false, error: null, mode: 'full-book', showContextBadge: false }

User clicks chat button:
  → { isOpen: true, ... }

User submits question:
  → { isLoading: true, error: null, ... }

Response received successfully:
  → { isLoading: false, error: null, ... }

Response fails (network error):
  → { isLoading: false, error: { type: 'network', message: '...' }, ... }

User selects text and asks question:
  → { mode: 'context-constrained', showContextBadge: true, ... }

User clears context or clicks "Search full book":
  → { mode: 'full-book', showContextBadge: false, ... }
```

---

### 6. UIError

Represents an error state in the chat UI.

**Attributes**:
- `type`: 'network' | 'timeout' | 'validation' | 'server' | 'unknown' - Error category
- `message`: string - User-friendly error message
- `code`: string | null - Optional error code from backend
- `retryable`: boolean - Whether the user should be offered a retry option

**Usage**:
```typescript
interface UIError {
  type: 'network' | 'timeout' | 'validation' | 'server' | 'unknown';
  message: string;
  code: string | null;
  retryable: boolean;
}
```

**Example**:
```typescript
{
  type: "network",
  message: "Unable to connect to server. Please check your connection.",
  code: null,
  retryable: true
}
```

**Error Type Mapping**:

| Type | Trigger | User Message | Retryable |
|------|---------|--------------|-----------|
| `network` | fetch() throws network error | "Unable to connect to server. Please check your connection." | true |
| `timeout` | Request exceeds 30 seconds | "Request timed out. Please try again." | true |
| `validation` | 400 response from backend | Specific validation message from backend | false |
| `server` | 500+ response from backend | "Something went wrong. Please try again in a moment." | true |
| `unknown` | Unexpected error | "An unexpected error occurred." | true |

---

## Data Transfer Objects (DTOs)

### ChatRequest

Payload sent from frontend to backend `/agent/chat` endpoint.

**Structure**:
```typescript
interface ChatRequest {
  message: string;
  session_id: string;
  context_text: string | null;
}
```

**Example (Full-Book Mode)**:
```json
{
  "message": "What is inverse kinematics?",
  "session_id": "7f3a9b2c-4d1e-4f8a-9c5b-2e6f1a3d4b5c",
  "context_text": null
}
```

**Example (Context-Constrained Mode)**:
```json
{
  "message": "Explain this in simpler terms",
  "session_id": "7f3a9b2c-4d1e-4f8a-9c5b-2e6f1a3d4b5c",
  "context_text": "The zero-moment point (ZMP) is the point on the ground where the sum of all moments equals zero."
}
```

**Validation**:
- `message` must be non-empty string (1-5000 characters)
- `session_id` must be valid UUID v4
- `context_text` is null for full-book queries, non-empty string for context-constrained queries

---

### ChatResponse

Payload received from backend `/agent/chat` endpoint.

**Structure**:
```typescript
interface ChatResponse {
  response: string;
  session_id: string;
  status: 'success' | 'guardrail_triggered' | 'error';
}
```

**Example (Success)**:
```json
{
  "response": "Inverse kinematics (IK) is the process of calculating joint angles required to position a robotic end-effector at a desired location [Source 1]. Unlike forward kinematics which calculates position from angles, IK solves the reverse problem [Source 2].\n\nSources:\n- [Source 1] /docs/kinematics/inverse-kinematics\n- [Source 2] /docs/kinematics/forward-vs-inverse",
  "session_id": "7f3a9b2c-4d1e-4f8a-9c5b-2e6f1a3d4b5c",
  "status": "success"
}
```

**Example (Guardrail Triggered)**:
```json
{
  "response": "I can only answer questions about humanoid robotics based on the book content.",
  "session_id": "7f3a9b2c-4d1e-4f8a-9c5b-2e6f1a3d4b5c",
  "status": "guardrail_triggered"
}
```

**Status Handling**:
- `success`: Parse citations and display response normally
- `guardrail_triggered`: Display response with warning indicator (no citations expected)
- `error`: Display error message to user with retry option

---

## State Management Strategy

### Context Structure

```typescript
interface ChatContextValue {
  // Session state
  session: Session;

  // UI state
  uiState: UIState;

  // Actions
  sendMessage: (message: string) => Promise<void>;
  selectText: (context: SelectedTextContext) => void;
  clearContext: () => void;
  resetConversation: () => void;
  toggleChat: () => void;
}
```

### Provider Implementation

```typescript
// website/src/contexts/ChatContext.tsx
import React, {createContext, useContext, useState, useCallback} from 'react';

const ChatContext = createContext<ChatContextValue | undefined>(undefined);

export function ChatProvider({children}: {children: React.ReactNode}) {
  const [session, setSession] = useState<Session>(() => initializeSession());
  const [uiState, setUIState] = useState<UIState>({
    isOpen: false,
    isLoading: false,
    error: null,
    mode: 'full-book',
    showContextBadge: false,
  });

  const sendMessage = useCallback(async (message: string) => {
    // Implementation details in plan.md
  }, [session]);

  const selectText = useCallback((context: SelectedTextContext) => {
    setSession(prev => ({
      ...prev,
      mode: 'context-constrained',
      selectedContext: context,
    }));
    setUIState(prev => ({
      ...prev,
      mode: 'context-constrained',
      showContextBadge: true,
    }));
  }, []);

  const clearContext = useCallback(() => {
    setSession(prev => ({
      ...prev,
      mode: 'full-book',
      selectedContext: null,
    }));
    setUIState(prev => ({
      ...prev,
      mode: 'full-book',
      showContextBadge: false,
    }));
  }, []);

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

  const toggleChat = useCallback(() => {
    setUIState(prev => ({...prev, isOpen: !prev.isOpen}));
  }, []);

  return (
    <ChatContext.Provider value={{session, uiState, sendMessage, selectText, clearContext, resetConversation, toggleChat}}>
      {children}
    </ChatContext.Provider>
  );
}

export function useChatContext() {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatContext must be used within ChatProvider');
  }
  return context;
}
```

---

## Session Persistence

### sessionStorage Schema

```typescript
// Key: 'chat_session_id'
// Value: string (UUID v4)
sessionStorage.setItem('chat_session_id', '7f3a9b2c-4d1e-4f8a-9c5b-2e6f1a3d4b5c');

// Key: 'chat_session'
// Value: JSON serialized Session object
sessionStorage.setItem('chat_session', JSON.stringify({
  sessionId: '7f3a9b2c-4d1e-4f8a-9c5b-2e6f1a3d4b5c',
  messages: [...],
  mode: 'full-book',
  selectedContext: null,
  createdAt: '2025-12-15T10:00:00.000Z',
  lastActivityAt: '2025-12-15T10:35:00.000Z'
}));
```

### Persistence Logic

```typescript
function initializeSession(): Session {
  // Try to restore from sessionStorage
  const storedSession = sessionStorage.getItem('chat_session');
  if (storedSession) {
    try {
      const parsed = JSON.parse(storedSession);
      // Rehydrate Date objects
      return {
        ...parsed,
        createdAt: new Date(parsed.createdAt),
        lastActivityAt: new Date(parsed.lastActivityAt),
        messages: parsed.messages.map(m => ({
          ...m,
          timestamp: new Date(m.timestamp),
        })),
      };
    } catch (error) {
      console.error('Failed to restore session:', error);
      // Fall through to create new session
    }
  }

  // Create new session
  return createNewSession();
}

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

function persistSession(session: Session): void {
  sessionStorage.setItem('chat_session', JSON.stringify(session));
  sessionStorage.setItem('chat_session_id', session.sessionId);
}
```

---

## Validation Utilities

### Message Validation

```typescript
function validateMessage(message: string): {valid: boolean; error?: string} {
  const trimmed = message.trim();

  if (trimmed.length === 0) {
    return {valid: false, error: 'Message cannot be empty'};
  }

  if (trimmed.length > 5000) {
    return {valid: false, error: 'Message too long (max 5000 characters)'};
  }

  return {valid: true};
}
```

### Context Validation

```typescript
function validateSelectedContext(context: SelectedTextContext): {valid: boolean; error?: string} {
  const trimmed = context.text.trim();

  if (trimmed.length < 10) {
    return {valid: false, error: 'Selected text too short (min 10 characters)'};
  }

  if (trimmed.length > 10000) {
    return {valid: false, error: 'Selected text too long (max 10,000 characters)'};
  }

  if (context.endOffset <= context.startOffset) {
    return {valid: false, error: 'Invalid text range'};
  }

  return {valid: true};
}
```

### Session ID Validation

```typescript
const UUID_V4_REGEX = /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i;

function isValidSessionId(sessionId: string): boolean {
  return UUID_V4_REGEX.test(sessionId);
}
```

---

## Open Questions

1. **Q**: Should we persist conversation history across browser sessions (using localStorage)?
   **A**: No (per spec - sessions are ephemeral within browser session). Using sessionStorage for privacy and simplicity.

2. **Q**: How long should frontend cache session data?
   **A**: For duration of browser session only. sessionStorage automatically clears on tab close.

3. **Q**: Should we implement optimistic UI updates (show user message immediately before backend confirms)?
   **A**: Yes - add message with `isLoading: true` for assistant response while waiting for backend.

4. **Q**: How to handle citation numbers that don't match between inline and source list?
   **A**: Display warning in console but render what we have. Don't block user from seeing response.

5. **Q**: Should we limit conversation history length to prevent memory issues?
   **A**: Yes - implement soft limit of 50 messages. After 50, show warning and suggest starting new conversation.

---

## Appendix: Complete TypeScript Definitions

```typescript
// website/src/types/chat.ts

/**
 * Core chat message in conversation thread
 */
export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  citations: SourceCitation[];
  isLoading: boolean;
  error: string | null;
}

/**
 * Ongoing conversation session
 */
export interface Session {
  sessionId: string;
  messages: ChatMessage[];
  mode: 'full-book' | 'context-constrained';
  selectedContext: SelectedTextContext | null;
  createdAt: Date;
  lastActivityAt: Date;
}

/**
 * User-selected text context for constrained answering
 */
export interface SelectedTextContext {
  text: string;
  sourceSection: string;
  startOffset: number;
  endOffset: number;
  selectedAt: Date;
}

/**
 * Source citation from agent response
 */
export interface SourceCitation {
  number: number;
  url: string;
  title: string;
  preview: string | null;
}

/**
 * Chat UI state
 */
export interface UIState {
  isOpen: boolean;
  isLoading: boolean;
  error: UIError | null;
  mode: 'full-book' | 'context-constrained';
  showContextBadge: boolean;
}

/**
 * UI error state
 */
export interface UIError {
  type: 'network' | 'timeout' | 'validation' | 'server' | 'unknown';
  message: string;
  code: string | null;
  retryable: boolean;
}

/**
 * Request payload to backend
 */
export interface ChatRequest {
  message: string;
  session_id: string;
  context_text: string | null;
}

/**
 * Response payload from backend
 */
export interface ChatResponse {
  response: string;
  session_id: string;
  status: 'success' | 'guardrail_triggered' | 'error';
}

/**
 * Chat context value for provider
 */
export interface ChatContextValue {
  session: Session;
  uiState: UIState;
  sendMessage: (message: string) => Promise<void>;
  selectText: (context: SelectedTextContext) => void;
  clearContext: () => void;
  resetConversation: () => void;
  toggleChat: () => void;
}
```
