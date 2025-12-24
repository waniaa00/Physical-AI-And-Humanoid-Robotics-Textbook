# Research: Docusaurus RAG Frontend Integration

**Branch**: `4-docusaurus-rag-integration` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)

## Overview

This document captures technical research findings for integrating a chat UI component into the Docusaurus frontend to communicate with the FastAPI RAG backend. Research focused on Docusaurus customization patterns, React component integration, API client design, state management, and deployment configuration.

## Research Questions

### RQ1: Docusaurus Custom Component Integration

**Question**: How can we add a custom React chat component to Docusaurus without breaking the existing site structure?

**Findings**:

#### Docusaurus Architecture Overview
- Docusaurus v2/v3 is built on React and uses MDX for content
- Supports custom React components via "swizzling" and theme customization
- Components can be added globally via `@theme/Root` wrapper or per-page via MDX

#### Integration Approaches

**Option 1: Root Theme Component** (RECOMMENDED)
```javascript
// website/src/theme/Root.tsx
import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';

export default function Root({children}) {
  return (
    <>
      {children}
      <ChatWidget /> {/* Globally available chat */}
    </>
  );
}
```

**Advantages**:
- Available on all pages automatically
- Maintains state across navigation
- Single source of truth for chat component

**Disadvantages**:
- Always loaded (small bundle size impact)
- Cannot disable on specific pages easily

**Option 2: MDX Component Import** (Per-page control)
```mdx
---
# In specific .mdx pages
---
import ChatWidget from '@site/src/components/ChatWidget';

## Chapter Content

<ChatWidget />
```

**Advantages**:
- Fine-grained control over where chat appears
- Can pass page-specific props

**Disadvantages**:
- Must import on every page manually
- State lost on navigation between pages
- Inconsistent user experience

**Option 3: Docusaurus Plugin** (Most robust)
```javascript
// website/plugins/chat-widget/index.js
module.exports = function(context, options) {
  return {
    name: 'chat-widget-plugin',
    injectHtmlTags() {
      return {
        postBodyTags: [{
          tagName: 'div',
          attributes: {id: 'chat-root'},
        }],
      };
    },
    getClientModules() {
      return ['./chatWidgetLoader'];
    },
  };
};
```

**Advantages**:
- Clean separation of concerns
- Can be toggled via plugin configuration
- Best practice for Docusaurus extensions

**Disadvantages**:
- More initial setup complexity
- Requires understanding Docusaurus plugin API

**Decision**: Use **Option 1 (Root Theme Component)** for MVP, migrate to **Option 3 (Plugin)** for production.

**Rationale**:
- Root component is quickest to implement and test
- Provides consistent UX across all pages
- Easy to iterate and debug during development
- Plugin approach better for long-term maintainability but adds setup overhead

---

### RQ2: React Component Architecture for Chat UI

**Question**: What component structure and libraries should we use for the chat interface?

**Findings**:

#### Component Hierarchy
```
<ChatWidget>                    // Container with open/close state
  ├─ <ChatButton />            // Floating action button to open chat
  ├─ <ChatWindow>              // Modal/panel when open
  │   ├─ <ChatHeader />        // Title, mode indicator, close button
  │   ├─ <MessageList>         // Scrollable conversation thread
  │   │   ├─ <UserMessage />   // User question bubble
  │   │   ├─ <AgentMessage />  // Agent response with citations
  │   │   └─ <LoadingMessage/> // Typing indicator
  │   ├─ <SelectedContext />   // Shows selected text if present
  │   └─ <ChatInput />         // Text input + send button
  └─ <ContextMenu />           // Right-click "Ask about this"
```

#### UI Library Options

**Option A: No UI Library (Vanilla React + CSS)**
```tsx
// Custom components with Tailwind or vanilla CSS
<div className="chat-bubble">
  <p>{message.content}</p>
</div>
```

**Advantages**:
- Full control over styling
- Matches Docusaurus theme exactly
- No additional dependencies

**Disadvantages**:
- More implementation time
- Reinvent wheel for common patterns
- Accessibility requires manual work

**Option B: Shadcn/ui Components** (RECOMMENDED)
```tsx
import { Button } from "@/components/ui/button"
import { ScrollArea } from "@/components/ui/scroll-area"
import { Dialog } from "@/components/ui/dialog"
```

**Advantages**:
- Copy-paste components (not npm dependency)
- Built on Radix UI primitives (excellent accessibility)
- Tailwind-based (customizable)
- Modern, professional design

**Disadvantages**:
- Requires Tailwind CSS setup in Docusaurus
- Initial setup overhead

**Option C: Chakra UI / MUI**
```tsx
import { Box, Text, Button } from '@chakra-ui/react';
```

**Advantages**:
- Comprehensive component library
- Built-in theming
- Good accessibility

**Disadvantages**:
- Heavy bundle size
- May clash with Docusaurus styling
- Opinionated design system

**Decision**: Use **Shadcn/ui components** with Tailwind CSS.

**Rationale**:
- Best balance of control and productivity
- Excellent accessibility out of the box
- Tailwind integration keeps bundle size reasonable
- Copy-paste approach means only including needed components
- Can customize to match Docusaurus theme

#### State Management

**Option 1: React useState + useContext** (RECOMMENDED for MVP)
```tsx
const ChatContext = createContext();

function ChatProvider({children}) {
  const [messages, setMessages] = useState([]);
  const [sessionId, setSessionId] = useState(() => generateSessionId());
  const [isOpen, setIsOpen] = useState(false);
  const [selectedContext, setSelectedContext] = useState(null);

  return (
    <ChatContext.Provider value={{messages, sessionId, ...}}>
      {children}
    </ChatContext.Provider>
  );
}
```

**Advantages**:
- No extra dependencies
- Simple for small state tree
- Built into React

**Disadvantages**:
- Can get messy with complex state
- Prop drilling for nested components

**Option 2: Zustand**
```tsx
const useChatStore = create((set) => ({
  messages: [],
  addMessage: (msg) => set((state) => ({
    messages: [...state.messages, msg]
  })),
}));
```

**Advantages**:
- Minimal boilerplate
- No prop drilling
- Easy to persist to localStorage
- TypeScript-friendly

**Disadvantages**:
- Additional dependency (~1KB)

**Decision**: **useState + useContext** for MVP.

**Rationale**: State tree is simple enough that Zustand's benefits don't justify the dependency.

---

### RQ3: API Client Design and Error Handling

**Question**: How should we structure HTTP requests to the FastAPI backend with proper error handling?

**Findings**:

#### HTTP Client Options

**Option A: Native fetch API** (RECOMMENDED)
```typescript
const response = await fetch('/api/agent/chat', {
  method: 'POST',
  headers: {'Content-Type': 'application/json'},
  body: JSON.stringify({message, session_id, context_text}),
});
const data = await response.json();
```

**Advantages**:
- No dependencies
- Built into browser
- TypeScript support built-in

**Disadvantages**:
- Manual error handling
- No request/response interceptors
- Must manually handle timeouts

**Option B: Axios**
```typescript
const {data} = await axios.post('/api/agent/chat', {
  message,
  session_id,
  context_text,
});
```

**Advantages**:
- Automatic JSON parsing
- Request/response interceptors
- Built-in timeout support
- Better error handling

**Disadvantages**:
- Additional dependency (~13KB)

**Decision**: **Native fetch** with custom wrapper.

**Rationale**:
- Fetch is sufficient for simple HTTP requests
- Custom wrapper can add error handling, timeouts, and retries
- Avoids dependency for functionality we can easily implement

#### API Client Wrapper

```typescript
// website/src/lib/apiClient.ts
const API_BASE_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
const REQUEST_TIMEOUT = 30000; // 30 seconds

interface ChatRequest {
  message: string;
  session_id: string;
  context_text: string | null;
}

interface ChatResponse {
  response: string;
  session_id: string;
  status: 'success' | 'guardrail_triggered' | 'error';
}

class APIError extends Error {
  constructor(
    public status: number,
    public code: string,
    message: string
  ) {
    super(message);
  }
}

async function sendMessage(request: ChatRequest): Promise<ChatResponse> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), REQUEST_TIMEOUT);

  try {
    const response = await fetch(`${API_BASE_URL}/agent/chat`, {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify(request),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      if (response.status === 400) {
        throw new APIError(400, 'VALIDATION_ERROR', 'Invalid request');
      }
      if (response.status >= 500) {
        throw new APIError(500, 'SERVER_ERROR', 'Server error occurred');
      }
      throw new APIError(response.status, 'UNKNOWN_ERROR', 'Request failed');
    }

    return await response.json();
  } catch (error) {
    if (error.name === 'AbortError') {
      throw new APIError(408, 'TIMEOUT', 'Request timed out');
    }
    throw error;
  }
}

export const apiClient = {
  sendMessage,
  // Future: getSessionHistory, clearSession, etc.
};
```

**Benefits**:
- Centralized error handling
- Timeout support
- Easy to test and mock
- TypeScript types for requests/responses

---

### RQ4: Selected Text Capture and Context Menu

**Question**: How should we capture user text selection and provide "Ask about this" functionality?

**Findings**:

#### Text Selection Capture

**Approach: window.getSelection() + Event Listeners**

```typescript
// website/src/hooks/useTextSelection.ts
import {useState, useEffect} from 'react';

interface SelectionState {
  text: string;
  range: Range | null;
}

function useTextSelection() {
  const [selection, setSelection] = useState<SelectionState>({
    text: '',
    range: null,
  });

  useEffect(() => {
    function handleSelectionChange() {
      const sel = window.getSelection();
      if (sel && sel.toString().trim().length > 0) {
        setSelection({
          text: sel.toString().trim(),
          range: sel.getRangeAt(0),
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
```

**Advantages**:
- Native browser API
- Works for any selectable text
- No library dependencies

**Limitations**:
- Only captures text, not source context (chapter, section)
- Cannot differentiate between book content and UI text

**Enhancement: Add data attributes to content**
```mdx
<div data-book-section="chapter-3" data-book-page="kinematics">
  <!-- Book content here -->
</div>
```

Then extract context:
```typescript
function getSelectionContext(range: Range) {
  const container = range.commonAncestorContainer;
  const section = container.closest('[data-book-section]');
  return {
    section: section?.getAttribute('data-book-section'),
    page: section?.getAttribute('data-book-page'),
  };
}
```

#### Context Menu Implementation

**Option 1: Browser Context Menu** (Right-click)

```typescript
// Intercept browser context menu
useEffect(() => {
  function handleContextMenu(e: MouseEvent) {
    const sel = window.getSelection();
    if (sel && sel.toString().trim()) {
      e.preventDefault();
      // Show custom menu at e.clientX, e.clientY
    }
  }

  document.addEventListener('contextmenu', handleContextMenu);
  return () => document.removeEventListener('contextmenu', handleContextMenu);
}, []);
```

**Advantages**:
- Familiar UX pattern
- Works on desktop

**Disadvantages**:
- Overrides browser context menu (may annoy power users)
- Doesn't work well on mobile

**Option 2: Floating Tooltip Button** (RECOMMENDED)

```tsx
function SelectionTooltip({selection, onAskAbout}) {
  const [position, setPosition] = useState({x: 0, y: 0});

  useEffect(() => {
    if (selection.range) {
      const rect = selection.range.getBoundingClientRect();
      setPosition({
        x: rect.left + rect.width / 2,
        y: rect.top - 40, // Above selection
      });
    }
  }, [selection]);

  if (!selection.text) return null;

  return (
    <div
      style={{
        position: 'fixed',
        left: position.x,
        top: position.y,
        transform: 'translateX(-50%)',
      }}
    >
      <Button onClick={() => onAskAbout(selection.text)}>
        Ask about this
      </Button>
    </div>
  );
}
```

**Advantages**:
- Works on mobile and desktop
- Non-intrusive (doesn't override browser features)
- Clear call-to-action

**Disadvantages**:
- Requires positioning logic

**Decision**: **Floating tooltip button** for better mobile support.

---

### RQ5: Environment Variable Configuration

**Question**: How should we configure backend URL for local vs. production environments in Docusaurus?

**Findings**:

#### Docusaurus Environment Variables

Docusaurus supports environment variables but with specific conventions:

**Setup**:
1. Create `.env` file in website directory:
```bash
# website/.env
BACKEND_URL=http://localhost:8000
```

2. Create `.env.production` for production builds:
```bash
# website/.env.production
BACKEND_URL=https://api.humanoid-robotics.com
```

3. Access in code via `process.env`:
```typescript
const BACKEND_URL = process.env.BACKEND_URL || 'http://localhost:8000';
```

**Important Notes**:
- Docusaurus uses Webpack's DefinePlugin
- Environment variables are **replaced at build time** (not runtime)
- Cannot change after build without rebuild
- Only variables prefixed with `DOCUSAURUS_` are exposed by default (or all if using custom config)

**Custom Configuration**:
```javascript
// docusaurus.config.js
module.exports = {
  customFields: {
    backendUrl: process.env.BACKEND_URL || 'http://localhost:8000',
  },
};
```

Access in code:
```typescript
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function ChatWidget() {
  const {siteConfig} = useDocusaurusContext();
  const backendUrl = siteConfig.customFields.backendUrl;
  // ...
}
```

**Decision**: Use `customFields` in `docusaurus.config.js` for type safety and centralization.

**Rationale**:
- Single source of truth for configuration
- Type-safe access via useDocusaurusContext
- Clear where config comes from
- Fallback defaults for development

#### Docker/Deployment Strategy

For runtime configuration (non-Webpack approach):

```dockerfile
# Dockerfile for frontend
FROM node:18 AS builder
WORKDIR /app
COPY website/package*.json ./
RUN npm ci
COPY website/ ./
ARG BACKEND_URL
ENV BACKEND_URL=$BACKEND_URL
RUN npm run build

FROM nginx:alpine
COPY --from=builder /app/build /usr/share/nginx/html
```

Then at deployment:
```bash
docker build --build-arg BACKEND_URL=https://api.prod.com -t frontend .
```

**Note**: This is build-time, not runtime. For true runtime config, need to:
1. Generate a `config.js` file at container startup
2. Inject via Nginx or server-side rendering

**Decision**: Build-time configuration is sufficient for MVP. Runtime config is future enhancement.

---

### RQ6: CORS Configuration on Backend

**Question**: What CORS settings are needed for frontend-backend communication?

**Findings**:

#### FastAPI CORS Setup

Backend must allow frontend origin. Add to `backend/main.py`:

```python
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI()

# CORS configuration
origins = [
    "http://localhost:3000",  # Local Docusaurus dev server
    "http://localhost:8000",  # Backend dev server (for testing)
    "https://humanoid-robotics.netlify.app",  # Production frontend
    "https://humanoid-robotics.com",  # Custom domain
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],  # Or ["GET", "POST"] for stricter control
    allow_headers=["*"],
)
```

**Environment-Based CORS**:
```python
import os

FRONTEND_URL = os.getenv("FRONTEND_URL", "http://localhost:3000")

origins = [
    FRONTEND_URL,
    "http://localhost:3000",  # Always allow local dev
]
```

**Wildcard for Development** (not recommended for production):
```python
if os.getenv("ENV") == "development":
    allow_origins=["*"]  # Allow all origins in dev
else:
    allow_origins=origins  # Restrict in production
```

**Decision**: Use environment-based CORS with explicit origin list.

**Rationale**:
- Security: No wildcards in production
- Flexibility: Can add/remove origins via environment variable
- Development: Localhost always allowed for easy testing

---

### RQ7: Session ID Generation and Management

**Question**: How should we generate and persist session IDs for conversation continuity?

**Findings**:

#### Session ID Generation

**Option 1: UUID v4** (RECOMMENDED)
```typescript
function generateSessionId(): string {
  return crypto.randomUUID(); // Native browser API (ES2015+)
  // Output: "9b1deb4d-3b7d-4bad-9bdd-2b0d7b3dcb6d"
}
```

**Advantages**:
- Collision-resistant
- No dependencies
- Standard format

**Option 2: Timestamp + Random**
```typescript
function generateSessionId(): string {
  return `${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  // Output: "1702656000000_k7x9m2p"
}
```

**Advantages**:
- Sortable by time
- Shorter than UUID

**Disadvantages**:
- Collision risk (though very low)
- Non-standard format

**Decision**: **UUID v4** for better collision resistance and standard format.

#### Session Persistence

**Option 1: sessionStorage** (RECOMMENDED for MVP)
```typescript
function getOrCreateSessionId(): string {
  let sessionId = sessionStorage.getItem('chat_session_id');
  if (!sessionId) {
    sessionId = crypto.randomUUID();
    sessionStorage.setItem('chat_session_id', sessionId);
  }
  return sessionId;
}
```

**Advantages**:
- Per-tab isolation (new tab = new session)
- Cleared when tab closes
- Privacy-friendly

**Disadvantages**:
- Lost on tab close
- Not shared across tabs

**Option 2: localStorage**
```typescript
// Persist across browser sessions
sessionId = localStorage.getItem('chat_session_id') || crypto.randomUUID();
```

**Advantages**:
- Persists across browser restarts
- Shared across tabs

**Disadvantages**:
- May lead to stale sessions
- Privacy concerns (long-term tracking)

**Option 3: URL Parameter**
```typescript
const params = new URLSearchParams(window.location.search);
const sessionId = params.get('session') || crypto.randomUUID();
```

**Advantages**:
- Shareable conversations
- Explicit session control

**Disadvantages**:
- Pollutes URL
- User can tamper

**Decision**: **sessionStorage** for MVP (per-tab sessions that clear on close).

**Rationale**:
- Most privacy-friendly
- Natural session boundaries (tab close)
- Avoids stale session issues
- Can upgrade to localStorage or URL params later if needed

---

### RQ8: Citation Rendering and Link Handling

**Question**: How should we parse and render source citations from agent responses?

**Findings**:

#### Citation Format from Backend

Backend returns responses with:
```
The zero-moment point (ZMP) is a key concept [Source 1].

Sources:
- [Source 1] https://example.com/chapter3
- [Source 2] https://example.com/chapter5
```

#### Parsing Strategy

**Regex Parsing** (simple, works for structured format):
```typescript
function parseCitations(response: string) {
  const citationRegex = /\[Source (\d+)\]/g;
  const inlineCitations = [...response.matchAll(citationRegex)].map(m => ({
    number: parseInt(m[1]),
    placeholder: m[0],
  }));

  const sourcesRegex = /^- \[Source (\d+)\] (.+)$/gm;
  const sources = [...response.matchAll(sourcesRegex)].map(m => ({
    number: parseInt(m[1]),
    url: m[2].trim(),
  }));

  return {inlineCitations, sources};
}
```

#### Rendering Component

```tsx
function AgentMessage({content}: {content: string}) {
  const {inlineCitations, sources} = parseCitations(content);

  // Split content into parts and citations sections
  const [mainText, sourcesSection] = content.split('\n\nSources:\n');

  // Replace inline citations with links
  const renderTextWithCitations = () => {
    let parts = [mainText];
    inlineCitations.forEach(cit => {
      const source = sources.find(s => s.number === cit.number);
      if (source) {
        parts = parts.flatMap(part =>
          typeof part === 'string'
            ? part.split(cit.placeholder).flatMap((text, i, arr) =>
                i < arr.length - 1
                  ? [text, <CitationLink key={cit.number} href={source.url} number={cit.number} />]
                  : [text]
              )
            : [part]
        );
      }
    });
    return parts;
  };

  return (
    <div className="agent-message">
      <div className="message-content">
        {renderTextWithCitations()}
      </div>
      {sources.length > 0 && (
        <div className="sources-list">
          <strong>Sources:</strong>
          <ul>
            {sources.map(src => (
              <li key={src.number}>
                <a href={src.url} target="_blank" rel="noopener">
                  [Source {src.number}] {src.url}
                </a>
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
}

function CitationLink({href, number}: {href: string; number: number}) {
  return (
    <a
      href={href}
      className="citation-link"
      target="_blank"
      rel="noopener noreferrer"
      title={`Jump to source ${number}`}
    >
      [Source {number}]
    </a>
  );
}
```

**Decision**: Use regex parsing with React component rendering.

**Rationale**:
- Backend citation format is structured and predictable
- Regex is sufficient for this use case
- Component approach allows styling and interaction
- Can enhance later with markdown parsing if needed

---

## Technical Decisions Summary

| Decision Area | Choice | Rationale |
|--------------|--------|-----------|
| **Component Integration** | Root Theme Component (`@theme/Root`) | Globally available, maintains state, simplest for MVP |
| **UI Library** | Shadcn/ui + Tailwind CSS | Balance of control and productivity, excellent accessibility |
| **State Management** | React useState + useContext | Sufficient for simple state tree, no extra dependencies |
| **HTTP Client** | Native fetch with custom wrapper | No dependencies, sufficient for simple requests, custom error handling |
| **Text Selection** | window.getSelection() + useTextSelection hook | Native API, works across all content |
| **Context Menu** | Floating tooltip button | Mobile-friendly, non-intrusive |
| **Environment Config** | customFields in docusaurus.config.js | Type-safe, centralized, clear defaults |
| **CORS** | Environment-based explicit origin list | Secure, flexible, allows local dev |
| **Session ID** | UUID v4 via crypto.randomUUID() | Collision-resistant, standard format |
| **Session Storage** | sessionStorage (per-tab) | Privacy-friendly, natural session boundaries |
| **Citation Parsing** | Regex with React components | Sufficient for structured format, allows styling |

---

## Dependencies and Environment Setup

### NPM Packages to Install

```json
{
  "dependencies": {
    "@docusaurus/core": "^3.0.0",  // Existing
    "@docusaurus/preset-classic": "^3.0.0",  // Existing
    "react": "^18.0.0",  // Existing
    "react-dom": "^18.0.0",  // Existing
    // NEW:
    "clsx": "^2.0.0",  // Utility for conditional classes
    "tailwindcss": "^3.4.0",  // For Shadcn/ui
    "tailwind-merge": "^2.2.0",  // Merge Tailwind classes
    "@radix-ui/react-dialog": "^1.0.5",  // Modal for chat window
    "@radix-ui/react-scroll-area": "^1.0.5",  // Scrollable message list
    "lucide-react": "^0.300.0"  // Icons (optional)
  },
  "devDependencies": {
    "typescript": "^5.3.0",  // Existing
    "@types/react": "^18.2.0",  // Existing
    "autoprefixer": "^10.4.0",  // For Tailwind
    "postcss": "^8.4.0"  // For Tailwind
  }
}
```

### Environment Variables

**Local Development (.env)**:
```bash
BACKEND_URL=http://localhost:8000
```

**Production (.env.production)**:
```bash
BACKEND_URL=https://api.humanoid-robotics.com
```

### Project Structure After Implementation

```
website/
├── docusaurus.config.js       # Updated with customFields
├── tailwind.config.js          # NEW: Tailwind configuration
├── postcss.config.js           # NEW: PostCSS for Tailwind
├── .env                        # NEW: Local env vars
├── .env.production             # NEW: Production env vars
├── src/
│   ├── theme/
│   │   └── Root.tsx            # NEW: Root wrapper for chat
│   ├── components/
│   │   ├── ChatWidget/
│   │   │   ├── index.tsx       # Main widget container
│   │   │   ├── ChatButton.tsx  # Floating action button
│   │   │   ├── ChatWindow.tsx  # Modal/panel when open
│   │   │   ├── MessageList.tsx # Conversation thread
│   │   │   ├── ChatInput.tsx   # Text input + send
│   │   │   └── SelectionTooltip.tsx  # "Ask about this" button
│   │   └── ui/                 # NEW: Shadcn/ui components
│   │       ├── button.tsx
│   │       ├── dialog.tsx
│   │       ├── scroll-area.tsx
│   │       └── input.tsx
│   ├── hooks/
│   │   ├── useTextSelection.ts # NEW: Capture text selection
│   │   └── useChatSession.ts   # NEW: Session management
│   ├── lib/
│   │   ├── apiClient.ts        # NEW: HTTP client wrapper
│   │   ├── sessionManager.ts   # NEW: Session ID handling
│   │   └── utils.ts            # NEW: Utility functions
│   └── css/
│       └── custom.css          # Updated with Tailwind imports
```

---

## Open Questions for Design Phase

1. **Chat Window Positioning**: Fixed bottom-right corner or side panel?
   - **Recommendation**: Fixed bottom-right (like Intercom, common pattern)

2. **Mobile Responsive Behavior**: Full-screen modal on mobile or compact view?
   - **Recommendation**: Full-screen modal on small screens (<768px)

3. **Keyboard Shortcuts**: Should we add shortcuts like Cmd+K to open chat?
   - **Recommendation**: Yes, Cmd+K or Ctrl+K for power users

4. **Loading State**: Typing indicator style (dots, spinner, "Thinking...")?
   - **Recommendation**: Typing dots animation (familiar pattern)

5. **Max Message Length**: Should we limit input characters?
   - **Recommendation**: 2000 characters (matches backend validation)

6. **Session Expiry UI**: Notify user when session expires?
   - **Recommendation**: Yes, show message "Session expired, starting new conversation"

7. **Accessibility**: ARIA labels and screen reader support needed?
   - **Recommendation**: Yes, full WCAG 2.1 AA compliance with Radix UI

These questions will be resolved during data model and contract design phases.

---

## References

- Docusaurus Documentation: https://docusaurus.io/docs
- Shadcn/ui Components: https://ui.shadcn.com/
- Radix UI Primitives: https://www.radix-ui.com/
- Tailwind CSS: https://tailwindcss.com/
- MDN Web Selection API: https://developer.mozilla.org/en-US/docs/Web/API/Selection

---

**Research Completed**: 2025-12-15
**Next Step**: Create data model definitions (data-model.md) and API contracts
