# Frontend-Backend API Contract

**Feature**: 4-docusaurus-rag-integration
**Created**: 2025-12-15
**Purpose**: Define HTTP API contract between Docusaurus frontend and FastAPI backend

## Overview

This document defines the API contract for communication between the Docusaurus chat UI (frontend) and the FastAPI RAG agent (backend). The frontend consumes the existing `/agent/chat` endpoint implemented in Feature 3.

## Base Configuration

### Endpoints

**Local Development**:
- Base URL: `http://localhost:8000`
- Agent Endpoint: `http://localhost:8000/agent/chat`

**Production**:
- Base URL: `https://api.humanoid-robotics.com` (or configured production URL)
- Agent Endpoint: `https://api.humanoid-robotics.com/agent/chat`

### Environment Variables

**Frontend** (Docusaurus):
```javascript
// website/docusaurus.config.js
module.exports = {
  customFields: {
    backendUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',
  },
};
```

**Backend** (FastAPI):
```python
# backend/.env
CORS_ORIGINS=http://localhost:3000,https://humanoid-robotics.netlify.app
```

### CORS Configuration

**Required Headers**:
- `Access-Control-Allow-Origin`: Explicit origin (no wildcard in production)
- `Access-Control-Allow-Methods`: POST, OPTIONS
- `Access-Control-Allow-Headers`: Content-Type, Authorization
- `Access-Control-Allow-Credentials`: true

**Allowed Origins**:
- Local development: `http://localhost:3000` (Docusaurus dev server)
- Production: `https://humanoid-robotics.netlify.app` (or configured production domain)

---

## API Endpoints

### POST /agent/chat

Send a user question to the RAG agent and receive a grounded response.

**Request**:

```http
POST /agent/chat HTTP/1.1
Host: localhost:8000
Content-Type: application/json

{
  "message": "What is inverse kinematics?",
  "session_id": "7f3a9b2c-4d1e-4f8a-9c5b-2e6f1a3d4b5c",
  "context_text": null
}
```

**Request Schema**:

```typescript
interface ChatRequest {
  message: string;        // User question (1-5000 characters)
  session_id: string;     // UUID v4 format
  context_text: string | null;  // Selected text for context-constrained mode, or null for full-book mode
}
```

**Request Validation**:
- `message`: Required, non-empty string, max 5000 characters
- `session_id`: Required, valid UUID v4 format
- `context_text`: Optional, if provided must be non-empty string (max 10,000 characters)

**Response (Success)**:

```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "response": "Inverse kinematics (IK) is the process of calculating joint angles required to position a robotic end-effector at a desired location [Source 1]. Unlike forward kinematics which calculates position from angles, IK solves the reverse problem [Source 2].\n\nSources:\n- [Source 1] /docs/kinematics/inverse-kinematics\n- [Source 2] /docs/kinematics/forward-vs-inverse",
  "session_id": "7f3a9b2c-4d1e-4f8a-9c5b-2e6f1a3d4b5c",
  "status": "success"
}
```

**Response Schema**:

```typescript
interface ChatResponse {
  response: string;       // Agent's answer with inline citations and source list
  session_id: string;     // Same session ID from request (or new if expired)
  status: 'success' | 'guardrail_triggered' | 'error';
}
```

**Response Status Codes**:

| Code | Meaning | Frontend Action |
|------|---------|-----------------|
| 200 | Success | Parse citations, display response |
| 400 | Validation error (invalid request) | Display specific error message to user |
| 408 | Request timeout (>30 seconds) | Display timeout error, offer retry |
| 500 | Server error | Display generic error, offer retry |
| 503 | Service unavailable | Display "Service temporarily unavailable" |

**Error Response (400 Validation Error)**:

```http
HTTP/1.1 400 Bad Request
Content-Type: application/json

{
  "detail": "Message cannot be empty"
}
```

**Error Response (500 Server Error)**:

```http
HTTP/1.1 500 Internal Server Error
Content-Type: application/json

{
  "detail": "An internal error occurred"
}
```

---

## Request/Response Examples

### Example 1: Full-Book Question Answering

**Request**:
```json
{
  "message": "What are the main challenges in bipedal walking?",
  "session_id": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
  "context_text": null
}
```

**Response**:
```json
{
  "response": "The main challenges in bipedal walking include:\n\n1. **Balance and Stability** - Maintaining the zero-moment point (ZMP) within the support polygon [Source 1]\n2. **Energy Efficiency** - Minimizing power consumption during gait cycles [Source 2]\n3. **Terrain Adaptation** - Handling uneven surfaces and obstacles [Source 3]\n\nSources:\n- [Source 1] /docs/balance/zmp-control\n- [Source 2] /docs/gait/energy-optimization\n- [Source 3] /docs/locomotion/terrain-adaptation",
  "session_id": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
  "status": "success"
}
```

---

### Example 2: Context-Constrained Explanation

**User selects text**: "The zero-moment point (ZMP) is the point on the ground where the sum of all moments equals zero."

**Request**:
```json
{
  "message": "Explain this in simpler terms",
  "session_id": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
  "context_text": "The zero-moment point (ZMP) is the point on the ground where the sum of all moments equals zero."
}
```

**Response**:
```json
{
  "response": "The zero-moment point (ZMP) is essentially the spot on the ground where a robot is perfectly balanced. Think of it like the center of a seesaw - when the seesaw is balanced, the forces on both sides cancel out. For a walking robot, the ZMP is the point where all the twisting forces (moments) from the robot's weight and movement balance out to zero. If the ZMP stays inside the robot's footprint, the robot won't tip over.",
  "session_id": "a1b2c3d4-e5f6-4a7b-8c9d-0e1f2a3b4c5d",
  "status": "success"
}
```

**Note**: Response does NOT include sources because agent was instructed to explain based only on provided context (no retrieval tool call).

---

### Example 3: Multi-Turn Conversation

**First Turn - Initial Question**:

Request:
```json
{
  "message": "What is forward kinematics?",
  "session_id": "b2c3d4e5-f6a7-4b8c-9d0e-1f2a3b4c5d6e",
  "context_text": null
}
```

Response:
```json
{
  "response": "Forward kinematics (FK) is the process of calculating the position and orientation of a robot's end-effector based on given joint angles [Source 1]. It uses the robot's kinematic model to transform joint space coordinates into Cartesian space coordinates [Source 2].\n\nSources:\n- [Source 1] /docs/kinematics/forward-kinematics\n- [Source 2] /docs/kinematics/coordinate-transforms",
  "session_id": "b2c3d4e5-f6a7-4b8c-9d0e-1f2a3b4c5d6e",
  "status": "success"
}
```

**Second Turn - Follow-Up Question**:

Request:
```json
{
  "message": "How does it differ from inverse kinematics?",
  "session_id": "b2c3d4e5-f6a7-4b8c-9d0e-1f2a3b4c5d6e",
  "context_text": null
}
```

Response:
```json
{
  "response": "The key difference is the direction of calculation:\n\n**Forward Kinematics (FK)**: Calculates end-effector position FROM joint angles (joints → position)\n**Inverse Kinematics (IK)**: Calculates joint angles FROM desired position (position → joints) [Source 1]\n\nFK always has a unique solution because there's only one way to calculate the final position from a set of joint angles. However, IK often has multiple solutions (or sometimes no solution) because there may be many ways to reach the same position [Source 2].\n\nSources:\n- [Source 1] /docs/kinematics/forward-vs-inverse\n- [Source 2] /docs/kinematics/ik-solutions",
  "session_id": "b2c3d4e5-f6a7-4b8c-9d0e-1f2a3b4c5d6e",
  "status": "success"
}
```

**Note**: Backend maintains conversation history using session_id, allowing it to understand "it" refers to forward kinematics.

---

### Example 4: Guardrail Triggered (Off-Topic Question)

**Request**:
```json
{
  "message": "What's the weather like today?",
  "session_id": "c3d4e5f6-a7b8-4c9d-0e1f-2a3b4c5d6e7f",
  "context_text": null
}
```

**Response**:
```json
{
  "response": "I can only answer questions about humanoid robotics based on the book content. Please ask a question related to robotics topics like kinematics, balance, gait, or control systems.",
  "session_id": "c3d4e5f6-a7b8-4c9d-0e1f-2a3b4c5d6e7f",
  "status": "guardrail_triggered"
}
```

**Frontend Handling**: Display response with warning indicator (e.g., yellow badge "Off-topic"). No citations expected.

---

### Example 5: Validation Error

**Request** (empty message):
```json
{
  "message": "",
  "session_id": "d4e5f6a7-b8c9-4d0e-1f2a-3b4c5d6e7f8a",
  "context_text": null
}
```

**Response**:
```http
HTTP/1.1 400 Bad Request
Content-Type: application/json

{
  "detail": "Message cannot be empty"
}
```

**Frontend Handling**: Display validation error "Please enter a question" below input field.

---

### Example 6: Network Error (Backend Unreachable)

**Request**: Sent but backend is down/unreachable

**Frontend Handling**:
```typescript
try {
  const response = await fetch(`${API_BASE_URL}/agent/chat`, {...});
} catch (error) {
  if (error.name === 'TypeError' && error.message.includes('fetch')) {
    // Network error
    setError({
      type: 'network',
      message: 'Unable to connect to server. Please check your connection.',
      code: null,
      retryable: true,
    });
  }
}
```

**User sees**: "Unable to connect to server. Please check your connection." with Retry button.

---

## Response Format Specification

### Citation Format

Agent responses follow this format:

```
[Main answer text with inline citations like [Source 1] and [Source 2]]

Sources:
- [Source 1] <url>
- [Source 2] <url>
```

**Parsing Rules**:
1. **Inline Citations**: Match `\[Source (\d+)\]` pattern
2. **Source List**: Match `^- \[Source (\d+)\] (.+)$` pattern (multiline)
3. **URL Extraction**: Text after `[Source N] ` is the URL
4. **Relative URLs**: Prefixed with Docusaurus base URL (e.g., `/docs/...` becomes `https://example.com/docs/...`)

**Example Parsing**:

Input:
```
Inverse kinematics solves for joint angles [Source 1].

Sources:
- [Source 1] /docs/kinematics/inverse-kinematics
```

Parsed:
```typescript
{
  inlineCitations: [{number: 1, placeholder: '[Source 1]'}],
  sources: [{number: 1, url: '/docs/kinematics/inverse-kinematics', title: 'Inverse Kinematics'}]
}
```

---

## Session Management

### Session Lifecycle

1. **Creation**: Frontend generates UUID v4 on first chat interaction
2. **Persistence**: Session ID stored in `sessionStorage` (per-tab, cleared on tab close)
3. **Backend Tracking**: Backend maintains session history for 30 minutes of inactivity
4. **Expiry Handling**: If backend session expires, backend creates new session and returns new session_id
5. **Frontend Response**: Frontend updates session_id if backend returns different ID

### Session Expiry Flow

**Scenario**: User returns to chat after 35 minutes (backend session expired)

Request (old session ID):
```json
{
  "message": "What is inverse kinematics?",
  "session_id": "old-expired-session-id",
  "context_text": null
}
```

Response (new session ID):
```json
{
  "response": "...",
  "session_id": "new-fresh-session-id",
  "status": "success"
}
```

Frontend Handling:
```typescript
const response = await sendMessage(request);
if (response.session_id !== currentSessionId) {
  // Backend created new session
  updateSessionId(response.session_id);
  // Optionally show banner: "Previous conversation expired, started new session"
}
```

---

## Timeout and Retry Configuration

### Timeouts

**Frontend Request Timeout**: 30 seconds
```typescript
const controller = new AbortController();
const timeoutId = setTimeout(() => controller.abort(), 30000);

fetch(url, {signal: controller.signal})
  .then(...)
  .finally(() => clearTimeout(timeoutId));
```

**Backend Processing Timeout**: Agent has 120 seconds max execution time (configured in backend)

### Retry Policy

**Retryable Errors**:
- Network errors (fetch failed)
- Timeout errors (408)
- Server errors (500, 503)

**Non-Retryable Errors**:
- Validation errors (400)
- Guardrail triggered (200 with status='guardrail_triggered')

**Retry Implementation**:
```typescript
async function sendMessageWithRetry(
  request: ChatRequest,
  maxRetries: number = 2
): Promise<ChatResponse> {
  let lastError: Error;

  for (let attempt = 0; attempt < maxRetries; attempt++) {
    try {
      return await sendMessage(request);
    } catch (error) {
      lastError = error;

      // Don't retry on validation errors
      if (error.status === 400) {
        throw error;
      }

      // Wait before retry (exponential backoff)
      if (attempt < maxRetries - 1) {
        await new Promise(resolve => setTimeout(resolve, 1000 * Math.pow(2, attempt)));
      }
    }
  }

  throw lastError;
}
```

---

## Security Considerations

### CORS

- **Production**: MUST use explicit origin list (no `*` wildcard)
- **Credentials**: `allow_credentials=True` for session cookies (if implemented)
- **Headers**: Restrict to `Content-Type`, `Authorization` only

### Input Validation

**Frontend**:
- Validate message length (1-5000 chars)
- Validate context_text length (1-10,000 chars)
- Sanitize user input before display (prevent XSS)
- Validate session ID format (UUID v4 regex)

**Backend**:
- Re-validate all inputs (don't trust frontend)
- Rate limit requests per session (prevent abuse)
- Implement guardrails for off-topic questions
- Sanitize agent responses before returning (though OpenAI SDK handles this)

### Environment Variables

**Never Hardcode**:
- Backend URLs
- API keys
- CORS origins

**Use Environment-Specific Config**:
```javascript
// ❌ BAD
const API_URL = 'http://localhost:8000';

// ✅ GOOD
const API_URL = process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000';
```

---

## Testing Contract Compliance

### Frontend Tests

```typescript
// Test: Request validation
describe('ChatAPI', () => {
  it('should reject empty message', async () => {
    await expect(sendMessage({message: '', session_id: 'valid-uuid', context_text: null}))
      .rejects.toThrow('Message cannot be empty');
  });

  it('should reject invalid session ID', async () => {
    await expect(sendMessage({message: 'test', session_id: 'invalid', context_text: null}))
      .rejects.toThrow('Invalid session ID format');
  });
});
```

### Backend Tests

```python
# Test: Response format
def test_chat_endpoint_returns_citations():
    response = client.post('/agent/chat', json={
        'message': 'What is inverse kinematics?',
        'session_id': str(uuid.uuid4()),
        'context_text': None
    })

    assert response.status_code == 200
    data = response.json()
    assert 'response' in data
    assert 'session_id' in data
    assert 'status' in data
    assert 'Sources:' in data['response']  # Must include citations
```

### Integration Tests

```typescript
// Test: End-to-end flow
it('should complete full conversation flow', async () => {
  const sessionId = crypto.randomUUID();

  // First question
  const response1 = await sendMessage({
    message: 'What is forward kinematics?',
    session_id: sessionId,
    context_text: null,
  });

  expect(response1.status).toBe('success');
  expect(response1.session_id).toBe(sessionId);
  expect(response1.response).toContain('Sources:');

  // Follow-up question (same session)
  const response2 = await sendMessage({
    message: 'How does it differ from inverse kinematics?',
    session_id: sessionId,
    context_text: null,
  });

  expect(response2.status).toBe('success');
  expect(response2.session_id).toBe(sessionId);
  expect(response2.response).toContain('inverse kinematics');
});
```

---

## Appendix: Complete Type Definitions

```typescript
// website/src/types/api.ts

/**
 * Request payload for /agent/chat endpoint
 */
export interface ChatRequest {
  message: string;
  session_id: string;
  context_text: string | null;
}

/**
 * Response payload from /agent/chat endpoint
 */
export interface ChatResponse {
  response: string;
  session_id: string;
  status: 'success' | 'guardrail_triggered' | 'error';
}

/**
 * API error wrapper
 */
export class APIError extends Error {
  constructor(
    public status: number,
    public code: string,
    message: string
  ) {
    super(message);
    this.name = 'APIError';
  }
}

/**
 * API client configuration
 */
export interface APIConfig {
  baseUrl: string;
  timeout: number;
  retryAttempts: number;
}
```
