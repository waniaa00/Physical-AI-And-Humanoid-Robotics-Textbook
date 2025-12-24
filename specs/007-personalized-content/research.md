# Research Documentation: Personalized Content Feature

## Overview
This document captures research findings for T005-T007: Docusaurus metadata APIs, Better Auth session access patterns, and post-login redirect mechanisms.

---

## T005: Docusaurus Metadata APIs

### Current State
- **Docusaurus Version**: 3.x (with future.v4 flag enabled in config)
- **Docs Plugin**: `@docusaurus/preset-classic` with docs preset
- **Frontmatter Structure**: Each chapter uses YAML frontmatter

### Existing Frontmatter Schema
```yaml
---
sidebar_position: 1
title: "Chapter 1: Introduction to Physical AI"
---
```

**Current Fields**:
- `sidebar_position` (number): Order in sidebar
- `title` (string): Chapter display title

**Missing Field** (to be added in T025):
- `interests` (string[]): Array of interest IDs (e.g., `['physical-ai', 'ros2']`)

### Docusaurus API Hooks Available

#### 1. `useDocusaurusContext()`
**Location**: `@docusaurus/useDocusaurusContext`
**Usage**: Access site configuration
```typescript
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function Component() {
  const {siteConfig} = useDocusaurusContext();
  // Access: siteConfig.title, siteConfig.customFields, etc.
}
```

**Example in codebase**: `website/src/pages/index.tsx:4,12`

#### 2. `useAllDocsData()` (Future Integration)
**Location**: `@docusaurus/plugin-content-docs/client`
**Purpose**: Access all docs metadata at runtime
**Status**: NOT YET IMPLEMENTED

**Future Implementation** (from T009 TODO comment):
```typescript
import { useAllDocsData } from '@docusaurus/plugin-content-docs/client';

export function extractChapterMetadataFromDocusaurus(): ChapterMetadata[] {
  const docsData = useAllDocsData();
  // Extract and transform metadata from Docusaurus...
  // Parse frontmatter, build ChapterMetadata objects
}
```

### Recommended Approach for MVP
**Current Strategy** (T009): Use static metadata array in `metadataExtractor.ts`
- ✅ Simpler implementation for MVP
- ✅ No dependencies on Docusaurus plugin APIs
- ✅ Easy to test and validate
- ⚠️ Requires manual sync with actual chapter files

**Future Strategy** (Post-MVP):
- Replace static metadata with `useAllDocsData()` hook
- Extract frontmatter dynamically at runtime
- Eliminate manual sync requirement

### File Locations
- **Config**: `website/docusaurus.config.ts`
- **Chapters**: `website/docs/module1/*.md`, `website/docs/module2/*.md`, etc.
- **Metadata Extractor**: `website/src/utils/metadataExtractor.ts`

---

## T006: Better Auth Session Access Patterns

### Better Auth Integration Status
**Current Implementation**: Custom authentication context (NOT using Better Auth library yet)
**Location**: `website/src/contexts/AuthContext.tsx`

### AuthContext API

#### Interface
```typescript
interface AuthContextType {
  user: User | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  signUp: (data: SignUpData) => Promise<void>;
  signIn: (data: SignInData) => Promise<void>;
  signOut: () => Promise<void>;
  validateSession: () => Promise<boolean>;
}

interface User {
  user_id: string;
  email: string;
  interests: number[]; // Backend interest IDs
}
```

#### Hook Usage
```typescript
import { useAuth } from '@site/src/hooks/useAuth';

function Component() {
  const { user, isAuthenticated, isLoading, signIn, signOut } = useAuth();

  if (isLoading) return <div>Loading...</div>;
  if (!isAuthenticated) return <div>Please sign in</div>;

  return <div>Welcome, {user.email}</div>;
}
```

**Example in codebase**: `website/src/pages/signin.tsx:5,11`

### Session Management

#### Session Storage
- **Method**: HTTP-only cookies (`session_token`)
- **Library**: `js-cookie` (client-side cookie access)
- **Validation**: Backend validates session on each request

#### Session Lifecycle
1. **Sign In** → Backend creates session → Cookie set → `user` state updated
2. **Page Load** → `validateSession()` called automatically → Session validated
3. **Sign Out** → Backend destroys session → Cookie removed → `user` state cleared

#### Backend Endpoints
- `POST http://localhost:8000/auth/sign-up`
- `POST http://localhost:8000/auth/sign-in`
- `POST http://localhost:8000/auth/sign-out`
- `GET http://localhost:8000/auth/session/validate`

### Accessing User Interests

#### Current Pattern (Profile Page)
```typescript
// website/src/pages/profile.tsx:30-35
const currentUserId = getCurrentUserId(); // From localStorage
const response = await fetch(`http://localhost:8000/interests/${currentUserId}`);
const data = await response.json();
// data.interests = [1, 2, 3] (backend interest IDs)
```

#### Recommended Pattern (Personalized Content)
```typescript
import { useAuth } from '@site/src/hooks/useAuth';

function PersonalizedContent() {
  const { user, isAuthenticated } = useAuth();

  // user.interests contains backend interest IDs [1, 2, 3]
  // Map to frontend interest strings ['physical-ai', 'ros2', ...]
  const frontendInterests = mapBackendToFrontendInterests(user?.interests || []);

  const { content } = usePersonalizedContent(frontendInterests);
}
```

### Interest ID Mapping Strategy
- **Backend**: Uses numeric IDs (1, 2, 3, ...)
- **Frontend**: Uses string IDs ('physical-ai', 'ros2', 'kinematics', ...)
- **Mapping Required**: T013 will define bidirectional mapping

### Session Initialization
**Automatic Validation** (T056): `AuthContext` validates session on app load
```typescript
// website/src/contexts/AuthContext.tsx:54-67
useEffect(() => {
  const initAuth = async () => {
    await validateSession();
  };
  initAuth();
}, []);
```

---

## T007: Post-Login Redirect Mechanism

### Current Redirect Pattern

#### Sign In Page (T043)
**Location**: `website/src/pages/signin.tsx:41`
```typescript
await signIn({ email, password });
window.location.href = '/docs/intro'; // Current redirect target
```

#### Sign Up Page (Similar Pattern)
**Location**: `website/src/pages/signup.tsx` (similar pattern)
```typescript
await signUp(data);
window.location.href = '/docs/intro'; // After successful sign-up
```

### Proposed Redirect Logic for Personalized Content

#### Decision Flow
```
1. User signs in successfully
2. Check: Does user have interests?
   - YES → Redirect to /personalized-content
   - NO  → Redirect to /personalized-content (shows InterestPrompt)
3. User can browse all content from personalized page
```

#### Implementation Location
**File to Modify**: `website/src/pages/signin.tsx:41`

**Before** (Current):
```typescript
await signIn({ email, password });
window.location.href = '/docs/intro';
```

**After** (T023-T024):
```typescript
await signIn({ email, password });

// FR-001: Redirect to personalized content page after sign-in
window.location.href = '/personalized-content';
```

#### Alternative: Client-Side Navigation (Docusaurus Router)
**Using Docusaurus Link** (preferred for SPA behavior):
```typescript
import { useHistory } from '@docusaurus/router';

const history = useHistory();
await signIn({ email, password });
history.push('/personalized-content');
```

**Trade-offs**:
- `window.location.href`: Full page reload (simpler, guarantees fresh state)
- `history.push()`: SPA navigation (faster, no reload)

**Recommendation**: Use `window.location.href` for MVP to ensure clean state after authentication

### Redirect Targets Summary
| Current Behavior | New Behavior (FR-001) |
|------------------|----------------------|
| `/signin` → `/docs/intro` | `/signin` → `/personalized-content` |
| `/signup` → `/docs/intro` | `/signup` → `/personalized-content` |

### Session Persistence
- Redirect works because session is cookie-based (persists across page loads)
- AuthContext re-validates session on mount of personalized page
- No additional session handling needed

---

## T012: useAuth Wrapper Assessment

### Existing Hook
**Location**: `website/src/hooks/useAuth.ts`
```typescript
export { useAuthContext as useAuth } from '../contexts/AuthContext';
```

### Decision: NO NEW WRAPPER NEEDED
✅ **Existing hook is sufficient**
- Already exports `useAuth` from AuthContext
- Provides all necessary authentication state and methods
- No additional wrapper required for personalized content feature

### Usage in Personalized Content
```typescript
import { useAuth } from '@site/src/hooks/useAuth';

export default function PersonalizedContentPage() {
  const { user, isAuthenticated, isLoading } = useAuth();

  // Use user.interests for personalization
  // Use isAuthenticated for access control
}
```

**Task T012 can be marked as SKIPPED** (no action needed)

---

## Summary

### Key Decisions
1. **Metadata Strategy**: Use static metadata for MVP, migrate to `useAllDocsData()` post-MVP
2. **Session Access**: Use existing `useAuth()` hook from AuthContext
3. **Post-Login Redirect**: Change target from `/docs/intro` to `/personalized-content` using `window.location.href`
4. **Interest Mapping**: Backend numeric IDs → Frontend string IDs (documented in data-model.md)
5. **useAuth Wrapper**: NOT NEEDED, existing hook is sufficient

### Implementation Checklist
- [x] Document Docusaurus metadata APIs (T005)
- [x] Document Better Auth session access patterns (T006)
- [x] Document post-login redirect mechanism (T007)
- [x] Assess useAuth wrapper need (T012) → SKIP

### Next Steps
1. Complete data model documentation (T013-T014)
2. Begin Phase 3: UI component implementation (T015-T025)
3. Implement post-login redirect change (T023-T024)
4. Add frontmatter tags to chapter files (T025)
