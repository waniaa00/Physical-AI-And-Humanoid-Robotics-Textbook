# Implementation Plan: Personalized Content Page

**Branch**: `007-personalized-content` | **Date**: 2025-12-23 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/007-personalized-content/spec.md`

## Summary

Create a personalized content landing page for authenticated users that filters and displays book chapters and sections based on their selected interests. The system redirects users to this page after sign-in, using metadata-driven mapping (Docusaurus frontmatter tags) to match interests with content. Users without interests receive a prompt to select them, and all users can navigate between personalized and full book views.

**Technical Approach**: Implement as a React/TypeScript page in Docusaurus using Better Auth session context for authentication, read chapter metadata from Docusaurus configuration to build interest-to-content mappings, and render filtered content server-side or at build-time for optimal performance.

## Technical Context

**Language/Version**: TypeScript (matching existing Docusaurus site), React 18
**Primary Dependencies**: Docusaurus 3.x, Better Auth (existing), React Router
**Storage**: User interests stored in existing Neon PostgreSQL via Better Auth; chapter metadata in Docusaurus frontmatter
**Testing**: Jest + React Testing Library (Docusaurus standard), E2E with Playwright
**Target Platform**: Web (Docusaurus static site with React hydration)
**Project Type**: Web (frontend-focused with existing backend auth API)
**Performance Goals**: Page load <2s, content filtering <500ms, redirect after sign-in <1s
**Constraints**: No vector database queries for personalization; no content duplication; preserve existing book URLs; must work with static site generation
**Scale/Scope**: ~20-30 book chapters, 5-10 interest categories, expected 100-1000 concurrent users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

✅ **Simplicity**: Single new page component, leverage existing auth system, metadata-driven (no complex configuration)
✅ **Modularity**: Personalized page is independent feature, doesn't modify existing book pages
✅ **No Over-Engineering**: Uses standard Docusaurus patterns, no new frameworks or abstractions
✅ **Testability**: Clear acceptance criteria, mockable auth context, deterministic content filtering
✅ **Performance**: Static metadata extraction at build time, client-side filtering is O(n) over small dataset

**No constitution violations detected** - proceeding with implementation.

## Project Structure

### Documentation (this feature)

```text
specs/007-personalized-content/
├── plan.md              # This file
├── research.md          # Phase 0: Investigation of Docusaurus metadata APIs
├── data-model.md        # Phase 1: Interest-content mapping structure
├── quickstart.md        # Phase 1: Developer guide for adding interest tags
├── contracts/           # Phase 1: TypeScript interfaces
│   ├── interest-types.ts
│   ├── content-metadata.ts
│   └── personalized-view.ts
└── tasks.md             # Phase 2: Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

```text
website/
├── src/
│   ├── pages/
│   │   ├── personalized-content.tsx         # NEW: Main personalized page
│   │   └── ...                               # Existing pages
│   ├── components/
│   │   ├── PersonalizedContent/              # NEW: Feature components
│   │   │   ├── PersonalizedContentView.tsx  # Main view component
│   │   │   ├── ContentCard.tsx              # Chapter/section card
│   │   │   ├── InterestPrompt.tsx           # Empty state for no interests
│   │   │   └── PersonalizedNav.tsx          # Navigation component
│   │   ├── Root/                             # MODIFY: Add post-login redirect
│   │   │   └── index.tsx                    # Update redirect logic
│   │   └── ...                               # Existing components
│   ├── hooks/
│   │   ├── usePersonalizedContent.ts         # NEW: Content filtering hook
│   │   ├── useContentMetadata.ts             # NEW: Metadata extraction hook
│   │   └── ...                               # Existing hooks
│   ├── utils/
│   │   ├── interestMapper.ts                 # NEW: Interest-to-content mapping
│   │   └── ...                               # Existing utils
│   ├── config/
│   │   └── interests.config.ts               # NEW: Interest definitions
│   └── types/
│       └── personalization.ts                # NEW: TypeScript types
│
├── docs/                                      # MODIFY: Add frontmatter tags
│   ├── module1/
│   │   ├── chapter1-introduction.md          # Add: interests: ["Physical AI", "ROS 2"]
│   │   ├── chapter2-ros2-fundamentals.md     # Add: interests: ["ROS 2"]
│   │   ├── chapter3-kinematics.md            # Add: interests: ["Kinematics"]
│   │   └── chapter4-dynamics-control.md      # Add: interests: ["Dynamics & Control"]
│   └── ...
│
└── tests/
    ├── personalized-content.test.tsx          # NEW: Page tests
    ├── usePersonalizedContent.test.ts         # NEW: Hook tests
    └── interestMapper.test.ts                 # NEW: Mapping logic tests

backend/
└── (No changes - uses existing Better Auth endpoints)
```

**Structure Decision**: Web application structure (Option 2) - frontend-focused feature using existing backend. The personalized content page is entirely frontend code within the Docusaurus site, leveraging existing Better Auth session management via HTTP-only cookies.

## Complexity Tracking

> **No constitution violations** - this section is empty.

---

## Phase 0: Research & Technical Discovery

**Objective**: Understand Docusaurus metadata APIs, Better Auth session access, and build-time vs runtime trade-offs for content filtering.

### Research Tasks

#### R1: Docusaurus Metadata System
- **Question**: How to extract frontmatter metadata from all documentation pages at build time?
- **Investigation**:
  - Research `@docusaurus/plugin-content-docs` API
  - Find documentation metadata access patterns in Docusaurus config
  - Determine if metadata is available via `useDocsSidebar()` hook or requires custom plugin
- **Deliverable**: Document approach for reading all chapter metadata (frontmatter tags)

#### R2: Better Auth Session in Docusaurus Pages
- **Question**: How to access authenticated user context and interests in a Docusaurus page?
- **Investigation**:
  - Review existing `website/src/pages/profile.tsx` for auth context usage
  - Identify session API calls or context providers
  - Determine if interests are already stored in user profile
- **Deliverable**: Code pattern for accessing user interests from auth session

#### R3: Post-Login Redirect Mechanism
- **Question**: Where and how to implement redirect to /personalized-content after sign-in?
- **Investigation**:
  - Review `website/src/pages/signin.tsx` sign-in flow
  - Check if `Root/index.tsx` or auth callback handles post-login navigation
  - Identify redirect configuration point
- **Deliverable**: Exact file and function to modify for post-login redirect

#### R4: Static vs Dynamic Content Filtering
- **Question**: Should interest-to-content mapping happen at build time or runtime?
- **Investigation**:
  - Analyze trade-offs: build-time (faster, less flexible) vs runtime (slower, more flexible)
  - Assess if Docusaurus SSG allows build-time metadata aggregation
  - Consider that interests change dynamically (requires runtime filtering)
- **Deliverable**: Decision on filtering strategy with justification

#### R5: Chapter URL Pattern Analysis
- **Question**: What is the URL structure for book chapters to generate correct links?
- **Investigation**:
  - Document existing chapter URL patterns (e.g., `/docs/module1/chapter3-kinematics`)
  - Verify if URLs are predictable from frontmatter or require sidebar config
- **Deliverable**: URL generation formula from chapter metadata

**Research Outputs**: `specs/007-personalized-content/research.md`

---

## Phase 1: Design & Contracts

**Objective**: Define data models, TypeScript interfaces, interest-content mapping structure, and API contracts.

### 1.1 Data Model

**Document**: `specs/007-personalized-content/data-model.md`

#### Interest Definition Model
```typescript
interface Interest {
  id: string;              // e.g., "kinematics", "ros2"
  label: string;           // e.g., "Kinematics", "ROS 2 Fundamentals"
  description: string;     // e.g., "Forward/inverse kinematics, Jacobians"
  icon?: string;           // Optional emoji or icon identifier
}
```

#### Chapter Metadata Model
```typescript
interface ChapterMetadata {
  id: string;              // e.g., "chapter3-kinematics"
  title: string;           // e.g., "Chapter 3: Kinematics"
  url: string;             // e.g., "/docs/module1/chapter3-kinematics"
  interests: string[];     // e.g., ["kinematics", "robotics-fundamentals"]
  module?: string;         // e.g., "module1"
  order?: number;          // Chapter sequence number
}
```

#### Personalized Content View Model
```typescript
interface PersonalizedContentView {
  userInterests: string[];           // User's selected interest IDs
  matchedChapters: ChapterMetadata[]; // Chapters matching interests
  emptyState: boolean;                // True if no interests or no matches
  totalChapters: number;              // Total available chapters
  matchCount: number;                 // Number of matched chapters
}
```

#### User Profile Extension
```typescript
// Extends existing Better Auth user profile
interface UserProfile {
  id: string;
  email: string;
  interests: string[];  // Array of interest IDs (may already exist)
  // ... other existing profile fields
}
```

### 1.2 Interest-Content Mapping Strategy

**Approach**: Metadata-driven using Docusaurus frontmatter

#### Frontmatter Example
```markdown
---
title: "Chapter 3: Kinematics"
sidebar_position: 3
interests: ["kinematics", "robotics-fundamentals"]
---

# Kinematics

[Chapter content...]
```

#### Mapping Logic
1. **Build Time**: Extract all chapter metadata into a static JSON structure
2. **Runtime**: Filter chapters based on user's selected interests
3. **Matching Rule**: Include chapter if ANY of its interest tags match ANY user interest

**Rationale**: Simple, maintainable, allows content authors to control categorization without code changes.

### 1.3 API/Service Contracts

**Document**: `specs/007-personalized-content/contracts/`

#### `usePersonalizedContent` Hook Contract
```typescript
// File: hooks/usePersonalizedContent.ts
interface UsePersonalizedContentResult {
  content: PersonalizedContentView | null;
  isLoading: boolean;
  error: Error | null;
  refetch: () => void;
}

function usePersonalizedContent(
  userInterests: string[]
): UsePersonalizedContentResult;
```

#### `useContentMetadata` Hook Contract
```typescript
// File: hooks/useContentMetadata.ts
interface UseContentMetadataResult {
  chapters: ChapterMetadata[];
  isLoading: boolean;
  error: Error | null;
}

function useContentMetadata(): UseContentMetadataResult;
```

#### Interest Mapper Utility Contract
```typescript
// File: utils/interestMapper.ts
function filterChaptersByInterests(
  chapters: ChapterMetadata[],
  userInterests: string[]
): ChapterMetadata[];

function getAvailableInterests(): Interest[];

function validateInterestIds(interestIds: string[]): boolean;
```

### 1.4 Component Architecture

#### Page Component
- **personalized-content.tsx**: Main page, auth guard, renders PersonalizedContentView or InterestPrompt

#### Feature Components
- **PersonalizedContentView**: Displays filtered chapters in card grid
- **ContentCard**: Individual chapter card (title, description, link, interests badges)
- **InterestPrompt**: Empty state UI for users without interests
- **PersonalizedNav**: Navigation between personalized view and full book

#### Hooks
- **usePersonalizedContent**: Orchestrates content filtering based on user interests
- **useContentMetadata**: Fetches/caches all chapter metadata
- **useAuth** (existing): Accesses user session and interests

### 1.5 Quickstart Guide

**Document**: `specs/007-personalized-content/quickstart.md`

**Content**:
- How to add interest tags to new chapters
- How to add new interest categories to `interests.config.ts`
- How to test personalized content locally
- How to verify interest-to-content mapping

---

## Phase 2: Implementation Tasks

**Objective**: Break down implementation into discrete, testable tasks.

**Note**: This phase is executed via `/sp.tasks` command, which generates `specs/007-personalized-content/tasks.md`.

**Expected Task Categories** (preview):

1. **Setup & Configuration**
   - Create interest configuration file
   - Define TypeScript types and interfaces
   - Set up test infrastructure

2. **Metadata Extraction**
   - Implement chapter metadata extraction from Docusaurus
   - Create metadata caching/aggregation mechanism
   - Add frontmatter tags to existing chapters

3. **Core Logic**
   - Implement interest-to-content mapping utility
   - Create content filtering hook
   - Build personalized view data transformation

4. **UI Components**
   - Build PersonalizedContentView component
   - Create ContentCard component
   - Implement InterestPrompt empty state
   - Build PersonalizedNav component

5. **Page Integration**
   - Create /personalized-content page
   - Add auth guard and redirect logic
   - Implement post-login redirect mechanism

6. **Navigation & User Flow**
   - Add navigation between personalized and full book views
   - Handle interest updates and content refresh

7. **Testing**
   - Unit tests for filtering logic
   - Component tests for UI
   - Integration tests for full user flow
   - E2E tests for auth + redirect + personalization

8. **Documentation & Deployment**
   - Update developer documentation
   - Add interest tagging guide for content authors

---

## Phase 3: Testing Strategy

### Unit Tests

- **Interest Mapper** (`interestMapper.test.ts`)
  - Filter chapters with single interest
  - Filter chapters with multiple interests
  - Handle no matching content
  - Handle no user interests
  - Validate interest IDs

- **Hooks** (`usePersonalizedContent.test.ts`, `useContentMetadata.test.ts`)
  - Mock data loading and transformation
  - Test error handling
  - Test refetch mechanism

### Component Tests

- **PersonalizedContentView** (`PersonalizedContentView.test.tsx`)
  - Render matched chapters
  - Display correct chapter count
  - Handle empty state
  - Render navigation links

- **InterestPrompt** (`InterestPrompt.test.tsx`)
  - Display prompt message
  - Show browse all content link
  - Render correctly when no interests selected

### Integration Tests

- **Full Page Flow** (`personalized-content.test.tsx`)
  - Authenticated user with interests sees filtered content
  - Authenticated user without interests sees prompt
  - Unauthenticated user redirected to sign-in
  - Navigate to full book and back

### E2E Tests (Playwright)

- **User Journey 1**: Sign in → Redirect to personalized page → See filtered chapters
- **User Journey 2**: Update interests in profile → Return to personalized page → See updated content
- **User Journey 3**: Navigate from personalized page to full book → Navigate back
- **User Journey 4**: Attempt unauthenticated access → Redirect to sign-in → Sign in → Return to personalized page

---

## Phase 4: Deployment & Rollout

### Pre-Deployment Checklist

- [ ] All unit tests passing
- [ ] All integration tests passing
- [ ] E2E tests passing in staging environment
- [ ] Performance benchmarks met (page load <2s, filtering <500ms)
- [ ] Accessibility audit passed (WCAG 2.1 AA)
- [ ] Cross-browser testing complete (Chrome, Firefox, Safari, Edge)
- [ ] Mobile responsive testing complete
- [ ] Security review passed (auth guard, session handling)

### Deployment Steps

1. **Metadata Preparation**
   - Add interest tags to all existing book chapters
   - Validate interest-to-content mappings
   - Ensure all chapters have correct frontmatter

2. **Feature Flag** (Optional)
   - If available, deploy behind feature flag
   - Enable for internal testing first
   - Gradual rollout to users

3. **Database Migration** (if needed)
   - Verify user interests field exists in database
   - Seed default interests if users don't have any

4. **Deployment**
   - Deploy backend changes (if any)
   - Build and deploy frontend (Docusaurus site)
   - Verify post-login redirect works

5. **Monitoring**
   - Track page load times
   - Monitor redirect success rate
   - Track user engagement (time on personalized page vs full book)
   - Watch for errors in personalization logic

### Rollback Plan

- **Trigger**: Critical bugs, performance degradation, or auth issues
- **Steps**:
  1. Revert post-login redirect to previous behavior (redirect to homepage)
  2. Keep /personalized-content page accessible but not default
  3. Fix issues in development branch
  4. Re-test and re-deploy

---

## Risk Analysis

### Technical Risks

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Docusaurus metadata API changes | High | Low | Use stable plugin APIs; version lock dependencies |
| Build-time metadata extraction fails | High | Medium | Add fallback to runtime metadata loading |
| Performance degradation with many chapters | Medium | Low | Implement pagination or lazy loading if >100 chapters |
| Auth session not accessible in Docusaurus page | High | Low | Research shows existing pages access auth; follow same pattern |

### User Experience Risks

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Users confused by personalized vs full book | Medium | Medium | Clear navigation, "View Full Book" button prominent |
| Users frustrated by empty personalized page | Medium | High | InterestPrompt guides users to select interests or browse all |
| Users don't understand interest tags | Low | Medium | Add interest descriptions and examples |

### Data Risks

| Risk | Impact | Likelihood | Mitigation |
|------|--------|------------|------------|
| Interest-content mapping becomes outdated | Low | Medium | Document interest tagging process for content authors |
| Users have invalid interest IDs | Low | Low | Validate interest IDs; gracefully handle unknown interests |

---

## Success Metrics

### Technical Metrics
- Page load time <2s (SC-002)
- Content filtering <500ms
- Redirect after sign-in <1s (SC-001)
- Zero errors in production for first week

### User Engagement Metrics
- 90% of users with interests see ≥3 chapters (SC-003)
- 90% navigate to chapter in <3 clicks (SC-004)
- Interest update reflects in <3s (SC-005)
- 100% of unauth access blocked (SC-006)

### Adoption Metrics
- % of users who select interests (target: 70% within first month)
- % of users who use personalized page vs direct book access
- Time spent on personalized page vs homepage

---

## Open Questions

1. **Interest Seeding**: Should we provide default/suggested interests for new users during onboarding?
   - **Decision Needed By**: Before deployment
   - **Owner**: Product/UX team

2. **Interest Priority**: Should some interests take precedence over others in content ordering?
   - **Current Plan**: Display in book order (simpler)
   - **Alternative**: Allow users to rank interests (more complex)

3. **Content Granularity**: Should interests map to sections within chapters or only whole chapters?
   - **Current Plan**: Chapter-level only (simpler)
   - **Future Enhancement**: Section-level mapping if demand exists

---

## Dependencies

### External Dependencies
- **Better Auth**: User authentication and profile storage (existing)
- **Docusaurus**: Static site generator and plugin system (existing)
- **Neon PostgreSQL**: User data storage (existing)

### Internal Dependencies
- **Interest Selection Feature**: Users must have UI to select interests (assumed exists per spec assumptions)
- **Book Content Structure**: Stable chapter URLs (exists)
- **Session Management**: Auth state across navigation (exists)

### Blocked By
- None - all dependencies are satisfied or will be created as part of this feature

---

## Timeline Estimate

**Note**: This is a complexity estimate, not a deadline. Actual calendar time depends on team capacity.

- **Phase 0 (Research)**: 1-2 days
- **Phase 1 (Design)**: 2-3 days
- **Phase 2 (Implementation)**: 5-7 days
  - Setup: 1 day
  - Core Logic: 2 days
  - UI Components: 2 days
  - Integration: 1 day
  - Testing: 1-2 days
- **Phase 3 (Testing & QA)**: 2-3 days
- **Phase 4 (Deployment)**: 1 day

**Total Estimated Effort**: 11-16 days (developer days, not calendar days)

---

## Notes

- This plan assumes the Interest Selection feature (UI for users to choose interests) already exists. If not, that must be built first.
- The plan prioritizes simplicity: metadata-driven mapping, no complex algorithms, standard Docusaurus patterns.
- Performance should not be an issue given small dataset (~20-30 chapters) and client-side filtering.
- Future enhancements could include: section-level mapping, interest ranking, AI-based recommendations, reading progress tracking.
