---
description: "Task list for Personalized Content Page implementation"
---

# Tasks: Personalized Content Page

**Input**: Design documents from `/specs/007-personalized-content/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md (to be created), data-model.md (to be created), contracts/ (to be created)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

This is a **Web application** project:
- Frontend: `website/src/`
- Backend: `backend/` (existing - minimal changes)
- Tests: `website/tests/`, `website/src/__tests__/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for personalization feature

- [ ] T001 Create `website/src/types/personalization.ts` with TypeScript interfaces (Interest, ChapterMetadata, PersonalizedContentView, UserProfile)
- [ ] T002 Create `website/src/config/interests.config.ts` with available interest definitions
- [ ] T003 [P] Create `website/src/components/PersonalizedContent/` directory structure
- [ ] T004 [P] Create test directory structure `website/src/__tests__/personalization/`

**Acceptance**: All directories and base type files exist; types compile without errors

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Research & Documentation

- [ ] T005 Research Docusaurus metadata APIs and document in `specs/007-personalized-content/research.md` (how to extract frontmatter from all docs)
- [ ] T006 Research Better Auth session access patterns in existing pages (`website/src/pages/profile.tsx`) and document in research.md
- [ ] T007 Identify post-login redirect mechanism location and document in research.md

### Core Data & Utilities

- [ ] T008 Create `website/src/utils/interestMapper.ts` with functions: `filterChaptersByInterests()`, `getAvailableInterests()`, `validateInterestIds()`
- [ ] T009 Create `website/src/utils/metadataExtractor.ts` to read chapter metadata from Docusaurus configuration
- [ ] T010 Create `website/src/hooks/useContentMetadata.ts` hook to fetch/cache all chapter metadata
- [ ] T011 Create `website/src/hooks/usePersonalizedContent.ts` hook to orchestrate filtering based on user interests
- [ ] T012 [P] Create `website/src/hooks/useAuth.ts` wrapper (if not exists) to access Better Auth session

### Data Model Documentation

- [ ] T013 Document Interest, ChapterMetadata, PersonalizedContentView structures in `specs/007-personalized-content/data-model.md`
- [ ] T014 Document interest-to-content mapping strategy in data-model.md

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

**Acceptance**:
- All hooks return proper TypeScript types
- `interestMapper.ts` passes basic unit tests (filters correctly)
- `useContentMetadata()` can extract metadata from at least one test chapter
- research.md documents how to access auth session and redirect

---

## Phase 3: User Story 1 - Post-Login Personalized Landing (Priority: P1) 🎯 MVP

**Goal**: Authenticated users with interests are redirected to personalized page showing relevant chapters

**Independent Test**: Sign in with user account that has interests → Redirected to /personalized-content → See filtered chapters matching interests

### Tests for User Story 1 (Write First - Must FAIL Before Implementation)

- [ ] T015 [P] [US1] Create unit test `website/src/__tests__/personalization/interestMapper.test.ts` - test filtering with single interest, multiple interests, no matches
- [ ] T016 [P] [US1] Create component test `website/src/__tests__/personalization/PersonalizedContentView.test.tsx` - test rendering matched chapters, empty state, navigation
- [ ] T017 [US1] Create integration test `website/src/__tests__/personalization/personalized-content-page.test.tsx` - test auth guard, redirect, content display

### Implementation for User Story 1

#### Step 1: Core Components

- [ ] T018 [P] [US1] Create `website/src/components/PersonalizedContent/ContentCard.tsx` - display chapter title, URL link, interest badges, brief description
- [ ] T019 [P] [US1] Create `website/src/components/PersonalizedContent/PersonalizedNav.tsx` - navigation links ("View Full Book", "Back to My Content")
- [ ] T020 [US1] Create `website/src/components/PersonalizedContent/PersonalizedContentView.tsx` - main view component rendering ContentCard grid, chapter count, PersonalizedNav

#### Step 2: Main Page

- [ ] T021 [US1] Create `website/src/pages/personalized-content.tsx` - main page with auth guard, load user interests from session, render PersonalizedContentView
- [ ] T022 [US1] Add auth guard logic in personalized-content.tsx - redirect unauthenticated users to /signin with return URL

#### Step 3: Post-Login Redirect

- [ ] T023 [US1] Modify sign-in success handler in `website/src/pages/signin.tsx` (or identified redirect location from T007) to redirect to `/personalized-content` instead of homepage
- [ ] T024 [US1] Handle return URL parameter in redirect logic (for users redirected from protected page)

#### Step 4: Content Integration

- [ ] T025 [US1] Add frontmatter interest tags to existing chapters in `website/docs/module1/`:
  - `chapter1-introduction.md` → `interests: ["Physical AI", "ROS 2"]`
  - `chapter2-ros2-fundamentals.md` → `interests: ["ROS 2"]`
  - `chapter3-kinematics.md` → `interests: ["Kinematics"]`
  - `chapter4-dynamics-control.md` → `interests: ["Dynamics & Control"]`

**Checkpoint**: User Story 1 complete - users with interests see personalized page after sign-in

**Acceptance Criteria**:
- [ ] User with interests "Kinematics" signs in → Redirected to /personalized-content → Sees Chapter 3 displayed
- [ ] User with interests "ROS 2" signs in → Sees Chapters 1 and 2 displayed
- [ ] User with multiple interests sees all matching chapters in unified view
- [ ] Each chapter card links to correct book URL
- [ ] "View Full Book" button navigates to main docs

---

## Phase 4: User Story 4 - Access Control for Authenticated Users (Priority: P1)

**Goal**: Unauthenticated users attempting to access personalized page are redirected to sign-in

**Independent Test**: Attempt to access /personalized-content without authentication → Redirected to /signin

### Tests for User Story 4

- [ ] T026 [P] [US4] Create E2E test `website/tests/e2e/auth-guard.spec.ts` - test unauthenticated access redirects to signin
- [ ] T027 [P] [US4] Create E2E test for session expiry redirect

### Implementation for User Story 4

- [ ] T028 [US4] Verify auth guard implementation in T022 includes session validation
- [ ] T029 [US4] Test and verify redirect preserves return URL (`/signin?returnTo=/personalized-content`)
- [ ] T030 [US4] Implement sign-out redirect logic - when user signs out on personalized page, redirect to homepage or signin

**Checkpoint**: Access control verified - only authenticated users can view personalized content

**Acceptance Criteria**:
- [ ] Unauthenticated user navigating to `/personalized-content` → Redirected to `/signin`
- [ ] After sign-in, user is redirected back to `/personalized-content`
- [ ] Expired session triggers redirect to signin
- [ ] Sign-out from personalized page redirects to public page

---

## Phase 5: User Story 1 (continued) - Empty Interests Handling

**Goal**: Users with no selected interests see prompt to select interests with option to browse all content

**Independent Test**: Sign in with user account that has NO interests → See interest selection prompt with "Browse All Content" link

### Tests

- [ ] T031 [P] [US1-Empty] Create component test `website/src/__tests__/personalization/InterestPrompt.test.tsx` - test prompt message, navigation links

### Implementation

- [ ] T032 [US1-Empty] Create `website/src/components/PersonalizedContent/InterestPrompt.tsx` - empty state component with message, "Select Interests" link to profile, "Browse All Content" link to docs
- [ ] T033 [US1-Empty] Update `PersonalizedContentView.tsx` to render InterestPrompt when `userInterests.length === 0`
- [ ] T034 [US1-Empty] Update personalized-content.tsx to handle empty interests case and render InterestPrompt

**Checkpoint**: Empty state handled gracefully

**Acceptance Criteria**:
- [ ] User with no interests sees friendly prompt message
- [ ] "Select Interests" button navigates to profile page
- [ ] "Browse All Content" button navigates to full book docs
- [ ] Empty state is visually appealing and helpful

---

## Phase 6: User Story 2 - Dynamic Interest Updates (Priority: P2)

**Goal**: Users update interests in profile and immediately see personalized content page refresh

**Independent Test**: View personalized page with "Kinematics" → Add "Physical AI" to interests in profile → Return to personalized page → See Physical AI chapters added

### Tests for User Story 2

- [ ] T035 [P] [US2] Create integration test `website/src/__tests__/personalization/interest-update.test.tsx` - test content refresh when interests change
- [ ] T036 [P] [US2] Create E2E test `website/tests/e2e/interest-update.spec.ts` - test full flow: change interests → navigate back → see updated content

### Implementation for User Story 2

- [ ] T037 [US2] Implement interest refresh mechanism in `usePersonalizedContent.ts` - detect when user interests change (via session update or profile API)
- [ ] T038 [US2] Add interest change listener/polling in personalized-content.tsx - refetch content when interests update
- [ ] T039 [US2] Ensure profile page interest update triggers session refresh (verify existing implementation or add if missing)
- [ ] T040 [US2] Add visual feedback during content refresh (loading indicator)

**Checkpoint**: Interest updates dynamically refresh personalized content

**Acceptance Criteria**:
- [ ] Adding interest shows new matching chapters without page reload
- [ ] Removing interest removes corresponding chapters
- [ ] Changing all interests completely updates view
- [ ] Updates occur within 3 seconds (SC-005)

---

## Phase 7: User Story 3 - Navigation to Full Book Content (Priority: P2)

**Goal**: Users can navigate between personalized view and full book content seamlessly

**Independent Test**: Click "View Full Book" from personalized page → See full book TOC → Click "Back to My Content" → Return to personalized view

### Tests for User Story 3

- [ ] T041 [P] [US3] Create E2E test `website/tests/e2e/navigation.spec.ts` - test navigation between personalized and full book views
- [ ] T042 [P] [US3] Create integration test for preserving personalized context across navigation

### Implementation for User Story 3

- [ ] T043 [US3] Add "View Full Book" button in `PersonalizedNav.tsx` linking to `/docs` (or main book TOC)
- [ ] T044 [US3] Add "Back to My Content" or "Personalized View" link in main Docusaurus navbar (modify `website/docusaurus.config.ts` or theme)
- [ ] T045 [US3] Ensure personalized context (selected interests) persists across navigation (use session storage if needed)
- [ ] T046 [US3] Test navigation from personalized page → specific chapter → adjacent chapters → back to personalized

**Checkpoint**: Navigation between views is seamless

**Acceptance Criteria**:
- [ ] "View Full Book" navigates to complete book TOC
- [ ] Clicking chapter from personalized page allows navigation to adjacent chapters
- [ ] "Back to My Content" returns to personalized view
- [ ] No loss of personalized context during navigation

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

### Documentation

- [ ] T047 [P] Create developer quickstart guide in `specs/007-personalized-content/quickstart.md` - how to add interest tags to chapters
- [ ] T048 [P] Document interest configuration process in quickstart.md - how to add new interest categories
- [ ] T049 [P] Update main README.md with personalized content feature description

### Code Quality

- [ ] T050 Code cleanup and refactoring - ensure consistent naming, remove dead code
- [ ] T051 Add TypeScript strict mode compliance - fix any `any` types
- [ ] T052 Add ESLint compliance - fix linting warnings in new components

### Performance

- [ ] T053 Optimize content filtering performance - ensure <500ms for 50 chapters (benchmark test)
- [ ] T054 Implement memoization in `usePersonalizedContent` to avoid unnecessary re-filtering
- [ ] T055 Add lazy loading for ContentCard images (if any)

### Accessibility

- [ ] T056 Add ARIA labels to all interactive elements (buttons, links, cards)
- [ ] T057 Ensure keyboard navigation works (tab through cards, navigate to chapters)
- [ ] T058 Test with screen reader (NVDA or JAWS) - verify content is announced correctly
- [ ] T059 Verify color contrast meets WCAG 2.1 AA standards

### Additional Testing

- [ ] T060 [P] Add unit tests for `useContentMetadata.ts` hook
- [ ] T061 [P] Add unit tests for `usePersonalizedContent.ts` hook
- [ ] T062 [P] Add component snapshot tests for all PersonalizedContent components
- [ ] T063 Cross-browser testing (Chrome, Firefox, Safari, Edge)
- [ ] T064 Mobile responsive testing (iOS Safari, Android Chrome)

### Security

- [ ] T065 Security review of auth guard implementation - ensure no bypass possible
- [ ] T066 Verify session handling is secure (HTTP-only cookies, CSRF protection)
- [ ] T067 Test for XSS vulnerabilities in user-generated content (if any)

### Content Preparation

- [ ] T068 Add interest tags to ALL remaining book chapters (beyond module1)
- [ ] T069 Validate interest-to-chapter mapping completeness - ensure all chapters have at least one interest
- [ ] T070 Create content author guide for adding interest tags to new chapters

### Monitoring & Analytics (Optional)

- [ ] T071 [P] Add analytics tracking for personalized page views
- [ ] T072 [P] Add analytics for interest selection/changes
- [ ] T073 [P] Track user engagement metrics (time on personalized page, chapters clicked)

### Final Validation

- [ ] T074 Run through all acceptance scenarios from spec.md manually
- [ ] T075 Validate against all success criteria (SC-001 through SC-009)
- [ ] T076 Perform end-to-end user journey testing with test users
- [ ] T077 Run performance benchmarks (page load <2s, filtering <500ms, redirect <1s)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3)**: Depends on Foundational (Phase 2)
- **User Story 4 (Phase 4)**: Depends on Foundational (Phase 2) - Can run in parallel with US1
- **US1 Empty State (Phase 5)**: Depends on Phase 3 completion
- **User Story 2 (Phase 6)**: Depends on Phase 3 + Phase 5 completion
- **User Story 3 (Phase 7)**: Depends on Phase 3 completion - Can run in parallel with Phase 6
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1) - Core Experience**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 4 (P1) - Access Control**: Can start after Foundational (Phase 2) - Works with US1 but independently testable
- **User Story 1 Empty State**: Depends on US1 core implementation
- **User Story 2 (P2) - Dynamic Updates**: Depends on US1 complete - Adds dynamic refresh capability
- **User Story 3 (P2) - Navigation**: Depends on US1 complete - Adds bidirectional navigation

### Within Each User Story

- **Tests MUST be written FIRST** and FAIL before implementation
- **Models/Types before services/hooks**
- **Hooks before components**
- **Components before page integration**
- **Core implementation before integration**
- **Story complete before moving to next priority**

### Parallel Opportunities

**Phase 1 (Setup)**:
- T001, T002, T003, T004 can all run in parallel

**Phase 2 (Foundational)**:
- T008, T009, T010, T011, T012 can run in parallel after research (T005-T007) completes
- T013, T014 (documentation) can run in parallel with implementation

**Phase 3 (User Story 1)**:
- Tests T015, T016 can run in parallel (must complete before implementation)
- Components T018, T019 can run in parallel
- Content tagging T025 can run in parallel with component development

**Phase 4 (User Story 4)**:
- Tests T026, T027 can run in parallel
- Can work on US4 in parallel with US1 (different focus areas)

**Phase 6 (User Story 2)**:
- Tests T035, T036 can run in parallel

**Phase 7 (User Story 3)**:
- Tests T041, T042 can run in parallel

**Phase 8 (Polish)**:
- Documentation tasks T047, T048, T049 can run in parallel
- Unit tests T060, T061, T062 can run in parallel
- Analytics T071, T072, T073 can run in parallel

---

## Implementation Strategy

### MVP First (User Story 1 + User Story 4 Only)

**Minimum Viable Product - Delivers Core Value**

1. ✅ Complete Phase 1: Setup (T001-T004)
2. ✅ Complete Phase 2: Foundational (T005-T014) - CRITICAL
3. ✅ Complete Phase 3: User Story 1 (T015-T025) - Core personalized experience
4. ✅ Complete Phase 4: User Story 4 (T026-T030) - Access control
5. ✅ Complete Phase 5: US1 Empty State (T031-T034) - Handle users without interests
6. **STOP and VALIDATE**: Test MVP independently with real users
7. Deploy/demo if ready

**MVP Delivers**:
- ✅ Post-login redirect to personalized page
- ✅ Filtered chapters based on user interests
- ✅ Auth guard (only authenticated users)
- ✅ Empty state for users without interests
- ✅ Basic navigation to full book

### Incremental Delivery

1. **Foundation** (Phases 1-2) → Foundation ready for all features
2. **MVP** (Phases 3-5) → Core personalized experience → **DEPLOY** 🚀
3. **Dynamic Updates** (Phase 6) → Add interest refresh → **DEPLOY** 🚀
4. **Enhanced Navigation** (Phase 7) → Bidirectional navigation → **DEPLOY** 🚀
5. **Polish** (Phase 8) → Performance, accessibility, analytics → **DEPLOY** 🚀

Each increment adds value without breaking previous functionality.

### Parallel Team Strategy

With multiple developers:

1. **Together**: Complete Setup + Foundational (Phases 1-2)
2. **Once Foundational is done**:
   - **Developer A**: User Story 1 core (T015-T025)
   - **Developer B**: User Story 4 access control (T026-T030)
   - **Developer C**: User Story 1 empty state (T031-T034) - starts after Developer A's core components
3. **Next Sprint**:
   - **Developer A**: User Story 2 dynamic updates
   - **Developer B**: User Story 3 navigation
   - **Developer C**: Polish & testing
4. Stories complete and integrate independently

---

## Acceptance Checklist (Before Deployment)

### User Story 1: Post-Login Personalized Landing ✅
- [ ] User with interests "Kinematics" signs in → Redirected to /personalized-content → Sees Chapter 3
- [ ] User with interests "ROS 2" signs in → Sees Chapters 1 and 2
- [ ] User with multiple interests sees all matching chapters
- [ ] User with no interests sees prompt to select interests with "Browse All" option

### User Story 2: Dynamic Interest Updates ✅
- [ ] User adds interest → Personalized page updates to show new chapters
- [ ] User removes interest → Corresponding chapters removed
- [ ] User changes all interests → Completely different content displayed
- [ ] Updates occur within 3 seconds

### User Story 3: Navigation to Full Book ✅
- [ ] "View Full Book" navigates to complete book TOC
- [ ] User can navigate from personalized page to chapter to adjacent chapters
- [ ] "Back to My Content" returns to personalized view

### User Story 4: Access Control ✅
- [ ] Unauthenticated user accessing /personalized-content → Redirected to signin
- [ ] After signin, user returned to /personalized-content
- [ ] Expired session triggers redirect to signin

### Success Criteria Validation ✅
- [ ] SC-001: Redirect within 1s of sign-in completion
- [ ] SC-002: Page displays chapters within 2s of page load
- [ ] SC-003: Users with interests see ≥3 relevant chapters
- [ ] SC-004: 90% of users navigate to chapter in <3 clicks
- [ ] SC-005: Interest changes reflect within 3s
- [ ] SC-006: 100% of unauth access blocked
- [ ] SC-007: Navigate between views without losing context
- [ ] SC-008: Zero content duplication (all links to existing pages)
- [ ] SC-009: Interest changes reflect without sign-out/sign-in

### Performance ✅
- [ ] Page load time <2s
- [ ] Content filtering <500ms
- [ ] Redirect after sign-in <1s

### Accessibility ✅
- [ ] WCAG 2.1 AA compliance
- [ ] Keyboard navigation functional
- [ ] Screen reader compatible

### Cross-Browser ✅
- [ ] Chrome (latest)
- [ ] Firefox (latest)
- [ ] Safari (latest)
- [ ] Edge (latest)

### Mobile ✅
- [ ] iOS Safari responsive
- [ ] Android Chrome responsive

---

## Notes

- **[P] tasks** = different files, no dependencies - can run in parallel
- **[Story] label** maps task to specific user story for traceability
- Each user story should be **independently completable and testable**
- **Verify tests fail before implementing** (TDD approach)
- **Commit after each task** or logical group
- **Stop at checkpoints** to validate story independently
- **Avoid**: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Risk Mitigation Tasks

### If Docusaurus metadata API doesn't work as expected:
- [ ] **Fallback T078**: Implement runtime metadata loading from individual page components instead of build-time extraction

### If Better Auth session access is complex:
- [ ] **Fallback T079**: Create session context provider wrapper for easier access across components

### If performance is inadequate:
- [ ] **Fallback T080**: Implement pagination for personalized content (show 10 chapters per page)
- [ ] **Fallback T081**: Add virtual scrolling for large chapter lists

### If interest tagging becomes maintenance burden:
- [ ] **Fallback T082**: Create automated script to suggest interest tags based on chapter content analysis
- [ ] **Fallback T083**: Build simple admin UI for managing interest-to-chapter mappings

---

## Total Tasks: 83 (77 core + 6 fallback)

**Estimated Effort**: 11-16 developer days (as per plan.md)

**Priority Breakdown**:
- **P1 (MVP)**: 34 tasks (Phases 1-5) - ~6-8 days
- **P2 (Enhanced)**: 16 tasks (Phases 6-7) - ~3-4 days
- **Polish**: 27 tasks (Phase 8) - ~2-4 days
- **Fallback**: 6 tasks (only if needed)
