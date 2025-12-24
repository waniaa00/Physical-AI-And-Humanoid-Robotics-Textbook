# Tasks: Better Auth Integration with Interest-Based Personalization

**Input**: Design documents from `/specs/006-better-auth-integration/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests are OPTIONAL per specification requirements - only included where explicitly needed for security validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US4)
- Include exact file paths in descriptions

## Path Conventions

This project uses **web app structure**: `backend/` and `frontend/website/src/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and dependency installation

- [x] T001 Install Better Auth dependencies in backend/requirements.txt (better-auth-python or fastapi-users, bcrypt==4.1.1, python-jose[cryptography]==3.3.0)
- [x] T002 Install Better Auth React SDK in website/package.json (@better-auth/react)
- [x] T003 [P] Create backend/.env.example with JWT_SECRET_KEY, DATABASE_URL, CORS_ORIGINS
- [x] T004 [P] Update backend/main.py to import CORS middleware configuration

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core authentication infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T005 Create User model in backend/models/auth.py (id, email, password_hash, created_at, updated_at, last_sign_in_at, account_status)
- [x] T006 Create Session model in backend/models/auth.py (id, user_id, token_hash, expires_at, created_at, ip_address, user_agent)
- [x] T007 Create AuthToken Pydantic model in backend/models/auth.py (access_token, token_type, expires_in)
- [x] T008 [P] Create UserRepository in backend/repositories/user_repository.py (create_user, get_user_by_email, get_user_by_id, update_last_sign_in)
- [x] T009 [P] Implement bcrypt password hashing in backend/utils/password_hasher.py (hash_password with cost 12, verify_password)
- [x] T010 [P] Implement JWT token manager in backend/utils/token_manager.py (create_access_token with 7-day expiry, verify_token, decode_token)
- [x] T011 Initialize Better Auth service in backend/services/auth_service.py (configure Neon Postgres adapter)
- [x] T012 Create authentication middleware in backend/middleware/auth_middleware.py (get_optional_user, get_required_user dependencies)
- [x] T013 Update CORS middleware in backend/middleware/cors_middleware.py (allow credentials, allowed origins from env)
- [x] T014 Create database migration for users and sessions tables in database/migrations/006_create_auth_tables.sql
- [x] T015 Apply database migrations to Neon Postgres (create users, sessions, ensure user_interests table exists from Phase 5)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - New User Registration with Interest Selection (Priority: P1) 🎯 MVP

**Goal**: Enable new users to create accounts with email/password and select 2-5 interests for personalized chatbot responses

**Independent Test**: Create a new account at /signup, select 3 interests, verify account created in database, sign in successfully, and chatbot responses include interest-based analogies

### Implementation for User Story 1

- [x] T016 [P] [US1] Create SignUpRequest Pydantic model in backend/models/auth.py (email, password, interests: List[int], background, language_preference)
- [x] T017 [P] [US1] Create SignUpResponse Pydantic model in backend/models/auth.py (user_id, session_token, message)
- [x] T018 [US1] Implement sign_up method in backend/services/auth_service.py (validate email format, check duplicate, hash password, create user, create session, save interests)
- [x] T019 [US1] Create POST /auth/sign-up endpoint in backend/main.py (call auth_service.sign_up, set HttpOnly cookie, return 201)
- [x] T020 [US1] Add email validation (RFC 5322 format) in backend/services/auth_service.py
- [x] T021 [US1] Add password strength validation in backend/services/auth_service.py (min 8 chars, 1 uppercase, 1 lowercase, 1 number)
- [x] T022 [US1] Add interest count validation (2-5 interests) in backend/services/auth_service.py
- [x] T023 [US1] Implement interest association in backend/repositories/user_repository.py (save_user_interests to user_interests junction table)
- [x] T024 [US1] Add error handling for duplicate email (return 400 with "Email already registered")
- [x] T025 [US1] Add audit logging for sign-up events in backend/services/auth_service.py (log user_id, email, timestamp, success/failure)
- [x] T026 [P] [US1] Create AuthContext.tsx in website/src/contexts/AuthContext.tsx (manage user state, isAuthenticated, signUp, signIn, signOut methods)
- [x] T027 [P] [US1] Create useAuth custom hook in website/src/hooks/useAuth.ts (wraps AuthContext with Better Auth React SDK)
- [x] T028 [P] [US1] Create Better Auth client initialization in website/src/lib/auth.ts (configure API base URL, cookie settings)
- [x] T029 [US1] Update signup.tsx in website/src/pages/signup.tsx to integrate Better Auth (call useAuth().signUp, handle interests selection, display validation errors)
- [x] T030 [US1] Add interest selection UI component in website/src/pages/signup.tsx (checkboxes for 8 categories, enforce 2-5 constraint)
- [x] T031 [US1] Add client-side validation in website/src/pages/signup.tsx (email format, password strength, interest count 2-5)
- [x] T032 [US1] Implement redirect to chat page after successful sign-up in website/src/pages/signup.tsx

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently. New users can create accounts with interests and access personalized chatbot.

---

## Phase 4: User Story 2 - Returning User Authentication (Priority: P1)

**Goal**: Enable registered users to sign in with email/password and restore their authenticated session with saved interests

**Independent Test**: Sign in with valid credentials at /signin, verify session cookie set, access protected features, close browser, reopen within 7 days, verify still authenticated

### Implementation for User Story 2

- [x] T033 [P] [US2] Create SignInRequest Pydantic model in backend/models/auth.py (email, password)
- [x] T034 [P] [US2] Create SignInResponse Pydantic model in backend/models/auth.py (user_id, session_token, interests, message)
- [x] T035 [US2] Implement sign_in method in backend/services/auth_service.py (find user by email, verify password with bcrypt, create session, return JWT)
- [x] T036 [US2] Create POST /auth/sign-in endpoint in backend/main.py (call auth_service.sign_in, set HttpOnly cookie, return 200)
- [x] T037 [US2] Add generic error handling for invalid credentials in backend/services/auth_service.py (return "Invalid credentials" without revealing email existence)
- [x] T038 [US2] Update last_sign_in_at timestamp in backend/repositories/user_repository.py on successful sign-in
- [x] T039 [US2] Retrieve user interests in sign_in method (join user_interests table) and include in response
- [x] T040 [US2] Add audit logging for sign-in events in backend/services/auth_service.py (log user_id, email, timestamp, ip_address, success/failure)
- [x] T041 [US2] Update signin.tsx in website/src/pages/signin.tsx to integrate Better Auth (call useAuth().signIn, handle errors)
- [x] T042 [US2] Add error message display for invalid credentials in website/src/pages/signin.tsx
- [x] T043 [US2] Implement redirect to previous page or chat after successful sign-in in website/src/pages/signin.tsx
- [x] T044 [US2] Add session persistence in AuthContext.tsx (check for existing session on app load, validate with backend)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Users can create accounts and sign in.

---

## Phase 5: User Story 4 - Secure Session Management (Priority: P1)

**Goal**: Ensure authentication sessions are secure with proper logout, expiration handling, and token security

**Independent Test**: Sign in, verify HttpOnly cookie flags, attempt token tampering (should fail), sign out (cookie cleared), wait 7 days (session expired prompt)

### Implementation for User Story 4

- [x] T045 [P] [US4] Implement sign_out method in backend/services/auth_service.py (invalidate session in database, clear token_hash)
- [x] T046 [P] [US4] Create POST /auth/sign-out endpoint in backend/main.py (call auth_service.sign_out, clear cookie, return 200)
- [x] T047 [P] [US4] Implement session validation in backend/services/auth_service.py (verify JWT signature, check expiration, check session exists in DB)
- [x] T048 [P] [US4] Create GET /auth/session/validate endpoint in backend/main.py (return session status, user_id, expires_at)
- [x] T049 [US4] Configure JWT cookie security flags in backend/main.py (HttpOnly=True, Secure=True in production, SameSite=Lax)
- [x] T050 [US4] Implement automatic session expiration in backend/utils/token_manager.py (7-day TTL in JWT claims)
- [x] T051 [US4] Add session cleanup cron job or database trigger to delete expired sessions from sessions table
- [x] T052 [US4] Implement invalidate_all_sessions method in backend/repositories/user_repository.py (called on password change)
- [x] T053 [US4] Add token tampering detection in backend/middleware/auth_middleware.py (verify signature, return 401 if invalid)
- [x] T054 [US4] Implement signOut method in website/src/hooks/useAuth.ts (call /auth/sign-out, clear AuthContext state, redirect to sign-in)
- [x] T055 [US4] Add session expiration handling in AuthContext.tsx (detect 401 responses, show expiration message, redirect to sign-in)
- [x] T056 [US4] Add automatic session validation on app load in AuthContext.tsx (call /auth/session/validate, restore user state if valid)

**Checkpoint**: At this point, User Stories 1, 2, and 4 are complete. Authentication is fully secure with proper session lifecycle management.

---

## Phase 6: Protected Endpoints & Agent Personalization Integration

**Goal**: Protect RAG and agent endpoints with authentication middleware and inject user context for personalized responses

**Independent Test**: Sign in, use chatbot, verify responses include interest-based analogies. As guest user, verify chatbot works with generic responses (no personalization).

### Implementation for Protected Endpoints

- [x] T057 [P] Add get_optional_user dependency to POST /agent/chat endpoint in backend/main.py (optional authentication)
- [x] T058 [P] Add get_required_user dependency to POST /interests/save endpoint in backend/main.py (required authentication)
- [x] T059 [P] Add get_required_user dependency to GET /interests/{user_id} endpoint in backend/main.py (required authentication, verify user_id matches token)
- [x] T060 Inject user_id into request.state in backend/middleware/auth_middleware.py (available to all downstream handlers)
- [x] T061 Update agent chat handler in backend/main.py to use request.state.user_id (pass to PersonalizationService if not None)
- [x] T062 Integrate user_id with PersonalizationService.build_system_prompt in backend/services/personalization_service.py (retrieve interests, build personalized prompt)
- [x] T063 Add user context to agent system prompt injection in backend/main.py (if authenticated: personalized prompt, else: base prompt)
- [x] T064 Update ChatContext.tsx in website/src/contexts/ChatContext.tsx to send Authorization header with JWT token
- [x] T065 Add personalization indicator in chatbot UI in website/src/components/ChatKit (show "Personalized for your interests" badge when authenticated)

**Checkpoint**: RAG chatbot now supports both authenticated (personalized) and guest (generic) users. All protected endpoints require proper authentication.

---

## Phase 7: User Story 3 - Interest Management for Existing Users (Priority: P2)

**Goal**: Allow authenticated users to view and update their interest selections from profile page

**Independent Test**: Sign in, navigate to /profile, view current interests, change interest selections (maintain 2-5), verify chatbot immediately reflects updated preferences

### Implementation for User Story 3

- [x] T066 [P] [US3] Create UpdateInterestsRequest Pydantic model in backend/models/auth.py (interests: List[int])
- [x] T067 [P] [US3] Create UpdateInterestsResponse Pydantic model in backend/models/auth.py (message, interests)
- [x] T068 [US3] Implement update_user_interests method in backend/repositories/user_repository.py (delete old associations, insert new ones)
- [x] T069 [US3] Create PUT /interests/{user_id} endpoint in backend/main.py (verify user_id matches auth token, update interests, return 200)
- [ ] T070 [US3] Add interest count validation (2-5) in PUT /interests/{user_id} endpoint
- [ ] T071 [US3] Update profile.tsx in website/src/pages/profile.tsx to display current user interests (fetch from /interests/{user_id})
- [ ] T072 [US3] Add interest editing UI in website/src/pages/profile.tsx (checkboxes, enforce 2-5 constraint, save button)
- [ ] T073 [US3] Implement interest update handler in website/src/pages/profile.tsx (call PUT /interests/{user_id}, show success message)
- [ ] T074 [US3] Add optimistic UI update in profile.tsx (update local state immediately, revert on error)
- [ ] T075 [US3] Clear personalization cache in PersonalizationService when interests updated (ensure immediate effect on next chat)

**Checkpoint**: At this point, User Stories 1, 2, 3, and 4 are complete. Users can register, sign in, manage sessions, and update interests.

---

## Phase 8: User Story 5 - Guest Access Without Authentication (Priority: P3)

**Goal**: Ensure guest users can access documentation and chatbot with non-personalized responses (backward compatibility)

**Independent Test**: Open platform without signing in, navigate to documentation, use chatbot, verify generic responses (no analogies), attempt to access /profile (redirected to /signin)

### Implementation for User Story 5

- [ ] T076 [P] [US5] Create ProtectedRoute component in website/src/components/ProtectedRoute.tsx (redirect to /signin if not authenticated)
- [ ] T077 [US5] Wrap /profile route with ProtectedRoute in website/docusaurus.config.ts or routing setup
- [ ] T078 [US5] Verify /agent/chat endpoint allows guest access (no Authorization header required)
- [ ] T079 [US5] Verify chatbot returns base prompt (no personalization) when user_id is None in backend/main.py
- [ ] T080 [US5] Add "Sign up for personalized responses" CTA in chatbot UI for guest users in website/src/components/ChatKit
- [ ] T081 [US5] Test documentation pages are accessible without authentication (Docusaurus default behavior)
- [ ] T082 [US5] Add value proposition messaging in website/src/pages/signup.tsx ("Get personalized analogies based on your interests")

**Checkpoint**: All user stories (1-5) are now independently functional. MVP is complete.

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and production readiness

- [ ] T083 [P] Add rate limiting to POST /auth/sign-in endpoint (5 attempts per 15 minutes per IP using FastAPI-Limiter)
- [ ] T084 [P] Add Content Security Policy headers in backend/main.py (prevent XSS)
- [ ] T085 [P] Add input sanitization for email and password fields in backend/services/auth_service.py
- [ ] T086 [P] Create comprehensive error response model in backend/models/auth.py (error_code, message, details)
- [ ] T087 Update quickstart.md in specs/006-better-auth-integration/quickstart.md with final setup instructions
- [ ] T088 [P] Add loading states to signup.tsx and signin.tsx (disable submit button during API call)
- [ ] T089 [P] Add success toast notifications in website/src/pages/signup.tsx and signin.tsx
- [ ] T090 [P] Add password visibility toggle in signup and signin forms
- [ ] T091 Run security audit with OWASP ZAP on authentication endpoints
- [ ] T092 [P] Add backend unit tests for auth_service.py (test sign_up, sign_in, password validation, email validation)
- [ ] T093 [P] Add backend unit tests for password_hasher.py (test bcrypt cost 12, verify_password)
- [ ] T094 [P] Add backend unit tests for token_manager.py (test JWT creation, expiration, signature verification)
- [ ] T095 [P] Add backend integration test for complete auth flow (sign-up → sign-in → validate → sign-out)
- [ ] T096 [P] Add frontend tests for AuthContext.tsx (test signUp, signIn, signOut state updates)
- [ ] T097 [P] Add frontend tests for useAuth.ts hook (test authentication flow)
- [ ] T098 Update .specify/agents/ClaudeCode.md with Better Auth capabilities per plan.md section 1.4
- [ ] T099 Create PHR for task implementation workflow
- [ ] T100 Run quickstart.md validation on clean development environment

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-8)**: All depend on Foundational phase completion
  - User Story 1 (P1): Can start after Foundational - No dependencies on other stories
  - User Story 2 (P1): Can start after Foundational - Independent of US1 but naturally follows it
  - User Story 4 (P1): Depends on US2 (sign-out requires sign-in to exist)
  - Phase 6 (Protected Endpoints): Depends on US1, US2, US4 (authentication must work first)
  - User Story 3 (P2): Depends on US1, US2 (must be able to sign up and sign in first)
  - User Story 5 (P3): Can start after Foundational - Tests backward compatibility
- **Polish (Phase 9)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories ✅ MVP START
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - Independent but naturally follows US1
- **User Story 4 (P1)**: Requires US2 (sign-out needs sign-in to exist first)
- **Protected Endpoints (Phase 6)**: Requires US1, US2, US4 complete (full auth lifecycle)
- **User Story 3 (P2)**: Requires US1 and US2 (must authenticate to update interests)
- **User Story 5 (P3)**: Independent after Foundational - Can be implemented anytime

### Within Each User Story

- Models before services
- Services before endpoints
- Backend endpoints before frontend integration
- Core implementation before validation/error handling
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T001-T004)
- All Foundational tasks marked [P] can run in parallel within their groups (T008-T010, T006-T007, T012-T013)
- Once Foundational phase completes:
  - US1 tasks marked [P] can run together (T016-T017, T026-T028)
  - US2 tasks marked [P] can run together (T033-T034)
  - US4 tasks marked [P] can run together (T045-T048)
- Polish tasks marked [P] can all run in parallel (T083-T097)

---

## Parallel Example: User Story 1 (Sign-Up)

```bash
# Launch all models for User Story 1 together:
Task T016: "Create SignUpRequest Pydantic model in backend/models/auth.py"
Task T017: "Create SignUpResponse Pydantic model in backend/models/auth.py"

# Launch all frontend context/infrastructure together:
Task T026: "Create AuthContext.tsx in website/src/contexts/AuthContext.tsx"
Task T027: "Create useAuth custom hook in website/src/hooks/useAuth.ts"
Task T028: "Create Better Auth client initialization in website/src/lib/auth.ts"
```

---

## Implementation Strategy

### MVP First (User Stories 1, 2, 4, Phase 6 Protected Endpoints)

1. Complete Phase 1: Setup (T001-T004)
2. Complete Phase 2: Foundational (T005-T015) - **CRITICAL BLOCKING PHASE**
3. Complete Phase 3: User Story 1 (T016-T032) - New user registration with interests
4. Complete Phase 4: User Story 2 (T033-T044) - Returning user sign-in
5. Complete Phase 5: User Story 4 (T045-T056) - Secure session management
6. Complete Phase 6: Protected Endpoints (T057-T065) - RAG chatbot personalization
7. **STOP and VALIDATE**: Test authentication flow end-to-end independently
8. Deploy/demo if ready

**MVP Scope**: 65 tasks (T001-T065) - Delivers secure authentication with interest-based personalization

### Incremental Delivery After MVP

1. Add User Story 3 (T066-T075) - Interest updates → Test independently → Deploy
2. Add User Story 5 (T076-T082) - Guest access verification → Test independently → Deploy
3. Add Polish (T083-T100) - Security hardening, testing, documentation → Deploy

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T015)
2. Once Foundational is done:
   - Developer A: User Story 1 (T016-T032)
   - Developer B: User Story 2 (T033-T044) + User Story 4 (T045-T056) sequentially
   - Developer C: Prepare Phase 6 integration (review PersonalizationService code)
3. Phase 6 (T057-T065): All developers collaborate on integration testing
4. User Stories 3 and 5 can be assigned independently after MVP validation

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Tests are optional per spec - included only where security validation critical
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
- Security tasks (rate limiting, CSP, audit logging) are in Polish phase but should not block MVP

---

## Task Count Summary

- **Phase 1 (Setup)**: 4 tasks
- **Phase 2 (Foundational)**: 11 tasks ⚠️ CRITICAL BLOCKING PHASE
- **Phase 3 (User Story 1 - Sign-Up)**: 17 tasks 🎯 MVP START
- **Phase 4 (User Story 2 - Sign-In)**: 12 tasks
- **Phase 5 (User Story 4 - Session Management)**: 12 tasks
- **Phase 6 (Protected Endpoints)**: 9 tasks ✅ MVP COMPLETE (T001-T065)
- **Phase 7 (User Story 3 - Interest Updates)**: 10 tasks
- **Phase 8 (User Story 5 - Guest Access)**: 7 tasks
- **Phase 9 (Polish)**: 18 tasks

**Total**: 100 tasks

**MVP Scope**: 65 tasks (Phases 1-6) - Delivers secure authentication with interest-based RAG personalization
**Post-MVP**: 35 tasks (Phases 7-9) - Interest management, guest access, polish
