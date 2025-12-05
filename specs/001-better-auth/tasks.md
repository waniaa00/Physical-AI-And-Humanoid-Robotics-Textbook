# Tasks: Better Auth — Full Authentication & Authorization

**Input**: Design documents from `specs/001-better-auth/`
**Prerequisites**: plan.md (✅ complete), spec.md (✅ complete)

**Organization**: Tasks are grouped by implementation phase following the plan.md sprint structure. Each user story from spec.md is addressed across relevant phases.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task supports (US1-US8)
- Include exact file paths in descriptions

## Path Conventions (Web App)

- **Backend**: `backend/src/`, `backend/tests/`
- **Frontend**: `website/src/`, `website/tests/`
- Follows plan.md project structure

---

## Phase 0: Research & Technology Selection (1-2 days)

**Purpose**: Answer critical technology choices before implementation

**⚠️ CRITICAL**: Must complete research before any implementation begins

### Research Tasks

- [ ] R001 [P] Evaluate Better Auth SDK options (FastAPI-Users vs Authlib vs custom) - Document in `specs/001-better-auth/research.md`
- [ ] R002 [P] Configure OAuth2 providers (Google Cloud Console, GitHub Apps) - Document setup steps in `research.md`
- [ ] R003 [P] Design database schema best practices (UUID vs auto-increment, bcrypt cost factor) - Document in `research.md`
- [ ] R004 [P] Research JWT security strategies (HS256 vs RS256, token rotation) - Document in `research.md`
- [ ] R005 [P] Evaluate rate limiting solutions (SlowAPI vs fastapi-limiter vs custom) - Document in `research.md`
- [ ] R006 [P] Research Qdrant access control integration (metadata filtering strategies) - Document in `research.md`
- [ ] R007 [P] Evaluate email services (SendGrid vs AWS SES vs Resend) - Document in `research.md`
- [ ] R008 [P] Research frontend auth state management (Context vs Zustand vs TanStack Query) - Document in `research.md`

**Checkpoint**: Research complete - Technology decisions documented in `research.md`

---

## Phase 1: Setup & Foundation (Day 1)

**Purpose**: Project initialization and blocking prerequisites for all user stories

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### Environment Setup

- [ ] T001 Create backend project structure per plan.md (`backend/src/`, `backend/tests/`, `backend/alembic/`)
- [ ] T002 Initialize Python 3.11+ project with virtual environment in `backend/`
- [ ] T003 [P] Create `backend/requirements.txt` with dependencies (FastAPI, SQLAlchemy, Alembic, python-jose, passlib, Authlib, slowapi, httpx, psycopg, qdrant-client, openai)
- [ ] T004 [P] Create `backend/.env.example` with environment variable template
- [ ] T005 [P] Setup frontend dependencies in `website/package.json` (zustand, react-hook-form, zod, axios)

### Database Foundation (Blocking)

- [ ] T006 Connect to Neon PostgreSQL from `backend/src/database/session.py`
- [ ] T007 Initialize Alembic migrations in `backend/database/migrations/`
- [ ] T008 Create initial migration for User table (id, email, hashed_password, is_verified, is_locked, failed_login_attempts, locked_until, created_at, updated_at, last_login_at) in `backend/database/migrations/versions/001_create_users.py`
- [ ] T009 [P] Create migration for UserProfile table in `backend/database/migrations/versions/002_create_user_profiles.py`
- [ ] T010 [P] Create migration for RefreshToken table in `backend/database/migrations/versions/003_create_refresh_tokens.py`
- [ ] T011 [P] Create migration for Role table (seed data: reader, contributor, admin) in `backend/database/migrations/versions/004_create_roles.py`
- [ ] T012 [P] Create migration for UserRole junction table in `backend/database/migrations/versions/005_create_user_roles.py`
- [ ] T013 [P] Create migration for AuditLog table in `backend/database/migrations/versions/006_create_audit_logs.py`
- [ ] T014 [P] Create migration for PasswordResetToken table in `backend/database/migrations/versions/007_create_password_reset_tokens.py`
- [ ] T015 [P] Create migration for EmailVerificationToken table in `backend/database/migrations/versions/008_create_email_verification_tokens.py`
- [ ] T016 [P] Extend QueryLog table with user_id (nullable) in `backend/database/migrations/versions/009_extend_query_logs.py`
- [ ] T017 Run all migrations on Neon database: `alembic upgrade head`

### SQLAlchemy Models (Blocking)

- [ ] T018 [P] Create User model in `backend/src/models/user.py`
- [ ] T019 [P] Create UserProfile model in `backend/src/models/user.py`
- [ ] T020 [P] Create RefreshToken model in `backend/src/models/session.py`
- [ ] T021 [P] Create Role and UserRole models in `backend/src/models/role.py`
- [ ] T022 [P] Create AuditLog model in `backend/src/models/audit_log.py`
- [ ] T023 [P] Create PasswordResetToken model in `backend/src/models/user.py`
- [ ] T024 [P] Create EmailVerificationToken model in `backend/src/models/user.py`
- [ ] T025 [P] Extend QueryLog model in `backend/src/models/query_log.py`

### FastAPI Foundation (Blocking)

- [ ] T026 Create FastAPI app entry point in `backend/src/main.py`
- [ ] T027 Configure CORS middleware in `backend/src/middleware/cors.py`
- [ ] T028 Setup config management with Pydantic Settings in `backend/src/config.py`
- [ ] T029 Create database session dependency `get_db()` in `backend/src/database/session.py`

**Checkpoint**: Foundation ready - Database schema created, models defined, FastAPI initialized

---

## Phase 2: Authentication Core (User Story 1 - Email/Password Auth, Priority P1) 🎯 MVP

**Goal**: Implement secure email/password sign-up, login, logout with email verification

**Independent Test**: Create account → receive verification email → verify email → login → access protected endpoint → logout

### Password & JWT Utilities

- [ ] T030 [P] [US1] Implement password hashing utilities (bcrypt cost 12) in `backend/src/auth/password.py`
- [ ] T031 [P] [US1] Implement JWT creation/validation (HS256, 15-min expiry) in `backend/src/auth/jwt.py`
- [ ] T032 [P] [US1] Create Pydantic schemas for auth requests/responses in `backend/src/auth/schemas.py`

### Email Service Integration

- [ ] T033 [US1] Integrate email service (SendGrid/AWS SES/Resend per research) in `backend/src/auth/email.py`
- [ ] T034 [P] [US1] Create email verification template
- [ ] T035 [P] [US1] Create password reset email template
- [ ] T036 [P] [US1] Create account locked notification template

### Sign-Up Flow (US1)

- [ ] T037 [US1] Implement POST /auth/signup endpoint in `backend/src/auth/router.py`
  - Validate email format and password policy (FR-009)
  - Hash password with bcrypt (FR-003)
  - Create User record (is_verified=false)
  - Generate email verification token (32 bytes, SHA-256)
  - Send verification email
  - Return user_id and message

### Email Verification (US1)

- [ ] T038 [US1] Implement POST /auth/verify-email endpoint in `backend/src/auth/router.py`
  - Validate token (not expired, not used)
  - Update User.is_verified = true
  - Return success message

### Login Flow (US1)

- [ ] T039 [US1] Implement POST /auth/login endpoint in `backend/src/auth/router.py`
  - Find user by email (case-insensitive)
  - Check account lock status (locked_until)
  - Verify password with bcrypt
  - Handle failed attempts (increment counter, lock after 5 failures for 15 min)
  - Generate JWT access token (15-min exp)
  - Generate refresh token (random 32 bytes)
  - Store RefreshToken (SHA-256 hash, expires_at = NOW + 7 days)
  - Set HttpOnly cookie with refresh token
  - Log audit event (login_success or login_failed)
  - Return access_token and user info

### Logout & Token Refresh (US1, US5)

- [ ] T040 [US1] [US5] Implement POST /auth/logout endpoint in `backend/src/auth/router.py`
  - Revoke current RefreshToken (revoked_at = NOW)
  - Clear refresh_token cookie
  - Log audit event (logout)

- [ ] T041 [US5] Implement POST /auth/refresh endpoint in `backend/src/auth/router.py`
  - Read refresh_token from cookie
  - Hash token with SHA-256, lookup RefreshToken
  - Validate (not revoked, not expired)
  - Generate new access token
  - Rotate refresh token (mark old as revoked, create new)
  - Set new refresh_token cookie
  - Return new access_token

### Token Theft Detection (US5)

- [ ] T042 [US5] Implement token theft detection logic in `backend/src/auth/router.py`
  - If revoked token is used, invalidate ALL user's refresh tokens
  - Require re-authentication
  - Log security event

### Rate Limiting (US1)

- [ ] T043 [US1] Implement SlowAPI rate limiter in `backend/src/middleware/rate_limit.py`
- [ ] T044 [US1] Apply rate limiting to POST /auth/login (5 attempts / IP / 5 minutes)
- [ ] T045 [US1] Apply rate limiting to POST /auth/signup (10 attempts / IP / hour)

**Checkpoint**: User Story 1 complete - Email/password auth working end-to-end

---

## Phase 3: Password Reset (User Story 3, Priority P1)

**Goal**: Implement secure password reset flow with time-limited tokens

**Independent Test**: Request reset → receive email → click link → set new password → login with new password

### Password Reset Endpoints

- [ ] T046 [US3] Implement POST /auth/request-password-reset endpoint in `backend/src/auth/router.py`
  - Find user by email (don't reveal if not exists)
  - Generate reset token (32 bytes, SHA-256, 1-hour expiry)
  - Send password reset email
  - Return generic message

- [ ] T047 [US3] Implement POST /auth/reset-password endpoint in `backend/src/auth/router.py`
  - Validate token (not expired, not used)
  - Validate new password meets policy (FR-009)
  - Hash new password with bcrypt
  - Update User.hashed_password
  - Mark token as used
  - Invalidate all refresh tokens (force re-login)
  - Return success message

- [ ] T048 [US3] Apply rate limiting to password reset (3 requests / email / hour)

**Checkpoint**: User Story 3 complete - Password reset flow working

---

## Phase 4: OAuth2 Social Login (User Story 2, Priority P2)

**Goal**: Implement Google and GitHub OAuth2 login with account linking

**Independent Test**: Click "Sign in with Google" → authorize → logged in (new account created or existing linked)

### OAuth2 Configuration

- [ ] T049 [P] [US2] Configure Google OAuth client with Authlib in `backend/src/auth/oauth.py`
- [ ] T050 [P] [US2] Configure GitHub OAuth client with Authlib in `backend/src/auth/oauth.py`

### OAuth2 Endpoints

- [ ] T051 [US2] Implement GET /auth/oauth/google endpoint in `backend/src/auth/router.py`
  - Generate OAuth state (CSRF protection)
  - Redirect to Google consent screen

- [ ] T052 [US2] Implement GET /auth/callback/google endpoint in `backend/src/auth/router.py`
  - Validate state (CSRF check)
  - Exchange code for Google access token
  - Fetch user profile (email, name, picture)
  - Check if User exists (by email)
    - If exists: link Google account (if not already linked), log in
    - If not exists: create User (is_verified=true, hashed_password=null)
  - Issue JWT + refresh token
  - Set refresh_token cookie
  - Redirect to frontend with success

- [ ] T053 [US2] Implement GET /auth/oauth/github endpoint in `backend/src/auth/router.py`
- [ ] T054 [US2] Implement GET /auth/callback/github endpoint in `backend/src/auth/router.py`

### Account Linking Logic

- [ ] T055 [US2] Implement account linking validation in `backend/src/auth/oauth.py`
  - Prevent linking OAuth account already linked to another user
  - Show appropriate error message

**Checkpoint**: User Story 2 complete - OAuth2 login working for Google and GitHub

---

## Phase 5: Session Management (User Story 5, Priority P1)

**Goal**: Allow users to view and manage active sessions across devices

**Independent Test**: Login from multiple devices → view session list → revoke specific session → verify device logged out

### Session Endpoints

- [ ] T056 [US5] Implement GET /auth/me endpoint in `backend/src/auth/router.py`
  - Validate JWT from Authorization header
  - Return user info (id, email, display_name, avatar_url, roles)

- [ ] T057 [US5] Implement GET /auth/sessions endpoint in `backend/src/auth/router.py`
  - Require authentication
  - Query RefreshToken table for user's active sessions
  - Return list (id, device_info, ip_address, created_at, last_active)

- [ ] T058 [US5] Implement DELETE /auth/sessions/{id} endpoint in `backend/src/auth/router.py`
  - Require authentication
  - Revoke specific session (revoked_at = NOW)
  - Log audit event

- [ ] T059 [US5] Implement DELETE /auth/sessions endpoint in `backend/src/auth/router.py`
  - Require authentication
  - Revoke all sessions except current
  - Log audit event

**Checkpoint**: User Story 5 complete - Session management working

---

## Phase 6: RBAC Implementation (User Story 4, Priority P2)

**Goal**: Implement role-based access control with reader, contributor, admin roles

**Independent Test**: Create users with different roles → test access to protected endpoints (ingestion, admin) → verify permissions enforced

### RBAC Dependencies & Middleware

- [ ] T060 [US4] Implement `get_current_user` dependency in `backend/src/auth/dependencies.py`
  - Validate JWT from Authorization header
  - Decode token, extract user_id
  - Query User from database
  - Return User object or raise 401

- [ ] T061 [US4] Implement `get_current_user_optional` dependency in `backend/src/auth/dependencies.py`
  - Same as get_current_user but return None if no token (for anonymous access)

- [ ] T062 [US4] Implement `require_role` dependency factory in `backend/src/auth/dependencies.py`
  - Accept required_role parameter (reader, contributor, admin)
  - Check user's roles (via UserRole junction table)
  - Raise 403 if user doesn't have required role

### Role Assignment on Signup

- [ ] T063 [US4] Update POST /auth/signup endpoint to assign "reader" role by default
  - Create UserRole entry linking user to "reader" role

### Protect RAG Endpoints (US4)

- [ ] T064 [US4] Update POST /rag/ingest endpoint in `backend/src/rag/router.py`
  - Add `require_role("contributor")` dependency
  - Only contributors and admins can ingest documents

- [ ] T065 [US4] Update POST /rag/query-global endpoint in `backend/src/rag/router.py`
  - Add `get_current_user_optional` dependency
  - Allow anonymous users (read-only)
  - Log query with user_id if authenticated

- [ ] T066 [US4] Update POST /rag/query-local endpoint in `backend/src/rag/router.py`
  - Add `get_current_user_optional` dependency
  - Allow anonymous users

**Checkpoint**: User Story 4 (RBAC core) complete - Role enforcement working on endpoints

---

## Phase 7: Admin Panel API (User Story 7, Priority P2)

**Goal**: Implement admin endpoints for user management, role assignment, and audit log viewing

**Independent Test**: Login as admin → view user list → change user role → view audit logs → verify changes

### Admin User Management

- [ ] T067 [US7] Implement GET /admin/users endpoint in `backend/src/admin/router.py`
  - Require admin role
  - Support pagination (page, limit)
  - Support search (email filter)
  - Support role filter
  - Return users list with roles and account status

- [ ] T068 [US7] Implement GET /admin/users/{user_id} endpoint in `backend/src/admin/router.py`
  - Require admin role
  - Return user details with roles, sessions, recent audit logs

- [ ] T069 [US7] Implement PATCH /admin/users/{user_id}/role endpoint in `backend/src/admin/router.py`
  - Require admin role
  - Validate user cannot modify own role (FR-023)
  - Update UserRole (remove old, add new)
  - Log audit event (role_changed)
  - Return success message

- [ ] T070 [US7] Implement DELETE /admin/users/{user_id}/sessions endpoint in `backend/src/admin/router.py`
  - Require admin role
  - Revoke all user's refresh tokens
  - Log audit event
  - Return success message

### Admin Audit Log Viewing

- [ ] T071 [US7] Implement GET /admin/audit-logs endpoint in `backend/src/admin/router.py`
  - Require admin role
  - Support pagination
  - Support filters (event_type, user_id, start_date, end_date)
  - Return audit logs

### Admin Safeguards

- [ ] T072 [US7] Implement self-modification prevention in PATCH /admin/users/{user_id}/role
  - Check if user_id matches current_user.id
  - Return error: "Cannot modify your own role"

- [ ] T073 [US7] Implement last-admin-deletion prevention
  - Check if deleting/demoting last admin
  - Return error: "Cannot delete/demote the last admin account"

**Checkpoint**: User Story 7 complete - Admin panel API working

---

## Phase 8: Account Deletion & Data Export (User Story 6, Priority P3)

**Goal**: Implement GDPR-compliant account deletion and data export

**Independent Test**: Create account → add data (queries, profile) → delete account → verify all PII removed

### Account Deletion

- [ ] T074 [US6] Implement DELETE /auth/account endpoint in `backend/src/auth/router.py`
  - Require authentication + password confirmation
  - Cascade delete User → UserProfile, RefreshToken, UserRole
  - Anonymize QueryLog (set user_id = NULL)
  - Log audit event (account_deleted)
  - Return success message

### Data Export

- [ ] T075 [US6] Implement GET /auth/export endpoint in `backend/src/auth/router.py`
  - Require authentication
  - Query user's profile data
  - Query user's query history (QueryLog)
  - Generate JSON export
  - Return downloadable JSON file

**Checkpoint**: User Story 6 complete - GDPR compliance implemented

---

## Phase 9: Anonymous User Support (User Story 8, Priority P2)

**Goal**: Allow anonymous users to access RAG queries with graceful degradation

**Independent Test**: Access site without login → ask RAG query → receive answer → see prompts to sign in for history/feedback

### Anonymous Access

- [ ] T076 [US8] Update frontend RAG chatbot widget in `website/src/components/RAGChatbot/ChatWidget.tsx`
  - Check auth state (isAuthenticated)
  - Show "Sign in to save chat history" message if anonymous
  - Allow query submission without auth

- [ ] T077 [US8] Update POST /rag/query-global to handle anonymous users (already done in T065)
- [ ] T078 [US8] Update POST /rag/query-local to handle anonymous users (already done in T066)

### Feature Prompts for Anonymous Users

- [ ] T079 [US8] Update feedback UI to show "Sign in to provide feedback" for anonymous users
- [ ] T080 [US8] Update document ingestion UI to show "Contributors only - Sign up to request access"

**Checkpoint**: User Story 8 complete - Anonymous user graceful degradation working

---

## Phase 10: Qdrant Integration & RAG Access Control

**Goal**: Integrate Qdrant Cloud with role-based metadata filtering for secure RAG

**Independent Test**: Ingest document as contributor with access roles → query as reader → verify access control

### Qdrant Setup

- [ ] T081 [P] Connect to Qdrant Cloud from `backend/src/rag/qdrant.py`
- [ ] T082 [P] Create "book_content" collection with vector dimensions matching OpenAI embeddings

### Qdrant Access Control

- [ ] T083 Implement `get_qdrant_filter` function in `backend/src/rag/dependencies.py`
  - If anonymous: filter `access.is_public = true`
  - If reader: filter `access.is_public = true OR access.roles contains "reader"`
  - If contributor/admin: no filter (access all)

- [ ] T084 Update POST /rag/ingest endpoint to tag points with access metadata
  - Add payload: `{ "access": { "is_public": true/false, "roles": ["reader", "contributor", "admin"], "created_by": user_id } }`

- [ ] T085 Update POST /rag/query-global to apply metadata filter
  - Use get_qdrant_filter to generate filter based on user role
  - Pass filter to qdrant_client.search()

### RAG Pipeline

- [ ] T086 [P] Implement OpenAI embedding generation in `backend/src/rag/embeddings.py`
- [ ] T087 Implement POST /rag/embed endpoint for generating embeddings
- [ ] T088 Implement context-restricted RAG (no hallucinations) in `backend/src/rag/query.py`
  - Query Qdrant with metadata filter
  - Pass top-k results to OpenAI Agents
  - Return answer restricted to retrieved context

### Selected Text RAG (US8)

- [ ] T089 [US8] Implement POST /rag/query-local endpoint for selected text
  - Accept selected_text in request body
  - Embed selected text
  - Query Qdrant with selected_text embedding
  - Return answer based on selected text context

**Checkpoint**: Qdrant integration complete - RAG access control working

---

## Phase 11: Frontend Authentication UI

**Goal**: Build login, signup, account settings UI in Docusaurus

**Independent Test**: Navigate to /signin → login → see authenticated navbar → access account settings → logout

### Auth UI Components

- [ ] T090 [P] Create LoginForm component in `website/src/components/Auth/LoginForm.tsx`
  - Email and password fields
  - Form validation with react-hook-form + zod
  - OAuth buttons (Google, GitHub)
  - "Forgot Password?" link
  - Submit to POST /auth/login
  - Store access_token in Zustand store
  - Redirect to return_url or dashboard

- [ ] T091 [P] Create SignupForm component in `website/src/components/Auth/SignupForm.tsx`
  - Email and password fields
  - Password policy validation (12 chars, uppercase, lowercase, number, special)
  - OAuth buttons
  - Submit to POST /auth/signup
  - Show "Check your email for verification link" message

- [ ] T092 [P] Create PasswordReset component in `website/src/components/Auth/PasswordReset.tsx`
  - Request reset view (email input)
  - Reset password view (new password input with token from URL)
  - Submit to POST /auth/request-password-reset or POST /auth/reset-password

- [ ] T093 [P] Create AccountSettings component in `website/src/components/Auth/AccountSettings.tsx`
  - Display user profile (email, display_name, avatar)
  - Active sessions list with revoke buttons
  - Delete account button (with password confirmation)
  - Data export button

- [ ] T094 [P] Create SessionList component in `website/src/components/Auth/SessionList.tsx`
  - Fetch sessions from GET /auth/sessions
  - Display device_info, ip_address, created_at
  - Revoke button for each session
  - "Revoke all other sessions" button

### Auth Pages

- [ ] T095 Create /signin page in `website/src/pages/signin.tsx`
  - Render LoginForm component

- [ ] T096 Create /signup page in `website/src/pages/signup.tsx`
  - Render SignupForm component

- [ ] T097 Create /account page in `website/src/pages/account.tsx`
  - Render AccountSettings component
  - Protected route (require authentication)

- [ ] T098 Create /reset-password page in `website/src/pages/reset-password.tsx`
  - Render PasswordReset component

**Checkpoint**: Auth UI complete - Users can sign up, log in, manage account

---

## Phase 12: Frontend Auth State Management

**Goal**: Implement Zustand store with auto-token-refresh

**Independent Test**: Login → wait 13 minutes → verify access token refreshes automatically → make API call → verify no interruption

### Zustand Auth Store

- [ ] T099 Create Zustand auth store in `website/src/lib/auth/authStore.ts`
  - State: user (User | null), accessToken (string | null), isAuthenticated (boolean), isLoading (boolean)
  - Actions: setUser, logout, refreshToken
  - Persist user to localStorage (not access token)

- [ ] T100 Create authService API functions in `website/src/lib/auth/authService.ts`
  - login(email, password): POST /auth/login
  - signup(email, password): POST /auth/signup
  - logout(): POST /auth/logout
  - refreshToken(): POST /auth/refresh
  - getCurrentUser(): GET /auth/me
  - OAuth redirect functions

- [ ] T101 Create useAuth hook in `website/src/lib/auth/useAuth.ts`
  - Re-export authStore selectors
  - Provide convenient auth state access

- [ ] T102 Create auto-token-refresh hook in `website/src/lib/auth/useTokenRefresh.ts`
  - Decode JWT to get expiration
  - Set timeout to refresh 2 minutes before expiry
  - Call refreshToken() automatically
  - Clear timeout on unmount

### Axios Interceptor

- [ ] T103 Create axios instance with auth interceptor in `website/src/lib/auth/axiosConfig.ts`
  - Add Authorization header with access token
  - Handle 401 errors (auto-logout and redirect to /signin)

**Checkpoint**: Auth state management complete - Auto-refresh working

---

## Phase 13: Protected Routes & Navigation

**Goal**: Protect routes and update navbar based on auth state

**Independent Test**: Access /admin as non-admin → see 403 error → login as admin → access /admin → see admin panel

### Protected Route Component

- [ ] T104 Create ProtectedRoute component in `website/src/components/ProtectedRoute.tsx`
  - Check isAuthenticated from auth store
  - Check user role if requireRole specified
  - Redirect to /signin if not authenticated
  - Show "Insufficient permissions" if wrong role

### Navbar Updates

- [ ] T105 Update navbar in `website/docusaurus.config.ts` or custom navbar component
  - Show "Sign In" button if anonymous
  - Show user avatar + dropdown menu if authenticated
    - Account Settings
    - Logout
  - Show "Admin Panel" link if user has admin role

### Admin Pages

- [ ] T106 Create /admin page in `website/src/pages/admin/index.tsx`
  - Protected route (require admin role)
  - Show navigation to Users, Audit Logs

- [ ] T107 Create /admin/users page in `website/src/pages/admin/users.tsx`
  - Fetch users from GET /admin/users
  - Search and filter UI
  - Change role button
  - Revoke sessions button

- [ ] T108 Create /admin/audit page in `website/src/pages/admin/audit.tsx`
  - Fetch audit logs from GET /admin/audit-logs
  - Filter by event type, user, date range
  - Paginated table view

**Checkpoint**: Protected routes and navigation complete

---

## Phase 14: RAG Chatbot Frontend Integration

**Goal**: Embed RAG chatbot widget into Docusaurus book with auth integration

**Independent Test**: Read book → select text → click "Ask AI" → see answer based on selected text

### Chatbot Widget

- [ ] T109 Create ChatWidget component in `website/src/components/RAGChatbot/ChatWidget.tsx`
  - Chat UI with message history
  - Input field for query
  - "Ask AI" button
  - Loading state
  - Submit to POST /rag/query-global
  - Display answer with sources
  - Show "Sign in to save chat history" if anonymous

- [ ] T110 Create selected text capture in ChatWidget
  - Listen for text selection events
  - Show "Ask AI about this selection" button when text selected
  - Submit selected text to POST /rag/query-local

- [ ] T111 Embed ChatWidget into Docusaurus theme
  - Add widget to docusaurus.config.ts or custom theme
  - Position as floating chat button
  - Expand/collapse animation

**Checkpoint**: RAG chatbot widget complete - Users can query book content

---

## Phase 15: Security Hardening

**Goal**: Implement comprehensive security measures

**Independent Test**: Attempt brute-force login → account locked after 5 failures → wait 15 min → can login again

### Security Middleware

- [ ] T112 [P] Implement CSRF protection in `backend/src/middleware/csrf.py`
  - Generate CSRF tokens
  - Validate CSRF tokens on state-changing endpoints

- [ ] T113 Verify rate limiting is active on all auth endpoints (already implemented in T043-T045, T048)

- [ ] T114 [P] Implement audit logging for all security events in `backend/src/auth/router.py`
  - Create audit log entries for login_success, login_failed, password_reset, role_changed, session_revoked
  - Anonymize IP addresses (mask last octet)

### HTTPS & Cookie Security

- [ ] T115 Configure production environment variables in `backend/.env`
  - Set ENVIRONMENT=production
  - Enable HTTPS-only cookies (Secure flag)
  - Set SameSite=Lax for cookies

- [ ] T116 Configure CORS for production in `backend/src/middleware/cors.py`
  - Whitelist frontend domain
  - No wildcard origins in production

**Checkpoint**: Security hardening complete

---

## Phase 16: Testing & Validation

**Goal**: Comprehensive testing across all user stories

### Unit Tests (Backend)

- [ ] T117 [P] Unit tests for password hashing in `backend/tests/unit/test_password.py`
- [ ] T118 [P] Unit tests for JWT creation/validation in `backend/tests/unit/test_jwt.py`
- [ ] T119 [P] Unit tests for email sending in `backend/tests/unit/test_email.py`

### Integration Tests (Backend)

- [ ] T120 [US1] Integration test: Sign-up → verify email → login flow in `backend/tests/integration/test_auth_flow.py`
- [ ] T121 [US2] Integration test: OAuth login (Google) in `backend/tests/integration/test_oauth.py`
- [ ] T122 [US3] Integration test: Password reset flow in `backend/tests/integration/test_password_reset.py`
- [ ] T123 [US4] Integration test: RBAC enforcement on protected endpoints in `backend/tests/integration/test_rbac.py`
- [ ] T124 [US5] Integration test: Session management (list, revoke) in `backend/tests/integration/test_sessions.py`
- [ ] T125 [US6] Integration test: Account deletion (GDPR) in `backend/tests/integration/test_account_deletion.py`
- [ ] T126 [US7] Integration test: Admin user management in `backend/tests/integration/test_admin.py`

### End-to-End Tests

- [ ] T127 [US1] E2E test: Complete sign-up to login journey
- [ ] T128 [US2] E2E test: OAuth login with account linking
- [ ] T129 [US3] E2E test: Forgot password to successful login
- [ ] T130 [US4] E2E test: Reader denied access to ingestion, contributor allowed
- [ ] T131 [US5] E2E test: Token refresh happens automatically
- [ ] T132 [US5] E2E test: Token theft detection triggers logout
- [ ] T133 E2E test: Rate limiting locks account after 5 failed logins
- [ ] T134 [US6] E2E test: Account deletion removes all PII
- [ ] T135 [US7] E2E test: Admin changes user role, user gains permissions
- [ ] T136 [US8] E2E test: Anonymous user queries RAG, sees sign-in prompts
- [ ] T137 E2E test: RAG query with role-based access control (Qdrant filtering)

### Security Audit

- [ ] T138 [P] Verify no hardcoded secrets in codebase
- [ ] T139 [P] Verify HTTPS-only cookies in production config
- [ ] T140 [P] Verify CORS whitelist (no wildcards in production)
- [ ] T141 [P] Verify JWT signature validation on all protected endpoints
- [ ] T142 [P] Verify SQL injection prevention (parameterized queries)
- [ ] T143 [P] Verify XSS prevention (input sanitization in frontend)

**Checkpoint**: All tests passing, security audit complete

---

## Phase 17: Deployment

**Goal**: Deploy to production with proper configuration

### Backend Deployment

- [ ] T144 Create production Dockerfile for FastAPI backend in `backend/Dockerfile`
- [ ] T145 Configure environment variables for production in deployment platform (Vercel/Render/Fly.io)
  - DATABASE_URL (Neon PostgreSQL)
  - JWT_SECRET
  - GOOGLE_CLIENT_ID, GOOGLE_CLIENT_SECRET
  - GITHUB_CLIENT_ID, GITHUB_CLIENT_SECRET
  - SENDGRID_API_KEY or AWS_SES credentials
  - QDRANT_URL, QDRANT_API_KEY
  - OPENAI_API_KEY
  - CORS_ORIGINS (frontend domain)
  - ENVIRONMENT=production

- [ ] T146 Deploy FastAPI backend to production platform
- [ ] T147 Run database migrations on production Neon database: `alembic upgrade head`
- [ ] T148 Seed roles in production database (reader, contributor, admin)

### Frontend Deployment

- [ ] T149 Configure production API URL in `website/.env.production`
  - NEXT_PUBLIC_API_URL=https://api.yourdomain.com

- [ ] T150 Deploy Docusaurus frontend to Vercel
- [ ] T151 Configure OAuth redirect URIs for production domain in Google Cloud Console and GitHub

### OAuth Configuration (Production)

- [ ] T152 [P] Update Google OAuth redirect URI to `https://api.yourdomain.com/auth/callback/google`
- [ ] T153 [P] Update GitHub OAuth redirect URI to `https://api.yourdomain.com/auth/callback/github`

### Smoke Tests (Production)

- [ ] T154 Smoke test: Sign up with email/password on production
- [ ] T155 Smoke test: Login with Google OAuth on production
- [ ] T156 Smoke test: Login with GitHub OAuth on production
- [ ] T157 Smoke test: RAG query as authenticated user
- [ ] T158 Smoke test: RAG query as anonymous user
- [ ] T159 Smoke test: Admin panel access with admin role
- [ ] T160 Smoke test: Token refresh after 15 minutes
- [ ] T161 Monitor logs for errors (CloudWatch/Sentry/Render logs)

**Checkpoint**: Production deployment complete, smoke tests passing

---

## Phase 18: Documentation & Polish

**Goal**: Complete documentation for developers and users

### Developer Documentation

- [ ] T162 Create `specs/001-better-auth/data-model.md` with complete database schema DDL
- [ ] T163 Create `specs/001-better-auth/contracts/auth-api.yaml` with OpenAPI spec for auth endpoints
- [ ] T164 Create `specs/001-better-auth/contracts/admin-api.yaml` with OpenAPI spec for admin endpoints
- [ ] T165 Create `specs/001-better-auth/quickstart.md` with developer setup guide

### User Documentation

- [ ] T166 [P] Add authentication guide to Docusaurus docs
- [ ] T167 [P] Add admin panel user guide
- [ ] T168 [P] Update architecture diagrams with auth flow

### Code Quality

- [ ] T169 [P] Run code linting and formatting (black, isort, mypy for backend; eslint, prettier for frontend)
- [ ] T170 [P] Code review and refactoring
- [ ] T171 Validate `quickstart.md` instructions work end-to-end

**Checkpoint**: Documentation complete, code quality verified

---

## Dependencies & Execution Order

### Phase Dependencies

1. **Phase 0 (Research)**: No dependencies - Start immediately → **BLOCKS all other phases**
2. **Phase 1 (Setup & Foundation)**: Depends on Research → **BLOCKS all user story phases**
3. **Phase 2 (Email/Password Auth - US1)**: Depends on Foundation → MVP!
4. **Phase 3 (Password Reset - US3)**: Depends on Foundation
5. **Phase 4 (OAuth - US2)**: Depends on Foundation
6. **Phase 5 (Session Management - US5)**: Depends on Phase 2 (login flow)
7. **Phase 6 (RBAC - US4)**: Depends on Foundation
8. **Phase 7 (Admin Panel - US7)**: Depends on Phase 6 (RBAC)
9. **Phase 8 (Account Deletion - US6)**: Depends on Foundation
10. **Phase 9 (Anonymous Support - US8)**: Depends on Phase 11 (Frontend UI)
11. **Phase 10 (Qdrant Integration)**: Can start after Phase 6 (RBAC) for access control
12. **Phase 11 (Frontend UI)**: Depends on Phases 2-5 (backend auth endpoints ready)
13. **Phase 12 (Frontend State)**: Depends on Phase 11 (UI components)
14. **Phase 13 (Protected Routes)**: Depends on Phase 12 (auth state)
15. **Phase 14 (Chatbot Widget)**: Depends on Phase 10 (Qdrant) and Phase 12 (auth state)
16. **Phase 15 (Security Hardening)**: Can run parallel with Phases 11-14
17. **Phase 16 (Testing)**: Depends on all feature phases (2-15) complete
18. **Phase 17 (Deployment)**: Depends on Phase 16 (tests passing)
19. **Phase 18 (Documentation)**: Can run parallel with implementation

### Critical Path (Minimum Viable Product)

**MVP = User Story 1 (Email/Password Auth) + Anonymous RAG Access**

1. Phase 0 (Research) → 1-2 days
2. Phase 1 (Setup & Foundation) → 1 day
3. Phase 2 (Email/Password Auth - US1) → 2-3 days
4. Phase 10 (Qdrant Integration - basic) → 1-2 days
5. Phase 11 (Frontend Login UI) → 1-2 days
6. Phase 12 (Frontend Auth State) → 1 day
7. Phase 14 (Chatbot Widget - basic) → 1-2 days
8. Phase 16 (Core Testing) → 2 days
9. Phase 17 (Deployment) → 1-2 days

**Total MVP Timeline**: 10-15 days

### Parallel Opportunities

**After Foundation (Phase 1) completes, these can run in parallel**:
- Phase 2 (US1 - Email/Password)
- Phase 3 (US3 - Password Reset)
- Phase 4 (US2 - OAuth)
- Phase 6 (US4 - RBAC)
- Phase 8 (US6 - Account Deletion)

**Frontend phases can run in parallel** (if backend APIs ready):
- Phase 11 (UI Components)
- Phase 12 (Auth State)
- Phase 13 (Protected Routes)

**Always parallel**:
- All tasks marked [P] within a phase
- All test tasks marked [P]
- Phase 15 (Security) can overlap with Phases 11-14
- Phase 18 (Documentation) can overlap with implementation

---

## Implementation Strategy

### MVP First (3 weeks)

**Week 1: Foundation + Email/Password Auth**
1. Complete Phase 0 (Research) - 1-2 days
2. Complete Phase 1 (Setup & Foundation) - 1 day
3. Start Phase 2 (Email/Password Auth)
4. VALIDATE: Can sign up, verify email, login

**Week 2: RAG Integration + Frontend**
5. Complete Phase 2 (Email/Password Auth)
6. Complete Phase 10 (Qdrant Integration - basic)
7. Start Phase 11 (Frontend UI - Login/Signup)
8. Start Phase 12 (Frontend Auth State)
9. VALIDATE: Can login via frontend, see auth state

**Week 3: Chatbot + Deployment**
10. Complete Phase 14 (Chatbot Widget)
11. Complete Phase 16 (Core Testing)
12. Complete Phase 17 (Deployment)
13. VALIDATE: MVP deployed - Users can sign up, login, query RAG chatbot

### Full Feature Rollout (4-5 weeks)

After MVP, incrementally add user stories:
- **Week 4**: Add OAuth (US2) + Password Reset (US3) + Session Management (US5)
- **Week 5**: Add RBAC (US4) + Admin Panel (US7) + Account Deletion (US6)
- **Week 6**: Polish, documentation, final testing

### Parallel Team Strategy

With 3 developers:
1. **Backend Dev**: Phases 0, 1, 2, 3, 4, 5, 6, 7, 8, 10 (sequential)
2. **Frontend Dev**: Phases 11, 12, 13, 14 (after backend APIs ready)
3. **DevOps/Testing**: Phase 15 (Security), Phase 16 (Testing), Phase 17 (Deployment)

---

## Success Criteria Validation

All 23 success criteria from spec.md are addressed:

- **SC-001 to SC-005 (User Experience)**: Phases 2-5, 11-14
- **SC-006 to SC-010 (Security)**: Phases 2, 6, 15, 16
- **SC-011 to SC-015 (Operational)**: Phases 17, 8
- **SC-016 to SC-018 (Admin)**: Phase 7
- **SC-019 to SC-023 (Integration)**: Phases 11-14, 17

---

## Notes

- [P] = Parallelizable (different files, no dependencies)
- [Story] = User story reference (US1-US8)
- Tasks follow plan.md sprint structure
- Research must complete before implementation
- Foundation blocks all user stories
- User stories can proceed in parallel after foundation
- Tests validate each story independently
- MVP = US1 + Anonymous RAG (10-15 days)
- Full feature set = 4-5 weeks
