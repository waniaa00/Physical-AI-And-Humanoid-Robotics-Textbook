# Feature Specification: Better Auth — Full Authentication & Authorization

**Feature Branch**: `001-better-auth`
**Created**: 2025-12-05
**Status**: Draft
**Input**: User description: "Better Auth — Full Authentication & Authorization for RAG Chatbot + Docusaurus Book"

## User Scenarios & Testing

### User Story 1 - Email/Password Sign-Up & Login (Priority: P1)

A new user wants to create an account to access personalized features like saved chat history and document annotations. They sign up with their email and password, receive a verification email, click the link to verify their account, and log in to access the personalized features.

**Why this priority**: This is the foundation of the auth system. Without basic sign-up and login, no other auth features can function. This enables user identity tracking, which is essential for personalized experiences and access control.

**Independent Test**: Can be fully tested by creating a new account through the sign-up form, receiving and clicking the verification email, and logging in successfully. Delivers the value of secure user identity management.

**Acceptance Scenarios**:

1. **Given** a new user on the sign-up page, **When** they enter valid email and password (meeting password policy), **Then** an account is created and a verification email is sent
2. **Given** an unverified user receives a verification email, **When** they click the verification link, **Then** their account is marked as verified
3. **Given** a verified user on the login page, **When** they enter correct credentials, **Then** they are logged in and redirected to their previous page or dashboard
4. **Given** a user enters incorrect credentials, **When** they attempt to log in, **Then** they see an error message and login fails (without revealing whether email or password was wrong)
5. **Given** a user attempts login 5 times with wrong password, **When** the 5th attempt fails, **Then** the account is temporarily locked for 15 minutes

---

### User Story 2 - Social Login (Google & GitHub OAuth) (Priority: P2)

A user wants to quickly sign up or log in using their existing Google or GitHub account instead of creating new credentials. They click "Sign in with Google/GitHub", authorize the application, and are automatically logged in with their social account linked to their profile.

**Why this priority**: Social login significantly reduces friction for new users and provides a better user experience. It's a common expectation in modern web applications and increases conversion rates for sign-ups.

**Independent Test**: Can be fully tested by clicking "Sign in with Google", completing the OAuth flow, and verifying the user is logged in. Delivers quick, passwordless authentication.

**Acceptance Scenarios**:

1. **Given** a new user clicks "Sign in with Google", **When** they complete OAuth authorization, **Then** a new account is created with their Google email and they are logged in
2. **Given** an existing user with email/password account clicks "Sign in with Google" using the same email, **When** they complete OAuth, **Then** their Google account is linked to their existing account
3. **Given** a user logs in with GitHub OAuth, **When** they return later and click "Sign in with GitHub", **Then** they are logged in without re-authorization
4. **Given** a user has both Google and GitHub linked, **When** they log in with either provider, **Then** they access the same account

---

### User Story 3 - Password Reset (Priority: P1)

A user forgot their password and needs to regain access to their account. They click "Forgot Password", enter their email, receive a reset link, click it, set a new password, and successfully log in with the new password.

**Why this priority**: Critical for account recovery. Without password reset, users who forget passwords lose access permanently, leading to poor user experience and support burden.

**Independent Test**: Can be fully tested by requesting a password reset, receiving the email with time-limited token, setting a new password, and logging in. Delivers account recovery capability.

**Acceptance Scenarios**:

1. **Given** a user clicks "Forgot Password", **When** they enter their registered email, **Then** a password reset email is sent with a time-limited token (valid for 1 hour)
2. **Given** a user receives a reset email, **When** they click the link within 1 hour, **Then** they see a form to set a new password
3. **Given** a user sets a new password meeting policy requirements, **When** they submit the form, **Then** their password is updated and old password no longer works
4. **Given** a user clicks a reset link after 1 hour, **When** they try to use it, **Then** they see an error that the link expired and can request a new one
5. **Given** a user requests password reset for non-existent email, **When** they submit, **Then** they see a generic message (don't reveal email doesn't exist for security)

---

### User Story 4 - Role-Based Access Control (Priority: P2)

An administrator needs to control who can ingest documents into the RAG system and who can only read. They assign roles (reader, contributor, admin) to users, and the system automatically enforces access rules: readers can only query, contributors can ingest documents, and admins can manage users.

**Why this priority**: Essential for multi-tenant systems and preventing unauthorized data ingestion. Protects the system from abuse and maintains data quality by limiting who can add content.

**Independent Test**: Can be tested by creating users with different roles, attempting role-specific actions, and verifying access control works. Delivers security and organizational structure.

**Acceptance Scenarios**:

1. **Given** a new user signs up, **When** their account is created, **Then** they are assigned the "reader" role by default
2. **Given** a user with "reader" role, **When** they attempt to access the document ingestion endpoint, **Then** they receive a 403 Forbidden error
3. **Given** a user with "contributor" role, **When** they access the document ingestion endpoint, **Then** they can successfully upload and ingest documents
4. **Given** an admin user in the admin panel, **When** they change another user's role from "reader" to "contributor", **Then** that user immediately gains ingestion permissions
5. **Given** a user with "admin" role, **When** they access the admin panel, **Then** they can view all users, modify roles, and view audit logs

---

### User Story 5 - Session Management & Token Refresh (Priority: P1)

A logged-in user continues working on the platform for several hours. Their access token expires after 15 minutes, but the system automatically refreshes it using their refresh token without interrupting their work. They can also view all active sessions and revoke specific sessions from untrusted devices.

**Why this priority**: Provides seamless user experience while maintaining security through short-lived access tokens. Session management is critical for security and user control over their account access.

**Independent Test**: Can be tested by logging in, waiting for access token expiration, verifying automatic refresh, and manually revoking sessions. Delivers secure, uninterrupted user experience.

**Acceptance Scenarios**:

1. **Given** a user logs in successfully, **When** authentication completes, **Then** they receive both an access token (15-minute expiry) and a refresh token (7-day expiry stored in HttpOnly cookie)
2. **Given** a user's access token expires after 15 minutes, **When** they make an API request, **Then** the system automatically uses the refresh token to issue a new access token
3. **Given** a user navigates to account settings, **When** they view active sessions, **Then** they see a list of all devices/locations with active refresh tokens
4. **Given** a user sees an unfamiliar session, **When** they click "Revoke" on that session, **Then** that refresh token is invalidated and the device must re-authenticate
5. **Given** a user clicks "Logout", **When** the logout completes, **Then** both access and refresh tokens are invalidated

---

### User Story 6 - Account Deletion (GDPR Compliance) (Priority: P3)

A user wants to permanently delete their account and all associated data. They navigate to account settings, click "Delete Account", confirm the deletion, and all their personal data (profile, query history, sessions) is permanently removed from the system.

**Why this priority**: Legal requirement for GDPR and privacy regulations. While not needed for MVP functionality, it's essential for production deployment in regions with privacy laws.

**Independent Test**: Can be tested by creating an account, adding data (queries, profile info), deleting the account, and verifying data is removed. Delivers privacy compliance.

**Acceptance Scenarios**:

1. **Given** a user in account settings clicks "Delete Account", **When** they confirm deletion, **Then** they are prompted to re-enter their password for security
2. **Given** a user confirms account deletion with correct password, **When** deletion processes, **Then** their user record, profile, query logs, and sessions are permanently deleted
3. **Given** an account is deleted, **When** someone tries to sign up with that email again, **Then** they can create a new account (email is freed up)
4. **Given** a user had RAG chat history before deletion, **When** their account is deleted, **Then** their query logs are anonymized or deleted according to retention policy

---

### User Story 7 - Admin User & Role Management (Priority: P2)

An administrator needs to manage user accounts, assign roles, view security audit logs, and respond to security incidents. They access an admin panel showing all users, can search/filter users, change roles, revoke sessions, and view audit logs of login attempts and security events.

**Why this priority**: Essential for managing a multi-user system at scale. Admins need visibility and control to maintain security, respond to issues, and manage access appropriately.

**Independent Test**: Can be tested by logging in as admin, managing user roles, reviewing audit logs, and verifying changes take effect. Delivers administrative control and security oversight.

**Acceptance Scenarios**:

1. **Given** an admin logs into the admin panel, **When** they access the users list, **Then** they see all registered users with their roles and account status
2. **Given** an admin searches for a specific user by email, **When** they enter the email, **Then** the user list filters to show matching users
3. **Given** an admin selects a user, **When** they change the role from "reader" to "contributor", **Then** the role is updated immediately and reflected in the user's permissions
4. **Given** an admin views the audit log, **When** they filter by event type "failed_login", **Then** they see all failed login attempts with timestamps, IP addresses, and user emails
5. **Given** an admin sees suspicious activity (multiple failed logins from same IP), **When** they revoke all sessions for that user, **Then** the user must re-authenticate on all devices

---

### User Story 8 - Anonymous User Graceful Degradation (Priority: P2)

An anonymous (not logged in) user wants to explore the RAG chatbot and read the book content without creating an account. They can perform read-only queries against the book content, but features like saved history, document ingestion, and feedback are disabled with clear prompts to sign up for access.

**Why this priority**: Reduces barriers to entry and allows users to experience value before committing to sign up. Increases conversion by letting users "try before they buy".

**Independent Test**: Can be tested by accessing the site without logging in, asking RAG queries, and verifying read-only access works while authenticated features show appropriate prompts. Delivers frictionless exploration.

**Acceptance Scenarios**:

1. **Given** an anonymous user accesses the chatbot widget, **When** they ask a question about the book content, **Then** they receive an answer from the global RAG system
2. **Given** an anonymous user receives an answer, **When** they look for chat history, **Then** they see a message "Sign in to save your chat history" with a link to login
3. **Given** an anonymous user tries to access document ingestion, **When** they navigate to that feature, **Then** they see "Contributors only - Sign up to request access"
4. **Given** an anonymous user selects text and clicks "Ask AI about this selection", **When** the local RAG query runs, **Then** they get an answer based on selected text (no Qdrant query needed)
5. **Given** an anonymous user wants to provide feedback on a response, **When** they click "Provide Feedback", **Then** they see a prompt to sign in first

---

### Edge Cases

- What happens when a user tries to verify their email with an expired verification token (older than 24 hours)?
  - Show error message: "Verification link expired. Request a new verification email."

- How does the system handle a user attempting to link a Google account that's already linked to another user account?
  - Prevent linking and show error: "This Google account is already associated with another user."

- What happens when a refresh token is used after it has been revoked or rotated?
  - Immediately invalidate all refresh tokens for that user (token theft detection) and require re-authentication.

- How does the system handle OAuth callback failures (user denies permission or network error during OAuth flow)?
  - Redirect to login page with error message: "OAuth authorization failed. Please try again or use email/password."

- What happens when an admin tries to delete their own admin account or change their own role to a lower privilege?
  - Prevent self-role-change with message: "Cannot modify your own role. Ask another admin."
  - Prevent self-deletion if they are the last admin: "Cannot delete the last admin account."

- How does the system handle concurrent login attempts from different devices/locations?
  - Allow concurrent sessions, but log each login with device fingerprint and location for security audit.
  - Show all active sessions in account settings for user review.

- What happens when password reset is requested for an unverified account?
  - Send password reset email (don't reveal verification status for security), but require email verification after password reset.

- How does the system handle rate limiting exceeded scenarios?
  - Return 429 Too Many Requests with Retry-After header
  - For auth endpoints: temporarily block IP after excessive requests (e.g., 10 failed logins in 5 minutes = 15-minute block)

- What happens when a user's session is revoked by an admin while they're actively using the system?
  - Next API request returns 401 Unauthorized, frontend redirects to login with message: "Your session was ended by an administrator."

- How does the system handle database connection failures during authentication?
  - Return 503 Service Unavailable with generic error message
  - Log detailed error server-side for ops team
  - Retry transient errors (connection timeouts) before failing

## Requirements

### Functional Requirements

**Authentication & Account Management**

- **FR-001**: System MUST allow users to create accounts using email and password with minimum password length of 12 characters
- **FR-002**: System MUST send verification emails with signed tokens (valid for 24 hours) upon account creation
- **FR-003**: System MUST hash passwords using bcrypt or argon2 with appropriate salt rounds (minimum cost factor: bcrypt 12, argon2id with recommended parameters)
- **FR-004**: System MUST provide password reset flow with time-limited tokens (valid for 1 hour)
- **FR-005**: System MUST allow users to log in with verified email and password
- **FR-006**: System MUST support OAuth2 social login with Google and GitHub providers
- **FR-007**: System MUST allow account linking when OAuth email matches existing account email
- **FR-008**: System MUST allow users to delete their accounts with password confirmation
- **FR-009**: System MUST enforce password policy: minimum 12 characters, at least one uppercase, one lowercase, one number, one special character

**Session & Token Management**

- **FR-010**: System MUST issue JWT access tokens with 15-minute expiration upon successful authentication
- **FR-011**: System MUST issue refresh tokens with 7-day expiration stored as HttpOnly, Secure, SameSite cookies
- **FR-012**: System MUST automatically refresh access tokens using valid refresh tokens without user interaction
- **FR-013**: System MUST store refresh tokens hashed in database (not plaintext)
- **FR-014**: System MUST implement token rotation: new refresh token issued on each refresh, old one invalidated
- **FR-015**: System MUST provide session listing endpoint showing all active sessions for a user (device, location, last active time)
- **FR-016**: System MUST allow users to revoke individual sessions or all sessions except current
- **FR-017**: System MUST invalidate all refresh tokens when user logs out or changes password
- **FR-018**: System MUST detect token theft: if revoked refresh token is used, invalidate all user's tokens and require re-authentication

**Role-Based Access Control (RBAC)**

- **FR-019**: System MUST support three roles: reader (default), contributor, admin
- **FR-020**: System MUST assign "reader" role to all new user accounts by default
- **FR-021**: System MUST enforce role-based permissions on all protected endpoints (ingestion requires contributor/admin, admin panel requires admin)
- **FR-022**: System MUST allow admins to change user roles via admin panel
- **FR-023**: System MUST prevent users from modifying their own roles
- **FR-024**: System MUST prevent deletion of the last admin account
- **FR-025**: System MUST check permissions before allowing document ingestion, embedding, or admin operations

**Security & Audit**

- **FR-026**: System MUST implement rate limiting on auth endpoints: max 5 login attempts per IP per 5 minutes
- **FR-027**: System MUST lock accounts for 15 minutes after 5 consecutive failed login attempts
- **FR-028**: System MUST log all security events to audit log: login success/failure, password reset, role changes, session revocations
- **FR-029**: System MUST include IP address, user agent, and timestamp in audit logs
- **FR-030**: System MUST NOT log passwords, tokens, or other sensitive credentials
- **FR-031**: System MUST sanitize audit logs to prevent PII leaks (mask partial email, IP)
- **FR-032**: System MUST enforce HTTPS-only cookies (Secure flag) in production
- **FR-033**: System MUST implement CSRF protection for stateful endpoints (cookie-based sessions)
- **FR-034**: System MUST validate JWT signatures and expiration on every protected request
- **FR-035**: System MUST use environment variables for all secrets (JWT secret, OAuth credentials, database credentials)

**Integration with Existing Systems**

- **FR-036**: System MUST protect RAG endpoints (/query-global, /query-local, /ingest, /embed, /feedback) with authentication
- **FR-037**: System MUST allow anonymous users to access /query-global and /query-local in read-only mode (no history saved)
- **FR-038**: System MUST require contributor or admin role for document ingestion endpoints
- **FR-039**: System MUST store user_id with query logs in Neon database when user is authenticated
- **FR-040**: System MUST allow query logs to have nullable user_id for anonymous queries
- **FR-041**: System MUST provide admin endpoints for user management, role assignment, and audit log viewing
- **FR-042**: System MUST integrate with Docusaurus frontend for auth UI components (SignUpForm, LoginForm, AccountSettings)

**Data Management & Privacy**

- **FR-043**: System MUST permanently delete user data upon account deletion: user record, profile, sessions, refresh tokens
- **FR-044**: System MUST anonymize or delete query logs associated with deleted user accounts based on retention policy (default: anonymize by removing user_id)
- **FR-045**: System MUST allow data export for GDPR compliance: users can download their profile data and query history
- **FR-046**: System MUST not send plaintext passwords via email or log them
- **FR-047**: System MUST store OAuth tokens securely in database (encrypted at rest recommended)

**Graceful Degradation & User Experience**

- **FR-048**: System MUST allow Docusaurus book to function as static site when auth service is unavailable (auth-dependent features gracefully disabled)
- **FR-049**: System MUST display clear auth state in UI: anonymous user sees "Sign In" prompt, authenticated user sees their name/avatar
- **FR-050**: System MUST show appropriate messages for auth-dependent features when user is anonymous: "Sign in to save chat history", "Sign in to provide feedback"
- **FR-051**: System MUST redirect users to their intended destination after successful login (return_url parameter)
- **FR-052**: System MUST provide clear error messages for auth failures without revealing security-sensitive information (e.g., don't say "email not found", say "invalid credentials")

### Key Entities

- **User**: Represents a registered user account with email, hashed password, verification status, created timestamp, and last login timestamp. Has one-to-one relationship with UserProfile and one-to-many with RefreshToken, AuditLog, QueryLog, and UserRole.

- **UserProfile**: Extended user information including display name, avatar URL, and metadata JSON (for extensibility). Linked to User.

- **RefreshToken**: Stores hashed refresh tokens with expiration time, revocation status, and device/location fingerprint. Multiple refresh tokens per user (one per active session). Rotated on each refresh.

- **AuditLog**: Security event log capturing user actions like login, logout, password reset, role changes, session revocations. Includes event type, event data (JSON), user_id, IP address, user agent, and timestamp.

- **Role**: Predefined roles in the system (reader, contributor, admin). Has many-to-many relationship with User via UserRole junction table.

- **UserRole**: Junction table linking users to roles, supporting multiple roles per user if needed.

- **QueryLog**: (Existing entity extended) RAG chat query history with optional user_id (nullable for anonymous users), query text, response, timestamp, and feedback rating.

- **Session**: (Optional) If implementing server-side sessions instead of pure JWT, stores session data with user_id, expiration, and session token hash.

## Success Criteria

### Measurable Outcomes

**User Experience**

- **SC-001**: Users can complete account creation (sign-up + email verification + login) in under 3 minutes on average
- **SC-002**: 90% of users successfully log in on their first attempt with correct credentials
- **SC-003**: Password reset flow (request + email + reset + login) completes in under 5 minutes for 95% of users
- **SC-004**: Users can link social accounts (Google/GitHub OAuth) in under 30 seconds
- **SC-005**: Session refresh happens transparently without user-visible delays or interruptions during normal usage

**Security & Performance**

- **SC-006**: Zero plaintext passwords or tokens logged or transmitted over unencrypted channels in production
- **SC-007**: 100% of protected endpoints (ingestion, admin) enforce role-based permissions correctly
- **SC-008**: Brute-force attacks are mitigated: accounts lock after 5 failed attempts, login rate limited to 5 attempts per IP per 5 minutes
- **SC-009**: Token theft detection triggers correctly: using a revoked refresh token invalidates all user sessions
- **SC-010**: Audit logs capture 100% of security-critical events (login, password change, role change, session revoke) with IP, timestamp, and user_id

**Operational & Compliance**

- **SC-011**: Authentication service maintains 99.5% uptime (allows ~3.6 hours downtime per month)
- **SC-012**: Auth endpoint response times: login completes in under 500ms, token refresh in under 200ms (p95)
- **SC-013**: Account deletion permanently removes all user PII within 24 hours (GDPR compliance)
- **SC-014**: Users can export their data (profile + query history) within 1 minute of request
- **SC-015**: System supports 1000 concurrent authenticated users without performance degradation

**Admin & Governance**

- **SC-016**: Admins can search, filter, and modify user roles in admin panel, with changes taking effect immediately (under 1 second)
- **SC-017**: Audit logs are searchable and filterable by event type, user, date range, and IP address
- **SC-018**: 100% of role changes and session revocations appear in audit log within 5 seconds

**Integration & Deployment**

- **SC-019**: Docusaurus frontend integrates seamlessly: auth UI components render correctly in light/dark theme
- **SC-020**: Anonymous users can access RAG read-only features (query-global, query-local) without sign-up
- **SC-021**: System gracefully degrades when auth service is down: static book content remains accessible, auth-dependent features show appropriate "unavailable" messages
- **SC-022**: Database migrations complete successfully with zero data loss during auth system deployment
- **SC-023**: OAuth flow works correctly in both local development (localhost callbacks) and production (HTTPS callbacks)
