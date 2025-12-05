# Implementation Plan: Better Auth — Full Authentication & Authorization

**Feature Branch**: `001-better-auth`
**Created**: 2025-12-05
**Status**: Planning Complete
**Specification**: [spec.md](./spec.md)

---

## Executive Summary

This plan details the complete integration of Better Auth authentication and authorization into the Physical AI & Humanoid Robotics Course ecosystem. The system will secure the RAG chatbot, Docusaurus book reader, and admin dashboard with JWT-based authentication, OAuth2 social login, role-based access control (RBAC), and comprehensive audit logging.

**Core Architecture**:
- **Backend**: FastAPI with Better Auth SDK, JWT access tokens (15-min) + refresh tokens (7-day, HttpOnly cookies)
- **Database**: Neon PostgreSQL (serverless) for users, sessions, roles, audit logs
- **Vector DB**: Qdrant Cloud with per-user metadata filtering for secure RAG
- **Frontend**: Docusaurus 3.x with React auth UI components
- **AI Integration**: OpenAI Agents with authenticated session context

**Key Features**:
- Email/password + OAuth2 (Google, GitHub) authentication
- RBAC with 3 roles: reader (default), contributor (can ingest docs), admin (full access)
- Session management with automatic token refresh
- Anonymous user graceful degradation (read-only RAG access)
- GDPR-compliant account deletion and data export
- Admin panel for user/role management
- Security: rate limiting, brute-force protection, audit logging, CSRF protection

**Timeline**: 3-4 weeks (Research: 1-2 days, Design: 1 day, Implementation: 10-15 days, Testing/Deploy: 3-5 days)

---

## Technical Context

### Technology Stack

**Language/Version**:
- Python 3.11+ (FastAPI backend)
- TypeScript/React 18+ (Docusaurus frontend)
- Node.js 18+ (Docusaurus build system)

**Primary Dependencies**:

*Backend*:
- `FastAPI 0.109+` - Web framework for auth service
- `Better Auth SDK` (Python) - Authentication library (version TBD in research)
- `python-jose[cryptography]` - JWT encoding/decoding
- `passlib[bcrypt]` - Password hashing with bcrypt
- `SQLAlchemy 2.0+` - ORM for database models
- `Alembic` - Database migrations
- `Authlib` - OAuth2 client for Google/GitHub
- `pydantic 2.0+` - Data validation
- `python-multipart` - Form data handling
- `slowapi` or `fastapi-limiter` - Rate limiting (TBD in research)
- `httpx` - Async HTTP client for OAuth2
- `sendgrid` or `aws-ses` or `resend` - Email service (TBD in research)

*Frontend*:
- `@docusaurus/core 3.x` - Static site generator
- `react 18+`, `react-dom 18+` - UI framework
- `axios` or `fetch` - HTTP client for API calls
- `react-hook-form` - Form validation
- `zod` - Schema validation
- `zustand` or `@tanstack/react-query` - State management (TBD in research)

*Database*:
- Neon PostgreSQL (serverless, managed)
- PostgreSQL 15+ compatible

*Infrastructure*:
- Vercel or Render or Fly.io (FastAPI deployment)
- Vercel (Docusaurus static site)
- Qdrant Cloud (vector database)

### Performance Goals

- **Login latency**: < 500ms (p95)
- **Token refresh**: < 200ms (p95)
- **Protected endpoint overhead**: < 50ms (auth check)
- **Concurrent users**: 1000+ without degradation
- **Uptime**: 99.5% (auth service)

### Constraints

- Must maintain backward compatibility with existing RAG endpoints
- Anonymous users must retain read-only access to RAG queries
- No breaking changes to Docusaurus book navigation/reading experience
- Environment-based configuration (dev vs production OAuth callbacks)
- HTTPS-only cookies and tokens in production
- GDPR compliance for EU users (data export + deletion)

### Assumptions

- Neon PostgreSQL database already provisioned or will be created
- Qdrant Cloud instance is accessible with API key
- Domain with HTTPS certificate available for production (OAuth callbacks require HTTPS)
- Email service credentials available (SendGrid/AWS SES/Resend API keys)
- Google Cloud Console and GitHub Apps can be configured for OAuth2
- OpenAI API key already configured for RAG agents

---

## Constitution Check

**Validation Date**: 2025-12-05

This plan has been validated against the project constitution principles:

### ✅ Principle VI: Code & Simulation Standards
- **Gate**: "All code snippets must be syntactically correct and executable"
- **Compliance**: Plan includes testing phase with unit tests for auth flows, integration tests for API endpoints, and E2E tests for complete user journeys. All code will be validated before deployment.
- **Gate**: "No hardcoded secrets, tokens, or credentials"
- **Compliance**: Plan mandates environment variables for all secrets (JWT secret, OAuth credentials, database URLs, API keys). Security section explicitly addresses secret management.

### ✅ Principle VII: Quality Gates & Validation
- **Gate**: "All code tested in target environment before publication"
- **Compliance**: Testing phase includes local dev environment validation, staging deployment, and production smoke tests. Each auth flow will be tested end-to-end.
- **Gate**: "No unverifiable claims"
- **Compliance**: All security claims (rate limiting, token rotation, CSRF protection) will be validated through automated tests and manual security review.

### ✅ Principle V: AI-Native Authoring Workflow
- **Gate**: "Planning precedes generation: no code or chapter produced without a valid spec"
- **Compliance**: This plan follows validated specification (spec.md). Implementation will proceed through Phase 0 (Research) → Phase 1 (Design) → Phase 2 (Tasks) → Implementation.
- **Gate**: "Prompt History Records (PHRs) document all major generation decisions"
- **Compliance**: PHR will be created for this planning phase and subsequent implementation phases.

**Result**: All constitution gates PASSED. Plan is compliant and ready for execution.

---

## Project Structure

```
humanoid-robotics/
├── backend/                          # FastAPI backend (NEW)
│   ├── src/
│   │   ├── __init__.py
│   │   ├── main.py                  # FastAPI app entry point
│   │   ├── config.py                # Settings and environment variables
│   │   ├── auth/                    # Authentication module (NEW)
│   │   │   ├── __init__.py
│   │   │   ├── dependencies.py      # get_current_user, require_role
│   │   │   ├── router.py            # Auth endpoints (/signup, /login, /logout, /refresh)
│   │   │   ├── oauth.py             # OAuth2 providers (Google, GitHub)
│   │   │   ├── jwt.py               # JWT creation/validation
│   │   │   ├── password.py          # Password hashing (bcrypt)
│   │   │   ├── email.py             # Email verification & password reset
│   │   │   └── schemas.py           # Pydantic models for auth
│   │   ├── admin/                   # Admin panel API (NEW)
│   │   │   ├── __init__.py
│   │   │   ├── router.py            # Admin endpoints (/users, /roles, /audit-logs)
│   │   │   └── schemas.py
│   │   ├── rag/                     # RAG endpoints (EXISTING, to be protected)
│   │   │   ├── __init__.py
│   │   │   ├── router.py            # /query-global, /query-local, /ingest, /embed
│   │   │   └── dependencies.py      # RAG-specific auth checks
│   │   ├── models/                  # SQLAlchemy models (NEW)
│   │   │   ├── __init__.py
│   │   │   ├── user.py              # User, UserProfile
│   │   │   ├── session.py           # RefreshToken
│   │   │   ├── role.py              # Role, UserRole
│   │   │   └── audit_log.py         # AuditLog
│   │   ├── database/
│   │   │   ├── __init__.py
│   │   │   ├── session.py           # Database session factory
│   │   │   └── migrations/          # Alembic migrations (NEW)
│   │   │       ├── versions/
│   │   │       └── env.py
│   │   ├── middleware/              # Custom middleware (NEW)
│   │   │   ├── __init__.py
│   │   │   ├── rate_limit.py        # Rate limiting
│   │   │   └── cors.py              # CORS configuration
│   │   └── utils/
│   │       ├── __init__.py
│   │       └── security.py          # Security utilities
│   ├── tests/                       # Backend tests (NEW)
│   │   ├── __init__.py
│   │   ├── conftest.py              # Pytest fixtures
│   │   ├── test_auth.py             # Auth flow tests
│   │   ├── test_rbac.py             # Role-based access control tests
│   │   └── test_integration.py      # Full integration tests
│   ├── alembic.ini                  # Alembic configuration
│   ├── requirements.txt             # Python dependencies
│   └── .env.example                 # Environment variable template
│
├── website/                         # Docusaurus frontend (EXISTING)
│   ├── src/
│   │   ├── components/
│   │   │   ├── Auth/                # Auth UI components (NEW)
│   │   │   │   ├── LoginForm.tsx
│   │   │   │   ├── SignupForm.tsx
│   │   │   │   ├── PasswordReset.tsx
│   │   │   │   ├── AccountSettings.tsx
│   │   │   │   ├── SessionList.tsx
│   │   │   │   └── AuthModal.tsx
│   │   │   ├── RAGChatbot/          # Chatbot widget (EXISTING, to be updated)
│   │   │   │   └── ChatWidget.tsx   # Add auth state check
│   │   │   └── Admin/               # Admin panel components (NEW)
│   │   │       ├── UserList.tsx
│   │   │       ├── RoleManager.tsx
│   │   │       └── AuditLogViewer.tsx
│   │   ├── lib/
│   │   │   └── auth/                # Auth utilities (NEW)
│   │   │       ├── authService.ts   # API calls for auth
│   │   │       ├── useAuth.ts       # Auth hook
│   │   │       └── types.ts         # TypeScript types
│   │   ├── pages/
│   │   │   ├── signin.tsx           # Sign-in page (EXISTING placeholder, to be updated)
│   │   │   ├── signup.tsx           # Sign-up page (EXISTING placeholder, to be updated)
│   │   │   ├── account.tsx          # Account settings page (NEW)
│   │   │   └── admin/               # Admin pages (NEW)
│   │   │       ├── index.tsx
│   │   │       ├── users.tsx
│   │   │       └── audit.tsx
│   │   └── css/
│   │       └── auth.module.css      # Auth component styles (EXISTING, to be enhanced)
│   └── docusaurus.config.ts         # Add auth navbar items
│
├── specs/001-better-auth/           # This feature's documentation
│   ├── spec.md                      # Feature specification (EXISTING)
│   ├── plan.md                      # This implementation plan
│   ├── research.md                  # Phase 0 research findings (TO BE CREATED)
│   ├── data-model.md                # Phase 1 database schema (TO BE CREATED)
│   ├── contracts/                   # Phase 1 API contracts (TO BE CREATED)
│   │   ├── auth-api.yaml
│   │   └── admin-api.yaml
│   └── quickstart.md                # Phase 1 developer guide (TO BE CREATED)
│
└── history/prompts/better-auth/     # Prompt History Records
    ├── 001-better-auth-specification.spec.prompt.md
    └── 002-better-auth-plan.plan.prompt.md
```

---

## Phase 0: Research & Technology Selection

**Duration**: 1-2 days
**Goal**: Answer critical technology choices before detailed design

### Research Task 1: Better Auth SDK Evaluation

**Question**: Which Python authentication library best fits our needs?

**Options to Evaluate**:
1. **FastAPI-Users** (https://fastapi-users.github.io/fastapi-users/)
   - Pros: FastAPI-native, JWT + cookie support, OAuth2, email verification built-in
   - Cons: Opinionated structure, may be overkill for simple use case
2. **Authlib** (https://authlib.org/)
   - Pros: Flexible, OAuth 1.0/2.0/OIDC support, well-maintained
   - Cons: Lower-level, requires more manual setup
3. **Custom implementation** (python-jose + passlib + Authlib for OAuth only)
   - Pros: Full control, minimal dependencies, tailored to our needs
   - Cons: More code to write and maintain, potential security pitfalls

**Evaluation Criteria**:
- OAuth2 provider support (Google, GitHub)
- JWT + refresh token handling
- Session management flexibility
- Database adapter compatibility (SQLAlchemy)
- Email verification workflow support
- Documentation quality and community support
- Bundle size impact on FastAPI app

**Deliverable**: Recommendation with justification in `research.md`

### Research Task 2: OAuth2 Provider Configuration

**Question**: How to configure Google and GitHub OAuth2 for both local dev and production?

**Items to Research**:
1. **Google Cloud Console**:
   - Create OAuth 2.0 client ID (Web application)
   - Authorized redirect URIs: `http://localhost:8000/auth/callback/google` (dev), `https://yourdomain.com/auth/callback/google` (prod)
   - Required scopes: `openid`, `email`, `profile`
   - Client ID and Client Secret storage (environment variables)

2. **GitHub OAuth Apps**:
   - Register new OAuth App (Settings > Developer settings > OAuth Apps)
   - Authorization callback URL: `http://localhost:8000/auth/callback/github` (dev), `https://yourdomain.com/auth/callback/github` (prod)
   - Required scopes: `user:email`
   - Client ID and Client Secret storage

3. **Multi-environment Handling**:
   - How to dynamically set redirect URLs based on environment (dev vs staging vs prod)?
   - Best practices for OAuth state parameter (CSRF protection)
   - Error handling for OAuth failures (user denies permission, network errors)

**Deliverable**: Step-by-step OAuth setup guide in `research.md`

### Research Task 3: Database Schema Design Best Practices

**Question**: What's the optimal PostgreSQL schema for users, sessions, roles, and audit logs?

**Items to Research**:
1. **User Table Design**:
   - UUID vs auto-increment ID for user IDs?
   - Email uniqueness constraint (case-insensitive index?)
   - Password hash storage (bcrypt work factor: 12 vs 14?)
   - Email verification status tracking (boolean vs enum?)
   - Account locking fields (is_locked, locked_until, failed_login_attempts)

2. **Refresh Token Storage**:
   - Store tokens hashed (SHA-256) or encrypted?
   - Token rotation strategy: delete old tokens or mark as revoked?
   - Device fingerprinting: what metadata to store (user-agent, IP, device name)?
   - Index strategy for fast token lookup

3. **Role-Based Access Control**:
   - Roles table with predefined rows (reader, contributor, admin) vs enum?
   - UserRole junction table for many-to-many or foreign key for one-to-many?
   - Permission-based vs role-based (do we need granular permissions or just roles)?

4. **Audit Log Schema**:
   - Event type enum or string?
   - IP address storage: full IP vs masked/anonymized?
   - Event metadata: JSON column for flexibility?
   - Retention policy: TTL or manual cleanup?
   - Indexes for efficient log queries (by user, by event type, by date range)

**Deliverable**: Recommended schema with indexes and constraints in `research.md`

### Research Task 4: JWT Security Best Practices

**Question**: What JWT strategy maximizes security while maintaining usability?

**Items to Research**:
1. **Signing Algorithm**:
   - HS256 (HMAC with SHA-256, symmetric key) vs RS256 (RSA with SHA-256, asymmetric key)?
   - When to use RS256: multiple services validating tokens, public key distribution
   - Secret key rotation strategy for HS256

2. **Token Expiration Strategy**:
   - Access token: 15 minutes (current plan) - is this optimal?
   - Refresh token: 7 days (current plan) - sliding window vs fixed expiration?
   - Refresh token rotation: issue new refresh token on each access token refresh?

3. **Token Storage**:
   - Access token: localStorage vs memory (secure but lost on refresh)?
   - Refresh token: HttpOnly cookie (secure, auto-sent) vs localStorage (XSS risk)?
   - CSRF protection when using cookies (SameSite=Lax vs Strict vs None)

4. **Token Theft Detection**:
   - Refresh token rotation: if revoked token is used, invalidate all user tokens?
   - How to detect and respond to token reuse?

**Deliverable**: JWT security recommendations in `research.md`

### Research Task 5: Rate Limiting & Brute-Force Protection

**Question**: How to implement effective rate limiting for auth endpoints?

**Options to Evaluate**:
1. **SlowAPI** (https://github.com/laurentS/slowapi)
   - Pros: FastAPI-native, decorator-based, memory or Redis backend
   - Cons: Requires Redis for distributed rate limiting

2. **fastapi-limiter** (https://github.com/long2ice/fastapi-limiter)
   - Pros: Redis-based, supports multiple strategies (fixed window, sliding window)
   - Cons: Requires Redis dependency

3. **Custom middleware with in-memory cache**
   - Pros: No external dependencies, simple for single-instance deploys
   - Cons: Not suitable for multi-instance (load-balanced) deployments

**Rate Limiting Strategy**:
- Login endpoint: 5 attempts per IP per 5 minutes
- Password reset: 3 requests per email per hour
- Account creation: 10 signups per IP per hour
- Token refresh: 20 requests per user per minute (generous for client retries)

**Account Lockout**:
- After 5 failed login attempts: lock account for 15 minutes
- Store failed_login_attempts and locked_until in User table
- Reset counter on successful login

**Deliverable**: Rate limiting implementation strategy in `research.md`

### Research Task 6: Qdrant Access Control Integration

**Question**: How to restrict Qdrant collections/points by user role using metadata filters?

**Items to Research**:
1. **Qdrant Metadata Filtering**:
   - Can Qdrant filter points by metadata fields (e.g., `user_id`, `role`, `is_public`)?
   - Payload schema for access control: `{"access": {"roles": ["reader", "contributor", "admin"], "is_public": true}}`
   - Performance impact of metadata filters on query speed

2. **Per-User Embedding Sessions**:
   - How to store "selected text" embeddings with user context?
   - Collection structure: single collection with user_id metadata vs per-user collections?
   - Cleanup strategy for anonymous user temporary embeddings

3. **Qdrant API Key Security**:
   - Store API key in environment variable (backend only, never expose to frontend)
   - Backend acts as proxy: frontend calls FastAPI → FastAPI calls Qdrant with API key
   - Rate limiting on Qdrant queries to prevent abuse

**Deliverable**: Qdrant integration strategy in `research.md`

### Research Task 7: Email Service Integration

**Question**: Which email service provider for verification emails and password resets?

**Options to Evaluate**:
1. **SendGrid** (https://sendgrid.com/)
   - Pros: Free tier (100 emails/day), good deliverability, templates
   - Cons: Requires account verification, credit card for higher tiers

2. **AWS SES** (https://aws.amazon.com/ses/)
   - Pros: Low cost ($0.10 per 1000 emails), high reliability
   - Cons: Requires AWS account, initial sandbox mode (need to request production access)

3. **Resend** (https://resend.com/)
   - Pros: Developer-friendly API, generous free tier (3000 emails/month), React Email support
   - Cons: Newer service, smaller community

**Email Templates Needed**:
- Email verification: "Click to verify your email"
- Password reset: "Click to reset your password (expires in 1 hour)"
- Account locked: "Your account was locked due to failed login attempts"
- Role changed: "Your role was updated by an admin"

**Evaluation Criteria**:
- Free tier limits
- Deliverability reputation (inbox vs spam folder)
- API complexity
- Template support
- SMTP vs API integration

**Deliverable**: Email service recommendation with setup guide in `research.md`

### Research Task 8: Frontend Auth State Management

**Question**: How to manage auth state in Docusaurus/React frontend?

**Options to Evaluate**:
1. **React Context API** (built-in)
   - Pros: No extra dependencies, simple for app-wide state
   - Cons: Can cause unnecessary re-renders, no persistence

2. **Zustand** (https://github.com/pmndrs/zustand)
   - Pros: Lightweight (1KB), minimal boilerplate, persistence middleware
   - Cons: Another dependency

3. **TanStack Query (React Query)** (https://tanstack.com/query)
   - Pros: Server state caching, automatic refetching, optimistic updates
   - Cons: Heavier dependency, learning curve

**Auth State to Manage**:
- Current user (id, email, name, role)
- Authentication status (loading, authenticated, unauthenticated)
- Access token (in memory, refreshed automatically)
- Refresh token (in HttpOnly cookie, managed by backend)

**Implementation Considerations**:
- Persist auth state across page refreshes (call `/auth/me` endpoint on mount)
- Auto-refresh access token before expiration (background timer)
- Logout on 401 errors (token expired, session revoked)
- Optimistic UI updates for better UX

**Deliverable**: Frontend state management recommendation in `research.md`

---

## Phase 1: Design Artifacts

**Duration**: 1 day
**Goal**: Create detailed design documents based on Phase 0 research findings

### Artifact 1: Database Schema (`data-model.md`)

**Contents**:
- Complete SQL DDL (CREATE TABLE statements) for all entities
- Entity-Relationship Diagram (ERD)
- Indexes for performance optimization
- Constraints (foreign keys, unique, not null, check)
- Sample data for testing

**Entities** (based on spec.md):

#### 1. User
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Unique user identifier |
| email | VARCHAR(255) | UNIQUE, NOT NULL, INDEX | User email (stored lowercase) |
| hashed_password | VARCHAR(255) | NULLABLE | Bcrypt hash (null for OAuth-only users) |
| is_verified | BOOLEAN | DEFAULT FALSE | Email verification status |
| is_locked | BOOLEAN | DEFAULT FALSE | Account temporarily locked (brute-force) |
| failed_login_attempts | INTEGER | DEFAULT 0 | Consecutive failed login counter |
| locked_until | TIMESTAMP | NULLABLE | When account lock expires |
| created_at | TIMESTAMP | DEFAULT NOW() | Account creation timestamp |
| updated_at | TIMESTAMP | DEFAULT NOW(), ON UPDATE NOW() | Last update timestamp |
| last_login_at | TIMESTAMP | NULLABLE | Last successful login |

**Indexes**:
- `CREATE UNIQUE INDEX idx_user_email ON user (LOWER(email));` (case-insensitive email)
- `CREATE INDEX idx_user_locked_until ON user (locked_until) WHERE is_locked = TRUE;` (for cleanup job)

#### 2. UserProfile
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Profile identifier |
| user_id | UUID | FOREIGN KEY → user.id, UNIQUE, NOT NULL | One-to-one with User |
| display_name | VARCHAR(100) | NULLABLE | User's display name |
| avatar_url | TEXT | NULLABLE | Profile picture URL |
| metadata | JSONB | DEFAULT '{}' | Extensible metadata (preferences, etc.) |

#### 3. RefreshToken
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Token identifier |
| user_id | UUID | FOREIGN KEY → user.id, NOT NULL, INDEX | Token owner |
| token_hash | VARCHAR(64) | UNIQUE, NOT NULL, INDEX | SHA-256 hash of refresh token |
| expires_at | TIMESTAMP | NOT NULL | Token expiration (7 days from issue) |
| revoked_at | TIMESTAMP | NULLABLE | Revocation timestamp (null = active) |
| device_info | TEXT | NULLABLE | User-Agent string |
| ip_address | VARCHAR(45) | NULLABLE | IP address (IPv4/IPv6, anonymized) |
| created_at | TIMESTAMP | DEFAULT NOW() | Token creation timestamp |

**Indexes**:
- `CREATE INDEX idx_refresh_token_user ON refresh_token (user_id) WHERE revoked_at IS NULL;` (active tokens per user)
- `CREATE INDEX idx_refresh_token_expires ON refresh_token (expires_at) WHERE revoked_at IS NULL;` (cleanup job)

#### 4. Role
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | SERIAL | PRIMARY KEY | Role identifier |
| name | VARCHAR(50) | UNIQUE, NOT NULL | Role name (reader, contributor, admin) |
| description | TEXT | NULLABLE | Role description |

**Seed Data**:
```sql
INSERT INTO role (name, description) VALUES
  ('reader', 'Can query RAG system and read book content'),
  ('contributor', 'Can ingest documents and manage embeddings'),
  ('admin', 'Full access to admin panel and user management');
```

#### 5. UserRole (Junction Table)
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Assignment identifier |
| user_id | UUID | FOREIGN KEY → user.id, NOT NULL | User receiving role |
| role_id | INTEGER | FOREIGN KEY → role.id, NOT NULL | Role being assigned |
| assigned_at | TIMESTAMP | DEFAULT NOW() | When role was assigned |
| assigned_by | UUID | FOREIGN KEY → user.id, NULLABLE | Admin who assigned (null = system) |

**Constraints**:
- `UNIQUE (user_id, role_id)` - User can't have same role twice

**Indexes**:
- `CREATE INDEX idx_user_role_user ON user_role (user_id);` (roles for a user)
- `CREATE INDEX idx_user_role_role ON user_role (role_id);` (users with a role)

#### 6. AuditLog
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Log entry identifier |
| user_id | UUID | FOREIGN KEY → user.id, NULLABLE, INDEX | User who triggered event (null = system) |
| event_type | VARCHAR(50) | NOT NULL, INDEX | Event type (login_success, login_failed, password_reset, role_changed, etc.) |
| event_data | JSONB | DEFAULT '{}' | Event-specific metadata |
| ip_address | VARCHAR(45) | NULLABLE | Anonymized IP address |
| user_agent | TEXT | NULLABLE | User-Agent string |
| created_at | TIMESTAMP | DEFAULT NOW(), INDEX | Event timestamp |

**Indexes**:
- `CREATE INDEX idx_audit_log_event_type ON audit_log (event_type, created_at DESC);`
- `CREATE INDEX idx_audit_log_user ON audit_log (user_id, created_at DESC);`
- `CREATE INDEX idx_audit_log_created ON audit_log (created_at DESC);` (for pagination)

#### 7. QueryLog (Existing, Extended)
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Query identifier |
| user_id | UUID | FOREIGN KEY → user.id, NULLABLE, INDEX | User who queried (null = anonymous) |
| query_text | TEXT | NOT NULL | User's query |
| response_text | TEXT | NOT NULL | RAG response |
| query_type | VARCHAR(20) | NOT NULL | global, local, selected_text |
| feedback_rating | INTEGER | CHECK (feedback_rating BETWEEN 1 AND 5), NULLABLE | User feedback (1-5 stars) |
| created_at | TIMESTAMP | DEFAULT NOW(), INDEX | Query timestamp |

#### 8. PasswordResetToken
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Token identifier |
| user_id | UUID | FOREIGN KEY → user.id, NOT NULL | User requesting reset |
| token_hash | VARCHAR(64) | UNIQUE, NOT NULL, INDEX | SHA-256 hash of reset token |
| expires_at | TIMESTAMP | NOT NULL | Token expiration (1 hour from issue) |
| used_at | TIMESTAMP | NULLABLE | When token was used (null = unused) |
| created_at | TIMESTAMP | DEFAULT NOW() | Token creation timestamp |

**Indexes**:
- `CREATE INDEX idx_password_reset_expires ON password_reset_token (expires_at) WHERE used_at IS NULL;`

#### 9. EmailVerificationToken
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT uuid_generate_v4() | Token identifier |
| user_id | UUID | FOREIGN KEY → user.id, NOT NULL | User to verify |
| token_hash | VARCHAR(64) | UNIQUE, NOT NULL, INDEX | SHA-256 hash of verification token |
| expires_at | TIMESTAMP | NOT NULL | Token expiration (24 hours from issue) |
| verified_at | TIMESTAMP | NULLABLE | When email was verified (null = unverified) |
| created_at | TIMESTAMP | DEFAULT NOW() | Token creation timestamp |

**Indexes**:
- `CREATE INDEX idx_email_verification_expires ON email_verification_token (expires_at) WHERE verified_at IS NULL;`

### Artifact 2: API Contracts (`contracts/auth-api.yaml`)

**Format**: OpenAPI 3.0 specification

**Endpoints to Define**:

#### Authentication Endpoints
- `POST /auth/signup` - Email/password registration
  - Request: `{ email, password }`
  - Response: `{ user_id, message: "Verification email sent" }`
  - Errors: 400 (invalid input), 409 (email exists)

- `POST /auth/login` - Email/password login
  - Request: `{ email, password }`
  - Response: `{ access_token, user: { id, email, role }, message: "Login successful" }`
  - Sets HttpOnly cookie with refresh token
  - Errors: 400 (invalid input), 401 (invalid credentials), 423 (account locked)

- `POST /auth/logout` - Invalidate tokens
  - Requires: Authorization header with access token
  - Response: `{ message: "Logged out successfully" }`
  - Clears refresh token cookie

- `POST /auth/refresh` - Refresh access token
  - Requires: Refresh token in HttpOnly cookie
  - Response: `{ access_token }`
  - Rotates refresh token (sets new cookie)
  - Errors: 401 (invalid/expired token)

#### OAuth2 Endpoints
- `GET /auth/oauth/google` - Initiate Google OAuth flow
  - Redirects to Google consent screen
  - Query params: `?return_url=/dashboard` (optional)

- `GET /auth/callback/google` - Google OAuth callback
  - Query params: `code`, `state`
  - Response: Redirect to frontend with auth success/failure
  - Sets refresh token cookie

- `GET /auth/oauth/github` - Initiate GitHub OAuth flow
- `GET /auth/callback/github` - GitHub OAuth callback

#### Account Management Endpoints
- `POST /auth/verify-email` - Verify email with token
  - Request: `{ token }`
  - Response: `{ message: "Email verified successfully" }`
  - Errors: 400 (invalid token), 404 (token not found), 410 (token expired)

- `POST /auth/request-password-reset` - Request password reset
  - Request: `{ email }`
  - Response: `{ message: "If email exists, reset link sent" }` (generic for security)

- `POST /auth/reset-password` - Reset password with token
  - Request: `{ token, new_password }`
  - Response: `{ message: "Password reset successfully" }`
  - Errors: 400 (invalid token/password), 410 (token expired/used)

- `GET /auth/me` - Get current user
  - Requires: Authorization header with access token
  - Response: `{ id, email, display_name, avatar_url, roles: ["reader"] }`
  - Errors: 401 (unauthenticated)

- `GET /auth/sessions` - List active sessions
  - Requires: Authentication
  - Response: `[ { id, device_info, ip_address, created_at, last_active } ]`

- `DELETE /auth/sessions/{session_id}` - Revoke specific session
  - Requires: Authentication
  - Response: `{ message: "Session revoked" }`

- `DELETE /auth/sessions` - Revoke all sessions except current
  - Requires: Authentication
  - Response: `{ message: "All sessions revoked" }`

- `DELETE /auth/account` - Delete account
  - Requires: Authentication + password confirmation
  - Request: `{ password }`
  - Response: `{ message: "Account deleted" }`
  - Errors: 401 (wrong password)

#### Admin Endpoints (`contracts/admin-api.yaml`)
- `GET /admin/users` - List all users
  - Requires: admin role
  - Query params: `?page=1&limit=50&search=email@example.com&role=reader`
  - Response: `{ users: [...], total, page, limit }`

- `GET /admin/users/{user_id}` - Get user details
  - Requires: admin role
  - Response: User object with roles, sessions, audit logs

- `PATCH /admin/users/{user_id}/role` - Change user role
  - Requires: admin role
  - Request: `{ role: "contributor" }`
  - Response: `{ message: "Role updated" }`
  - Creates audit log entry

- `DELETE /admin/users/{user_id}/sessions` - Revoke all user sessions
  - Requires: admin role
  - Response: `{ message: "All sessions revoked" }`

- `GET /admin/audit-logs` - Query audit logs
  - Requires: admin role
  - Query params: `?page=1&limit=100&event_type=login_failed&user_id={uuid}&start_date=2025-01-01&end_date=2025-12-31`
  - Response: `{ logs: [...], total, page, limit }`

### Artifact 3: Developer Quickstart (`quickstart.md`)

**Contents**:

```markdown
# Better Auth Quickstart Guide

## Prerequisites
- Python 3.11+
- Node.js 18+
- PostgreSQL 15+ (Neon account or local Postgres)
- Qdrant Cloud account with API key
- Email service account (SendGrid/AWS SES/Resend)
- Google Cloud Console project (for OAuth)
- GitHub OAuth App registered

## Backend Setup

### 1. Install Dependencies
```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Configure Environment Variables
Create `backend/.env`:
```env
# Database
DATABASE_URL=postgresql://user:password@neon-host.neon.tech/dbname

# JWT
JWT_SECRET=your-256-bit-secret-key-here  # Generate: openssl rand -hex 32
JWT_ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=15
REFRESH_TOKEN_EXPIRE_DAYS=7

# OAuth2 - Google
GOOGLE_CLIENT_ID=your-google-client-id.apps.googleusercontent.com
GOOGLE_CLIENT_SECRET=your-google-client-secret
GOOGLE_REDIRECT_URI=http://localhost:8000/auth/callback/google

# OAuth2 - GitHub
GITHUB_CLIENT_ID=your-github-client-id
GITHUB_CLIENT_SECRET=your-github-client-secret
GITHUB_REDIRECT_URI=http://localhost:8000/auth/callback/github

# Email Service (SendGrid example)
SENDGRID_API_KEY=SG.your-sendgrid-api-key
FROM_EMAIL=noreply@yourdomain.com

# Qdrant
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key

# OpenAI
OPENAI_API_KEY=sk-your-openai-api-key

# CORS
CORS_ORIGINS=http://localhost:3000,http://localhost:3001  # Docusaurus dev server

# Environment
ENVIRONMENT=development  # or production
```

### 3. Run Database Migrations
```bash
cd backend
alembic upgrade head
```

### 4. Seed Roles
```bash
python -m src.scripts.seed_roles
```

### 5. Start Backend Server
```bash
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

Backend running at: http://localhost:8000
API docs: http://localhost:8000/docs

## Frontend Setup

### 1. Install Dependencies
```bash
cd website
npm install
```

### 2. Configure Environment Variables
Create `website/.env.local`:
```env
NEXT_PUBLIC_API_URL=http://localhost:8000
```

### 3. Start Development Server
```bash
npm start
```

Frontend running at: http://localhost:3000

## OAuth2 Configuration

### Google Cloud Console
1. Go to https://console.cloud.google.com/
2. Create new project or select existing
3. Navigate to "APIs & Services" > "Credentials"
4. Click "Create Credentials" > "OAuth client ID"
5. Application type: "Web application"
6. Authorized redirect URIs:
   - `http://localhost:8000/auth/callback/google` (development)
   - `https://yourdomain.com/auth/callback/google` (production)
7. Copy Client ID and Client Secret to backend/.env

### GitHub OAuth App
1. Go to https://github.com/settings/developers
2. Click "New OAuth App"
3. Application name: "Physical AI & Humanoid Robotics"
4. Homepage URL: `http://localhost:3000` (dev) or `https://yourdomain.com` (prod)
5. Authorization callback URL:
   - `http://localhost:8000/auth/callback/github` (development)
   - `https://yourdomain.com/auth/callback/github` (production)
6. Copy Client ID and Client Secret to backend/.env

## Testing Auth Flows

### Email/Password Sign-Up
```bash
curl -X POST http://localhost:8000/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "SecurePassword123!"}'
```

Check email for verification link, then:
```bash
curl -X POST http://localhost:8000/auth/verify-email \
  -H "Content-Type: application/json" \
  -d '{"token": "verification-token-from-email"}'
```

### Login
```bash
curl -X POST http://localhost:8000/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "test@example.com", "password": "SecurePassword123!"}' \
  -c cookies.txt  # Save refresh token cookie
```

Response includes `access_token`.

### Access Protected Endpoint
```bash
curl -X GET http://localhost:8000/auth/me \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN"
```

### Token Refresh
```bash
curl -X POST http://localhost:8000/auth/refresh \
  -b cookies.txt  # Send refresh token cookie
```

## Database Schema Inspection
```bash
# Connect to Neon database
psql "postgresql://user:password@neon-host.neon.tech/dbname"

# List tables
\dt

# Describe user table
\d user

# Query users
SELECT id, email, is_verified, created_at FROM "user";
```

## Troubleshooting

### Common Issues

**Issue**: "ModuleNotFoundError: No module named 'src'"
**Fix**: Ensure you're in `backend/` directory and venv is activated

**Issue**: "Invalid signature" error on JWT validation
**Fix**: Check JWT_SECRET matches between token creation and validation

**Issue**: OAuth redirect URI mismatch
**Fix**: Ensure redirect URI in Google/GitHub console exactly matches backend/.env

**Issue**: CORS errors in frontend
**Fix**: Add frontend URL to CORS_ORIGINS in backend/.env

**Issue**: Refresh token not being sent
**Fix**: Ensure frontend and backend are on same domain OR backend sets SameSite=None with HTTPS
```

---

## Phase 2: Architecture & Design Decisions

### 1. Authentication Flow Design

**User Sign-Up Flow**:
```
User → Frontend (SignupForm)
  → POST /auth/signup { email, password }
    → Backend validates input (password policy, email format)
    → Hash password with bcrypt (cost factor: 12)
    → Create User record (is_verified=false)
    → Generate email verification token (random 32 bytes, SHA-256 hash stored)
    → Send verification email via SendGrid/SES/Resend
  → Response: { user_id, message: "Check your email" }

User clicks email link → GET /auth/verify-email?token=...
  → Backend validates token (not expired, not used)
  → Update User.is_verified = true
  → Redirect to frontend /signin with success message
```

**User Login Flow (Email/Password)**:
```
User → Frontend (LoginForm)
  → POST /auth/login { email, password }
    → Backend finds User by email (case-insensitive)
    → Check if account is locked (locked_until > NOW)
      → If locked: return 423 Locked
    → Verify password with bcrypt
      → If invalid:
        → Increment failed_login_attempts
        → If failed_login_attempts >= 5: lock account (locked_until = NOW + 15 minutes)
        → Log audit event: login_failed
        → Return 401 Unauthorized
      → If valid:
        → Reset failed_login_attempts = 0
        → Update last_login_at = NOW
        → Generate JWT access token (exp: 15 min)
        → Generate refresh token (random 32 bytes)
        → Store RefreshToken (token_hash=SHA-256, expires_at=NOW + 7 days)
        → Set HttpOnly cookie: refresh_token
        → Log audit event: login_success
        → Return { access_token, user: { id, email, role } }
```

**OAuth2 Login Flow (Google Example)**:
```
User clicks "Sign in with Google" → Frontend
  → GET /auth/oauth/google?return_url=/dashboard
    → Backend generates OAuth state (CSRF protection)
    → Redirect to Google consent screen with client_id, redirect_uri, scope, state

User authorizes → Google redirects to /auth/callback/google?code=...&state=...
  → Backend validates state (CSRF check)
  → Exchange code for Google access token
  → Fetch user profile from Google (email, name, picture)
  → Check if User with email exists
    → If exists: log in (issue JWT + refresh token)
    → If not exists: create User (is_verified=true, hashed_password=null)
  → Set refresh_token cookie
  → Redirect to frontend return_url with success
```

**Token Refresh Flow**:
```
Frontend detects access token expiring soon (< 2 min remaining)
  → POST /auth/refresh (sends refresh_token cookie automatically)
    → Backend reads refresh_token from cookie
    → Hash token with SHA-256, lookup RefreshToken by token_hash
    → Validate:
      → Token exists and not revoked (revoked_at IS NULL)
      → Token not expired (expires_at > NOW)
      → If invalid: return 401 (frontend redirects to login)
    → Generate new access token (exp: 15 min)
    → Rotate refresh token:
      → Mark old RefreshToken as revoked (revoked_at = NOW)
      → Generate new refresh token
      → Store new RefreshToken
      → Set new refresh_token cookie
    → Return { access_token }
```

**Logout Flow**:
```
User clicks "Logout" → Frontend
  → POST /auth/logout (sends access token + refresh token cookie)
    → Backend:
      → Revoke current RefreshToken (revoked_at = NOW)
      → Clear refresh_token cookie (Set-Cookie with max-age=0)
      → Log audit event: logout
    → Response: { message: "Logged out" }
  → Frontend clears access token from memory
  → Redirect to /signin
```

### 2. Role-Based Access Control (RBAC) Architecture

**Role Hierarchy**:
```
reader (default)
  ↓ can do
  - Query RAG (global, local, selected text)
  - Read book content
  - View own profile
  - View own query history

contributor (explicitly assigned)
  ↓ can do (all of reader +)
  - Ingest documents
  - Upload embeddings to Qdrant
  - Manage own document collections

admin (explicitly assigned)
  ↓ can do (all of contributor +)
  - View all users
  - Assign/remove roles
  - View audit logs
  - Revoke user sessions
  - Delete users (except last admin)
```

**Permission Enforcement (FastAPI Dependency Injection)**:
```python
# backend/src/auth/dependencies.py
from fastapi import Depends, HTTPException, status
from jose import jwt, JWTError

async def get_current_user(
    authorization: str = Header(...),
    db: Session = Depends(get_db)
) -> User:
    """Validate JWT and return current user."""
    try:
        token = authorization.replace("Bearer ", "")
        payload = jwt.decode(token, JWT_SECRET, algorithms=[JWT_ALGORITHM])
        user_id = payload.get("sub")
        if not user_id:
            raise HTTPException(status_code=401, detail="Invalid token")
        user = db.query(User).filter(User.id == user_id).first()
        if not user:
            raise HTTPException(status_code=401, detail="User not found")
        return user
    except JWTError:
        raise HTTPException(status_code=401, detail="Invalid token")

async def require_role(required_role: str):
    """Dependency factory for role-based access control."""
    async def role_checker(current_user: User = Depends(get_current_user)):
        user_roles = [ur.role.name for ur in current_user.user_roles]
        if required_role not in user_roles:
            raise HTTPException(status_code=403, detail=f"Requires {required_role} role")
        return current_user
    return role_checker

# Usage in routes:
@router.post("/ingest")
async def ingest_document(
    file: UploadFile,
    current_user: User = Depends(require_role("contributor"))
):
    # Only contributors and admins can reach here
    ...

@router.get("/admin/users")
async def list_users(
    current_user: User = Depends(require_role("admin"))
):
    # Only admins can reach here
    ...
```

### 3. Security Architecture

**Password Security**:
- Hashing: bcrypt with cost factor 12 (2^12 = 4096 iterations)
- Password policy enforced: min 12 chars, uppercase, lowercase, number, special char
- Never log or transmit plaintext passwords
- Password reset invalidates all refresh tokens (force re-login)

**Token Security**:
- Access token: JWT, HS256 signed, 15-minute expiration, stored in memory (frontend)
- Refresh token: Random 32-byte string, SHA-256 hash stored in DB, HttpOnly cookie
- Token rotation: New refresh token issued on each refresh, old one revoked
- Token theft detection: If revoked token is used, invalidate ALL user tokens

**Cookie Security (Production)**:
```python
# backend/src/auth/router.py
response.set_cookie(
    key="refresh_token",
    value=refresh_token,
    httponly=True,      # Not accessible to JavaScript (XSS protection)
    secure=True,        # HTTPS-only (production)
    samesite="lax",     # CSRF protection (or "strict" for max security)
    max_age=7*24*60*60, # 7 days in seconds
    path="/auth"        # Only sent to /auth endpoints
)
```

**CSRF Protection**:
- For cookie-based refresh tokens: SameSite=Lax prevents cross-site requests
- For OAuth state parameter: Random state validated on callback

**Rate Limiting**:
```python
# backend/src/middleware/rate_limit.py
from slowapi import Limiter
from slowapi.util import get_remote_address

limiter = Limiter(key_func=get_remote_address)

@router.post("/login")
@limiter.limit("5/5minutes")  # Max 5 login attempts per IP per 5 minutes
async def login(request: Request, ...):
    ...
```

**Audit Logging**:
- Log all security events: login (success/failure), password reset, role changes, session revocations
- Anonymize IP addresses (mask last octet: `192.168.1.XXX`)
- Never log passwords, tokens, or sensitive data
- Retention: audit logs kept for 90 days (configurable)

### 4. Qdrant Integration Architecture

**Access Control Strategy**:
```python
# backend/src/rag/dependencies.py
from qdrant_client import QdrantClient

async def get_qdrant_filter(current_user: User | None):
    """Generate Qdrant metadata filter based on user role."""
    if not current_user:
        # Anonymous user: only public content
        return Filter(
            must=[FieldCondition(key="access.is_public", match=MatchValue(value=True))]
        )

    user_roles = [ur.role.name for ur in current_user.user_roles]

    # Admin and contributor: access everything
    if "admin" in user_roles or "contributor" in user_roles:
        return None  # No filter

    # Reader: public content + content assigned to their role
    return Filter(
        should=[
            FieldCondition(key="access.is_public", match=MatchValue(value=True)),
            FieldCondition(key="access.roles", match=MatchAny(any=user_roles))
        ]
    )

@router.post("/query-global")
async def query_global(
    query: str,
    current_user: User | None = Depends(get_current_user_optional),
    db: Session = Depends(get_db)
):
    """Global RAG query with role-based access control."""
    filter = await get_qdrant_filter(current_user)

    # Query Qdrant with metadata filter
    results = qdrant_client.search(
        collection_name="book_content",
        query_vector=embed(query),
        query_filter=filter,
        limit=5
    )

    # Log query if user is authenticated
    if current_user:
        db.add(QueryLog(
            user_id=current_user.id,
            query_text=query,
            response_text=generate_response(results),
            query_type="global"
        ))
        db.commit()

    return {"response": generate_response(results)}
```

**Qdrant Point Payload Structure**:
```json
{
  "id": "uuid-v4",
  "vector": [0.1, 0.2, ...],
  "payload": {
    "text": "Content chunk from book chapter...",
    "chapter": "module1/chapter1-introduction",
    "page": 5,
    "access": {
      "is_public": true,
      "roles": ["reader", "contributor", "admin"],
      "created_by": "admin-user-id"
    },
    "metadata": {
      "source": "docusaurus-docs",
      "version": "1.0"
    }
  }
}
```

### 5. Frontend Auth State Management

**Recommendation**: Use **Zustand** for lightweight state management with persistence

**Auth Store (`website/src/lib/auth/authStore.ts`)**:
```typescript
import create from 'zustand';
import { persist } from 'zustand/middleware';

interface User {
  id: string;
  email: string;
  displayName: string;
  avatarUrl?: string;
  roles: string[];
}

interface AuthState {
  user: User | null;
  accessToken: string | null;
  isAuthenticated: boolean;
  isLoading: boolean;

  // Actions
  setUser: (user: User, accessToken: string) => void;
  logout: () => void;
  refreshToken: () => Promise<void>;
}

export const useAuthStore = create<AuthState>()(
  persist(
    (set, get) => ({
      user: null,
      accessToken: null,
      isAuthenticated: false,
      isLoading: true,

      setUser: (user, accessToken) => set({
        user,
        accessToken,
        isAuthenticated: true,
        isLoading: false
      }),

      logout: async () => {
        await fetch('http://localhost:8000/auth/logout', {
          method: 'POST',
          credentials: 'include', // Send refresh token cookie
          headers: {
            'Authorization': `Bearer ${get().accessToken}`
          }
        });
        set({ user: null, accessToken: null, isAuthenticated: false });
      },

      refreshToken: async () => {
        try {
          const response = await fetch('http://localhost:8000/auth/refresh', {
            method: 'POST',
            credentials: 'include' // Send refresh token cookie
          });
          if (response.ok) {
            const { access_token } = await response.json();
            set({ accessToken: access_token });
          } else {
            // Refresh failed, logout
            get().logout();
          }
        } catch (error) {
          console.error('Token refresh failed:', error);
          get().logout();
        }
      }
    }),
    {
      name: 'auth-storage',
      partialize: (state) => ({ user: state.user }) // Only persist user, not token
    }
  )
);
```

**Auto-Refresh Hook (`website/src/lib/auth/useTokenRefresh.ts`)**:
```typescript
import { useEffect } from 'react';
import { useAuthStore } from './authStore';
import { jwtDecode } from 'jwt-decode';

export function useTokenRefresh() {
  const { accessToken, refreshToken, isAuthenticated } = useAuthStore();

  useEffect(() => {
    if (!isAuthenticated || !accessToken) return;

    // Decode token to get expiration
    const decoded = jwtDecode(accessToken);
    const expiresAt = decoded.exp * 1000; // Convert to milliseconds
    const now = Date.now();

    // Refresh 2 minutes before expiration
    const refreshTime = expiresAt - now - (2 * 60 * 1000);

    if (refreshTime > 0) {
      const timeout = setTimeout(() => {
        refreshToken();
      }, refreshTime);

      return () => clearTimeout(timeout);
    } else {
      // Token already expired or expiring soon, refresh immediately
      refreshToken();
    }
  }, [accessToken, isAuthenticated]);
}
```

**Protected Component Example**:
```typescript
// website/src/components/ProtectedRoute.tsx
import { useAuthStore } from '@site/src/lib/auth/authStore';
import { useTokenRefresh } from '@site/src/lib/auth/useTokenRefresh';
import React from 'react';

export function ProtectedRoute({ children, requireRole }: { children: React.ReactNode, requireRole?: string }) {
  const { isAuthenticated, user, isLoading } = useAuthStore();
  useTokenRefresh(); // Auto-refresh tokens

  if (isLoading) {
    return <div>Loading...</div>;
  }

  if (!isAuthenticated) {
    return <div>Please <a href="/signin">sign in</a> to access this page.</div>;
  }

  if (requireRole && !user.roles.includes(requireRole)) {
    return <div>You don't have permission to access this page. Requires {requireRole} role.</div>;
  }

  return <>{children}</>;
}
```

---

## Phase 3: Implementation Phases

### Phase 3A: Backend Core Auth (5-7 days)

**Sprint 1: Database Setup (Day 1)**
- [ ] Create Alembic migration for User, UserProfile, RefreshToken, Role, UserRole tables
- [ ] Create seed script for Role table (reader, contributor, admin)
- [ ] Run migrations on Neon database
- [ ] Write database utility functions (get_db dependency)

**Sprint 2: Password Authentication (Days 2-3)**
- [ ] Implement JWT creation/validation (`src/auth/jwt.py`)
- [ ] Implement password hashing with bcrypt (`src/auth/password.py`)
- [ ] Create Pydantic schemas for auth requests/responses (`src/auth/schemas.py`)
- [ ] Implement POST /auth/signup endpoint (create user, send verification email)
- [ ] Implement POST /auth/login endpoint (validate credentials, issue tokens)
- [ ] Implement POST /auth/logout endpoint (revoke refresh token)
- [ ] Implement POST /auth/refresh endpoint (rotate refresh token)
- [ ] Add rate limiting to auth endpoints (SlowAPI)
- [ ] Unit tests for password hashing, JWT creation/validation
- [ ] Integration tests for signup/login/logout flows

**Sprint 3: Email Verification & Password Reset (Day 4)**
- [ ] Integrate email service (SendGrid/AWS SES/Resend)
- [ ] Create email templates (verification, password reset)
- [ ] Create EmailVerificationToken and PasswordResetToken tables (migration)
- [ ] Implement POST /auth/verify-email endpoint
- [ ] Implement POST /auth/request-password-reset endpoint
- [ ] Implement POST /auth/reset-password endpoint
- [ ] Unit tests for email sending, token validation
- [ ] Integration tests for verification/reset flows

**Sprint 4: OAuth2 Integration (Days 5-6)**
- [ ] Configure Google OAuth client with Authlib
- [ ] Implement GET /auth/oauth/google endpoint (redirect to Google)
- [ ] Implement GET /auth/callback/google endpoint (exchange code, create/login user)
- [ ] Configure GitHub OAuth client
- [ ] Implement GET /auth/oauth/github and callback endpoints
- [ ] Handle account linking (OAuth email matches existing user)
- [ ] Unit tests for OAuth flow logic
- [ ] Integration tests with mock OAuth providers

**Sprint 5: Session Management (Day 7)**
- [ ] Implement GET /auth/me endpoint (get current user)
- [ ] Implement GET /auth/sessions endpoint (list active sessions)
- [ ] Implement DELETE /auth/sessions/{id} endpoint (revoke specific session)
- [ ] Implement DELETE /auth/sessions endpoint (revoke all except current)
- [ ] Implement token theft detection (revoked token reuse)
- [ ] Unit tests for session management
- [ ] Integration tests for multi-device scenarios

### Phase 3B: RBAC & Admin Panel (3-4 days)

**Sprint 6: Role-Based Access Control (Days 8-9)**
- [ ] Implement `get_current_user` dependency (`src/auth/dependencies.py`)
- [ ] Implement `require_role` dependency factory
- [ ] Create middleware to attach user to request context
- [ ] Update RAG endpoints to use auth dependencies:
  - [ ] POST /ingest (require contributor/admin)
  - [ ] POST /query-global (optional auth, log if authenticated)
  - [ ] POST /query-local (optional auth)
- [ ] Unit tests for role checking logic
- [ ] Integration tests for protected endpoints

**Sprint 7: Admin API (Days 10-11)**
- [ ] Create AuditLog table (migration)
- [ ] Implement audit logging utility (log security events)
- [ ] Implement GET /admin/users endpoint (list users with pagination/search)
- [ ] Implement GET /admin/users/{id} endpoint (user details)
- [ ] Implement PATCH /admin/users/{id}/role endpoint (change role)
- [ ] Implement DELETE /admin/users/{id}/sessions endpoint (revoke sessions)
- [ ] Implement GET /admin/audit-logs endpoint (query logs)
- [ ] Prevent self-role-modification and last-admin-deletion
- [ ] Unit tests for admin operations
- [ ] Integration tests for admin workflows

### Phase 3C: Frontend Integration (3-4 days)

**Sprint 8: Auth UI Components (Days 12-13)**
- [ ] Create LoginForm component with email/password fields
- [ ] Create SignupForm component with password validation
- [ ] Create PasswordReset component (request + reset)
- [ ] Create AccountSettings component (profile, sessions, delete account)
- [ ] Create SessionList component (show active sessions, revoke)
- [ ] Add OAuth buttons (Google, GitHub) to LoginForm
- [ ] Style components with Docusaurus theme (light/dark mode support)
- [ ] Form validation with react-hook-form + zod

**Sprint 9: Auth State Management (Day 14)**
- [ ] Set up Zustand auth store
- [ ] Implement auto-token-refresh hook
- [ ] Implement API service functions (authService.ts)
- [ ] Add axios interceptor for auth headers
- [ ] Handle 401 errors (auto-logout)
- [ ] Persist user state (not access token) to localStorage

**Sprint 10: Protected Routes & UI Integration (Day 15)**
- [ ] Create ProtectedRoute component
- [ ] Update navbar: show "Sign In" for anonymous, user menu for authenticated
- [ ] Update RAG chatbot widget: check auth state, show "Sign in to save history" for anonymous
- [ ] Create admin pages:
  - [ ] /admin/users (user list with search/filter)
  - [ ] /admin/audit (audit log viewer)
- [ ] Add role-based UI (hide admin menu for non-admins)
- [ ] Test full user journey: signup → verify → login → query RAG → logout

### Phase 3D: Qdrant Integration & Data Privacy (2-3 days)

**Sprint 11: Qdrant Access Control (Day 16)**
- [ ] Add `access` metadata to existing Qdrant points (migration script)
- [ ] Implement `get_qdrant_filter` function based on user role
- [ ] Update /query-global endpoint to apply metadata filter
- [ ] Update /ingest endpoint to tag uploaded points with creator and access roles
- [ ] Test role-based filtering: reader sees only public, contributor sees all

**Sprint 12: GDPR Compliance (Day 17)**
- [ ] Implement DELETE /auth/account endpoint (password confirmation required)
- [ ] Cascade delete: User → UserProfile, RefreshToken, UserRole, QueryLog (anonymize or delete)
- [ ] Implement GET /auth/export endpoint (download user data as JSON)
- [ ] Test account deletion: verify all PII is removed
- [ ] Test data export: verify completeness

### Phase 3E: Testing & Deployment (3-5 days)

**Sprint 13: End-to-End Testing (Days 18-19)**
- [ ] E2E test: Email/password signup → verify → login → query RAG → logout
- [ ] E2E test: Google OAuth login → query RAG → logout
- [ ] E2E test: GitHub OAuth login → link to existing account
- [ ] E2E test: Password reset flow
- [ ] E2E test: Admin changes user role → user gains new permissions
- [ ] E2E test: Token refresh → access token rotated automatically
- [ ] E2E test: Token theft detection → all sessions revoked
- [ ] E2E test: Rate limiting → account locked after 5 failed logins
- [ ] E2E test: Anonymous user → read-only RAG access
- [ ] E2E test: Account deletion → all data removed

**Sprint 14: Security Audit (Day 20)**
- [ ] Code review: check for hardcoded secrets
- [ ] Verify HTTPS-only cookies in production config
- [ ] Verify CORS origins whitelist
- [ ] Verify rate limiting is active
- [ ] Verify audit logging captures all security events
- [ ] Verify password policy is enforced
- [ ] Verify JWT signature validation is correct
- [ ] Verify SQL injection prevention (use parameterized queries)
- [ ] Verify XSS prevention (sanitize user input in UI)

**Sprint 15: Deployment (Days 21-22)**
- [ ] Deploy FastAPI backend to Vercel/Render/Fly.io
- [ ] Configure production environment variables (secrets, HTTPS URLs)
- [ ] Run Alembic migrations on production Neon database
- [ ] Seed roles in production database
- [ ] Deploy Docusaurus frontend to Vercel
- [ ] Configure OAuth redirect URIs for production domain
- [ ] Test OAuth flows on production (Google, GitHub)
- [ ] Smoke test: signup, login, query RAG, admin panel
- [ ] Monitor logs for errors (CloudWatch, Sentry, or Render logs)

---

## Dependencies & Parallelizable Tasks

### External Dependencies
1. **Neon PostgreSQL Database** - Must be provisioned before backend setup
2. **Qdrant Cloud Instance** - Must be accessible with API key
3. **Email Service Account** (SendGrid/AWS SES/Resend) - Required for verification emails
4. **Google Cloud Console OAuth Client** - Required for Google OAuth
5. **GitHub OAuth App** - Required for GitHub OAuth
6. **Domain with HTTPS Certificate** - Required for production OAuth (dev can use localhost)

### Internal Dependencies (Sequential)
- Phase 0 (Research) → Phase 1 (Design) → Phase 3 (Implementation)
- Database schema design → Database migrations → Backend auth endpoints
- Backend auth endpoints → Frontend auth UI → Integration testing

### Parallelizable Tasks
- **Sprint 2 (Password Auth) || Sprint 3 (Email Verification)** - Different files, minimal overlap
- **Sprint 4 (OAuth) || Sprint 5 (Session Management)** - Can be developed in parallel
- **Sprint 6 (RBAC) || Sprint 7 (Admin Panel)** - Admin panel depends on RBAC, but RBAC can be tested independently
- **Sprint 8 (Frontend Components) || Sprint 9 (Auth State)** - Frontend team can work on UI while backend completes
- **Sprint 11 (Qdrant) || Sprint 12 (GDPR)** - Independent features

---

## Resource Requirements

### Team
- **1 Backend Developer** (Python/FastAPI expertise)
  - Responsible for: Database schema, auth endpoints, RBAC, admin API, Qdrant integration
  - Estimated effort: 12-15 days (full-time)

- **1 Frontend Developer** (TypeScript/React expertise)
  - Responsible for: Auth UI components, state management, Docusaurus integration, admin panel UI
  - Estimated effort: 5-7 days (full-time)

- **0.5 DevOps Engineer** (Database & deployment)
  - Responsible for: Neon database setup, Alembic migrations, FastAPI deployment, environment variables, monitoring
  - Estimated effort: 3-4 days (part-time)

### Tools & Services
- **Development**:
  - Code editor (VS Code, PyCharm)
  - Git version control
  - Postman/Insomnia for API testing
  - PostgreSQL client (psql, TablePlus, DBeaver)

- **Hosting & Infrastructure**:
  - Neon PostgreSQL (free tier: 0.5 GB storage, 1 project)
  - Qdrant Cloud (free tier: 1 GB RAM, 1 cluster)
  - Vercel (free tier: unlimited deploys for personal projects)
  - SendGrid (free tier: 100 emails/day) OR AWS SES ($0.10 per 1000 emails) OR Resend (free tier: 3000 emails/month)

- **Monitoring & Logging**:
  - Backend logs (Render/Fly.io/Vercel built-in logs)
  - Error tracking (optional: Sentry free tier)
  - Uptime monitoring (optional: UptimeRobot free tier)

---

## Success Criteria Validation

This implementation plan addresses all 23 success criteria from the specification:

### User Experience (SC-001 to SC-005)
- ✅ **SC-001**: Sign-up + verification + login flow optimized for speed (Sprint 2-3, E2E tests in Sprint 13)
- ✅ **SC-002**: Login success rate tracked via audit logs (Sprint 7)
- ✅ **SC-003**: Password reset flow tested for completion time (Sprint 3, Sprint 13)
- ✅ **SC-004**: OAuth flows tested for speed (Sprint 4, Sprint 13)
- ✅ **SC-005**: Auto-refresh implemented and tested (Sprint 9, Sprint 13)

### Security & Performance (SC-006 to SC-010)
- ✅ **SC-006**: Security audit checks for plaintext passwords/tokens (Sprint 14)
- ✅ **SC-007**: RBAC enforcement tested on all protected endpoints (Sprint 6, Sprint 13)
- ✅ **SC-008**: Rate limiting and account lockout implemented (Sprint 2, Sprint 13)
- ✅ **SC-009**: Token theft detection implemented and tested (Sprint 5, Sprint 13)
- ✅ **SC-010**: Audit logging implemented for all security events (Sprint 7)

### Operational & Compliance (SC-011 to SC-015)
- ✅ **SC-011**: Uptime monitored post-deployment (Sprint 15)
- ✅ **SC-012**: Endpoint latency tracked (performance goals defined in Technical Context)
- ✅ **SC-013**: Account deletion tested for GDPR compliance (Sprint 12, Sprint 13)
- ✅ **SC-014**: Data export implemented (Sprint 12)
- ✅ **SC-015**: Load testing for 1000 concurrent users (post-deployment)

### Admin & Governance (SC-016 to SC-018)
- ✅ **SC-016**: Admin panel search/filter tested (Sprint 7, Sprint 13)
- ✅ **SC-017**: Audit log filtering implemented (Sprint 7)
- ✅ **SC-018**: Audit log latency tested (Sprint 13)

### Integration & Deployment (SC-019 to SC-023)
- ✅ **SC-019**: Docusaurus theme integration tested (Sprint 10)
- ✅ **SC-020**: Anonymous user access tested (Sprint 11, Sprint 13)
- ✅ **SC-021**: Graceful degradation tested (Sprint 13)
- ✅ **SC-022**: Database migrations tested (Sprint 1, Sprint 15)
- ✅ **SC-023**: OAuth tested in local and production environments (Sprint 4, Sprint 15)

---

## Timeline Summary

| Phase | Duration | Deliverables |
|-------|----------|-------------|
| **Phase 0: Research** | 1-2 days | research.md with technology decisions |
| **Phase 1: Design** | 1 day | data-model.md, contracts/*.yaml, quickstart.md |
| **Phase 3A: Backend Core** | 5-7 days | Auth endpoints, email verification, OAuth, session management |
| **Phase 3B: RBAC & Admin** | 3-4 days | Role enforcement, admin API, audit logging |
| **Phase 3C: Frontend** | 3-4 days | Auth UI, state management, protected routes |
| **Phase 3D: Qdrant & Privacy** | 2-3 days | Access control, GDPR compliance |
| **Phase 3E: Testing & Deploy** | 3-5 days | E2E tests, security audit, production deployment |
| **Total** | **17-26 days** | **Full Better Auth system deployed** |

**Optimistic**: 3 weeks (with parallel work)
**Realistic**: 4 weeks (with dependencies and testing)
**Pessimistic**: 5 weeks (with unexpected blockers)

---

## Risk Analysis

### High-Risk Areas

1. **OAuth Provider Configuration Errors**
   - **Risk**: Misconfigured redirect URIs cause OAuth failures
   - **Mitigation**: Test OAuth in local dev first, use environment-based URLs, document setup thoroughly
   - **Contingency**: Have backup email/password flow working first

2. **Token Theft Detection False Positives**
   - **Risk**: Legitimate token refresh triggers theft detection, logs out all user sessions
   - **Mitigation**: Carefully implement rotation logic, test multi-device scenarios
   - **Contingency**: Add admin override to restore user sessions

3. **Rate Limiting Too Aggressive**
   - **Risk**: Legitimate users get locked out (shared IPs, VPNs)
   - **Mitigation**: Start with generous limits, monitor logs, add CAPTCHA as fallback
   - **Contingency**: Admin can manually unlock accounts

4. **Email Deliverability Issues**
   - **Risk**: Verification emails land in spam, users can't verify accounts
   - **Mitigation**: Use reputable email service (SendGrid/AWS SES), configure SPF/DKIM, test with multiple email providers
   - **Contingency**: Add manual verification bypass for admins

5. **Database Migration Failures**
   - **Risk**: Alembic migration fails on production, breaks auth system
   - **Mitigation**: Test migrations on staging environment first, use Neon's branching feature for safe testing
   - **Contingency**: Rollback plan with database backups

### Medium-Risk Areas

6. **CORS Misconfiguration**
   - **Risk**: Frontend can't call backend API due to CORS errors
   - **Mitigation**: Properly configure CORS_ORIGINS in backend, test cross-origin requests
   - **Contingency**: Temporarily allow all origins in dev, tighten in prod

7. **Qdrant Metadata Filter Performance**
   - **Risk**: Metadata filtering slows down RAG queries
   - **Mitigation**: Benchmark queries with filters, optimize indexes
   - **Contingency**: Cache query results, use simpler access control (all-or-nothing)

8. **Frontend Token Storage (XSS)**
   - **Risk**: Access token in localStorage vulnerable to XSS attacks
   - **Mitigation**: Store access token in memory only (lost on refresh, but safer), use HttpOnly cookies for refresh token
   - **Contingency**: Implement Content Security Policy (CSP) to mitigate XSS

---

## Next Steps

This plan is ready for **Phase 0 (Research)** execution. After research completion, Phase 1 artifacts will be generated, followed by Phase 2 task generation via `/sp.tasks`.

1. ✅ **Complete Phase 0**: Execute all 8 research tasks, generate `research.md`
2. ✅ **Complete Phase 1**: Generate `data-model.md`, `contracts/`, `quickstart.md`
3. ⏸️ **Run `/sp.tasks`**: Generate detailed implementation tasks (Phase 2 - separate command)
4. ⏸️ **Execute Implementation**: Sprints 1-15 as outlined in Phase 3
5. ⏸️ **Deploy to Production**: Sprint 15 deployment checklist
6. ⏸️ **Monitor & Iterate**: Post-deployment monitoring, user feedback, iterative improvements

**Current Status**: Plan complete, ready for research phase kickoff.

**Recommended First Action**: Begin Phase 0 Research Task 1 (Better Auth SDK Evaluation) or run all research tasks in parallel if resources allow.
