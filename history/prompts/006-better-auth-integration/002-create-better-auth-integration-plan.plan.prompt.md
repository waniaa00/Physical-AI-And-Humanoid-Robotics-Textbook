---
id: 002
title: Create Better Auth Integration Implementation Plan
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5
feature: 006-better-auth-integration
branch: 006-better-auth-integration
user: system
command: /sp.plan
labels: ["authentication", "better-auth", "architecture", "fastapi", "jwt", "security"]
links:
  spec: specs/006-better-auth-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/006-better-auth-integration/plan.md
tests:
  - Constitution check validation (PASS - all security gates satisfied)
---

## Prompt

```text
/sp.plan
- Initialize Better Auth in FastAPI backend
- Configure Neon Postgres adapter for user storage
- Add sign-up and sign-in endpoints with interest selection
- Protect RAG and agent endpoints with authentication
- Inject user context into agent for personalization
```

## Response snapshot

Created comprehensive implementation plan for Better Auth integration with:

**Technical Context**:
- FastAPI backend with Better Auth Python SDK (or FastAPI-Users fallback)
- Neon Serverless Postgres for user/session storage
- JWT tokens in HttpOnly cookies (7-day TTL)
- Optional authentication middleware for backward compatibility
- Performance targets: <500ms sign-up/sign-in, <50ms middleware overhead

**Project Structure**:
- Backend: 8 new files (auth_service.py, auth_middleware.py, auth.py models, user_repository.py, password_hasher.py, token_manager.py)
- Frontend: 5 new files (AuthContext.tsx, useAuth.ts, auth.ts, ProtectedRoute.tsx)
- Database: User, Session, UserInterest tables (extends Phase 5 schema)
- 5 API endpoints: sign-up, sign-in, sign-out, session/validate, password-reset-request

**Phase 0 Research Tasks (5 items)**:
1. R1: Better Auth Python SDK availability and FastAPI integration pattern
2. R2: Neon Postgres adapter configuration and connection pooling
3. R3: JWT vs session-based auth decision (recommended: JWT HttpOnly cookies)
4. R4: Password hashing best practices (bcrypt cost 12 or Argon2id)
5. R5: CORS and secure cookie configuration for localhost + production

**Phase 1 Design Artifacts (6 deliverables)**:
1. research.md - Technology decisions and library selection
2. data-model.md - User, Session, UserInterest entity definitions
3. contracts/auth-api.yaml - OpenAPI 3.0 spec for 5 authentication endpoints
4. contracts/middleware-interface.md - Optional auth middleware specification
5. quickstart.md - Local development setup guide
6. .specify/agents/ClaudeCode.md - Agent context update with Better Auth capabilities

**Key Design Decisions (5 critical choices)**:
1. Better Auth vs FastAPI-Users: Deferred to research (prefer Better Auth if Python SDK exists)
2. JWT in HttpOnly Cookies vs LocalStorage: HttpOnly cookies with SameSite=Lax (XSS/CSRF protection)
3. Password Reset: Deferred to post-MVP (requires email service integration)
4. Session Expiration: Fixed 7-day TTL (simple MVP, refresh tokens later)
5. Middleware Approach: Optional authentication for guest user compatibility

**Architecture**:
- Mermaid diagram showing Frontend (AuthContext) → Backend (AuthRoutes, AuthMiddleware, AuthService) → Database (Users, Sessions, Interests)
- Request flow: SignUp/SignIn → JWT generation → Cookie storage → Protected endpoint access with user context injection
- Middleware pattern: Optional auth (guest fallback) for `/agent/chat`, required auth for `/interests/save`

**Security Considerations (10 threats mitigated)**:
1. Password theft: bcrypt cost 12
2. Session hijacking: HttpOnly cookies
3. CSRF: SameSite=Lax cookies
4. XSS: Input sanitization, CSP headers
5. SQL injection: Parameterized queries
6. Account enumeration: Generic error messages
7. Brute force: Rate limiting (5 attempts/15 min)
8. Session fixation: Token invalidation on sign-out
9. Token theft: Secure flag in production
10. CORS: Explicit allowed origins

**Testing Strategy**:
- Unit tests: auth_service, password_hasher, token_manager (90-100% coverage)
- Integration tests: End-to-end auth flow, middleware validation
- Frontend tests: AuthContext, useAuth hook, ProtectedRoute
- Security tests: OWASP ZAP, SQL injection, XSS, CSRF, session attacks

**Dependencies Added**:
- Backend: better-auth-python (or fastapi-users), bcrypt==4.1.1, python-jose[cryptography]==3.3.0
- Frontend: @better-auth/react

**Migration Plan**: Temporary user ID (00000000-0000-0000-0000-000000000001) coexists until post-MVP migration

## Outcome

- ✅ Impact: Created production-ready implementation plan with Phase 0/1 research and design tasks
- 🧪 Tests: Constitution check validated (all security principles satisfied)
- 📁 Files: 1 file created (plan.md - 872 lines comprehensive architecture doc)
- 🔁 Next prompts: Execute Phase 0 research tasks (R1-R5) OR run `/sp.tasks` to generate task breakdown
- 🧠 Reflection: Plan balances security best practices (OWASP compliance) with backward compatibility (guest users). Research phase critical for Better Auth Python SDK availability decision.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed without blockers
- Graders run and results (PASS/FAIL): Constitution security gates PASS (JWT, bcrypt, HttpOnly cookies align with Principle X)
- Prompt variant (if applicable): User provided high-level bullet points; plan expanded to full architecture with 5 research tasks, 6 design deliverables
- Next experiment (smallest change to try): Begin Phase 0 research with R1 (Better Auth Python SDK availability) as critical path decision
