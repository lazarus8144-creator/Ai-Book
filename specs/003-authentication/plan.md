# Implementation Plan: User Authentication & Profile Management

**Branch**: `003-authentication` | **Date**: 2025-12-21 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-authentication/spec.md`

## Summary

Integrate Better-auth authentication system into the existing Physical AI & Humanoid Robotics textbook to enable user registration, login, and profile management. Users will be able to create accounts with email/password, maintain learning profiles (skill level, goals, prior experience), and access personalized features. This feature serves as the foundation for Phase 2 content personalization and enables user-specific RAG chatbot interactions.

**Primary Requirements**:
- Email/password authentication with secure credential storage
- User profile management with learning metadata
- Session persistence (24hr default, 30 days with "Remember Me")
- Password reset flow via email
- Integration with existing Docusaurus UI and FastAPI backend
- Zero impact on unauthenticated textbook access

**Technical Approach** (from research):
- Better-auth library for authentication logic and session management
- Neon Postgres for user account and profile storage
- FastAPI backend endpoints for auth operations
- React components integrated into Docusaurus navigation
- JWT tokens for stateless session management
- Resend or SendGrid for password reset emails

## Technical Context

**Language/Version**:
- Backend: Python 3.11+ (FastAPI 0.115.0)
- Frontend: TypeScript 5.6.2 (React 19.0.0, Docusaurus 3.9.2)

**Primary Dependencies**:
- Backend: FastAPI, Pydantic 2.9.0, SQLAlchemy 2.0+, python-jose[cryptography], passlib[bcrypt], python-multipart
- Frontend: @better-auth/react, react-hook-form, zod (validation)
- Database: psycopg2-binary (Neon Postgres driver)
- Email: resend or sendgrid Python SDK

**Storage**:
- Neon Postgres (serverless Postgres) for user accounts, profiles, sessions, and password reset tokens
- Tables: users, learning_profiles, sessions, password_reset_tokens

**Testing**:
- Backend: pytest with pytest-asyncio, httpx for API testing
- Frontend: Playwright for E2E auth flows, React Testing Library for component tests

**Target Platform**:
- Backend: Linux server (Railway/Render deployment, ASGI via Uvicorn)
- Frontend: Web browsers (Chrome, Firefox, Safari, Edge - ES2020+)

**Project Type**: Web application (existing frontend + backend structure)

**Performance Goals**:
- Registration completion < 2 minutes (SC-001)
- Login response < 10 seconds (SC-002)
- Password reset email delivery < 2 minutes (SC-004)
- Support 100 concurrent authenticated users (SC-003)
- Session validation overhead < 50ms per request

**Constraints**:
- MUST NOT break existing unauthenticated access (FR-032, FR-034)
- MUST use HTTPS for all auth endpoints (FR-025)
- MUST hash passwords with bcrypt/argon2 (FR-004, Constitution)
- MUST implement rate limiting on auth endpoints (FR-029, FR-023)
- MUST be WCAG 2.1 AA compliant (SC-010)
- Zero plaintext passwords in database (SC-006)

**Scale/Scope**:
- Initial: 50-100 active users
- Growth: Scale to 1000+ users without architecture changes
- Database: ~10 tables, <1GB data initially
- Frontend: 4 new pages/modals (signup, login, profile, password reset)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ I. Educational Excellence First
- **Pass**: Authentication is optional, does not block textbook access (FR-034)
- **Pass**: Existing RAG chatbot remains fully functional for unauthenticated users (FR-032)
- **Pass**: Profile fields capture learning context (skill level, goals) to enable personalization

### ✅ II. Progressive Enhancement Architecture
- **Pass**: Phase 1 (textbook + RAG) complete before starting authentication
- **Pass**: Authentication is Phase 2 feature (+50 points bonus)
- **Pass**: Feature can be independently tested without breaking core functionality
- **Pass**: Authentication enables but does not require future personalization features

### ✅ III. User-Centric Personalization
- **Pass**: Learning profile captures: skill level, learning goals, prior experience (FR-013)
- **Pass**: Profile data designed for future content personalization (FR-033)
- **Pass**: Users can opt out (authentication is optional, not required)

### ❌ IV. Multilingual Accessibility
- **Not Applicable**: Authentication UI is English-only in MVP
- **Future**: Will support Urdu translation in Feature 005 (same UI translation approach)

### ✅ V. AI-Native Development
- **Pass**: Specification created via `/sp.specify`
- **Pass**: Implementation plan via `/sp.plan` (this document)
- **Pass**: Tasks will be generated via `/sp.tasks`
- **Pass**: PHRs created for significant development sessions

### ✅ Mandatory Tech Stack Compliance
- **Pass**: Using Better-auth as specified in constitution
- **Pass**: FastAPI backend for authentication endpoints
- **Pass**: Neon Postgres for user data storage
- **Pass**: Docusaurus frontend integration

### ✅ Performance Requirements
- **Pass**: Auth flows target <1 second (Constitution requirement)
- **Pass**: Login < 10 seconds target (SC-002)
- **Pass**: 100 concurrent users target (SC-003)

### ✅ Security Protocols
- **Pass**: HTTPS enforced (FR-025)
- **Pass**: Bcrypt password hashing (FR-004)
- **Pass**: Secure session tokens (FR-006)
- **Pass**: Environment variables for secrets (Constitution)
- **Pass**: Rate limiting on auth endpoints (FR-029, FR-023)
- **Pass**: CORS configuration for deployed frontend (Constitution)

### ✅ Accessibility Compliance
- **Pass**: WCAG 2.1 AA target (SC-010)
- **Pass**: Keyboard navigation in auth forms
- **Pass**: Screen reader compatible form labels

**Constitution Check Result**: ✅ **PASSED** (13/13 applicable gates)

## Project Structure

### Documentation (this feature)

```text
specs/003-authentication/
├── spec.md              # Feature specification ✅
├── plan.md              # This file (architecture plan)
├── research.md          # Phase 0: Technology research and decisions
├── data-model.md        # Phase 1: Database schema and entities
├── quickstart.md        # Phase 1: Developer setup guide
├── contracts/           # Phase 1: API contracts
│   ├── auth-api.yaml    # OpenAPI spec for auth endpoints
│   └── types.ts         # TypeScript types for frontend
├── checklists/
│   └── requirements.md  # Spec validation checklist ✅
└── tasks.md             # Phase 2: Implementation tasks (via /sp.tasks)
```

### Source Code (repository root)

```text
# Backend: Extend existing FastAPI structure
rag-backend/
├── app/
│   ├── auth/                          # NEW: Authentication module
│   │   ├── __init__.py
│   │   ├── models.py                  # User, LearningProfile, Session, PasswordResetToken
│   │   ├── schemas.py                 # Pydantic models for API requests/responses
│   │   ├── service.py                 # Business logic (registration, login, profile)
│   │   ├── security.py                # Password hashing, token generation, JWT
│   │   ├── dependencies.py            # FastAPI dependencies (get_current_user)
│   │   └── email.py                   # Email sending service (password reset)
│   ├── api/
│   │   └── routes/
│   │       ├── auth.py                # NEW: /api/v1/auth/register, /login, /logout
│   │       ├── profile.py             # NEW: /api/v1/profile (GET, PUT)
│   │       ├── password.py            # NEW: /api/v1/auth/forgot-password, /reset-password
│   │       └── chat.py                # MODIFY: Add user context to chatbot
│   ├── database/                      # NEW: Database connection
│   │   ├── __init__.py
│   │   ├── connection.py              # Neon Postgres connection pool
│   │   └── migrations/                # Alembic migrations
│   │       ├── env.py
│   │       ├── versions/
│   │       │   └── 001_create_auth_tables.py
│   └── config.py                      # MODIFY: Add DATABASE_URL, JWT_SECRET, EMAIL_API_KEY
├── tests/
│   ├── auth/                          # NEW: Auth tests
│   │   ├── test_auth_api.py           # Test registration, login, logout
│   │   ├── test_profile_api.py        # Test profile CRUD
│   │   ├── test_password_reset.py     # Test password reset flow
│   │   └── test_security.py           # Test rate limiting, CSRF, password hashing
│   └── integration/                   # NEW: E2E auth flows
│       └── test_auth_flow.py          # Full user journey tests
├── alembic.ini                        # NEW: Alembic configuration
├── requirements.txt                   # MODIFY: Add SQLAlchemy, alembic, python-jose, passlib, resend

# Frontend: Extend existing Docusaurus structure
textbook/
├── src/
│   ├── components/
│   │   ├── AuthProvider/             # NEW: Better-auth context provider
│   │   │   ├── index.tsx
│   │   │   └── better-auth.config.ts # Better-auth client configuration
│   │   ├── SignUpModal/              # NEW: Registration modal
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   ├── LoginModal/               # NEW: Login modal
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   ├── ProfilePage/              # NEW: User profile page
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   ├── PasswordResetModal/       # NEW: Password reset flow
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   ├── AuthButton/               # NEW: Sign In/Sign Out button
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   └── ChatWidget/               # MODIFY: Add user context
│   │       └── index.tsx              # Pass authenticated user info to API
│   ├── pages/
│   │   └── profile.tsx               # NEW: Profile page route
│   └── theme/
│       ├── Root.tsx                  # MODIFY: Wrap with AuthProvider
│       └── Navbar/                   # MODIFY: Add AuthButton to navbar
│           └── index.tsx
├── static/
│   └── config.js                     # MODIFY: Add API_BASE_URL for auth
└── package.json                      # MODIFY: Add @better-auth/react, react-hook-form, zod
```

**Structure Decision**: Web application with separate backend and frontend (existing structure). Authentication backend logic in `rag-backend/app/auth/`, frontend components in `textbook/src/components/`. Database migrations managed via Alembic. Better-auth handles client-side session management, FastAPI provides REST endpoints.

## Complexity Tracking

> **No constitution violations requiring justification.**

All design choices align with constitution requirements. No additional complexity introduced beyond what is necessary for the feature.

---

## Phase 0: Research & Technology Decisions

See [research.md](./research.md) for detailed analysis of:

1. **Better-auth vs Custom Auth**: Why Better-auth for React integration
2. **JWT vs Session Tokens**: Stateless JWT approach for scalability
3. **Password Hashing Algorithm**: Bcrypt vs Argon2 comparison
4. **Email Service Provider**: Resend vs SendGrid vs AWS SES
5. **Database Schema Design**: Users, profiles, sessions table structure
6. **Rate Limiting Strategy**: Token bucket vs fixed window
7. **CSRF Protection**: Double-submit cookie vs synchronizer token pattern

**Key Decisions**:
- Better-auth for client-side auth state management (React integration, TypeScript support)
- FastAPI backend provides REST endpoints (Better-auth calls backend APIs)
- JWT tokens for stateless sessions (stored in httpOnly cookies)
- Bcrypt for password hashing (industry standard, good performance)
- Resend for email service (developer-friendly, free tier)
- SQLAlchemy ORM with Alembic migrations (schema versioning)
- Token bucket rate limiting via slowapi (per-IP and per-user limits)

---

## Phase 1: Data Model & Contracts

### Data Model

See [data-model.md](./data-model.md) for complete schema definitions.

**Core Entities**:

1. **User** (users table)
   - id (UUID, PK)
   - email (unique, indexed)
   - password_hash (bcrypt)
   - created_at, updated_at, last_login_at
   - is_active (soft delete support)

2. **LearningProfile** (learning_profiles table)
   - id (UUID, PK)
   - user_id (FK → users.id)
   - name (display name)
   - skill_level (enum: beginner/intermediate/advanced)
   - learning_goals (text)
   - prior_experience (text)
   - created_at, updated_at

3. **Session** (sessions table)
   - id (UUID, PK)
   - user_id (FK → users.id)
   - token_hash (indexed)
   - expires_at (indexed)
   - device_info (JSON: user agent, IP)
   - created_at

4. **PasswordResetToken** (password_reset_tokens table)
   - id (UUID, PK)
   - email (indexed)
   - token_hash (indexed)
   - expires_at (default: created_at + 1 hour)
   - used_at (nullable, marks token as consumed)
   - created_at

### API Contracts

See [contracts/auth-api.yaml](./contracts/auth-api.yaml) for OpenAPI specification.

**Endpoints**:

```text
POST   /api/v1/auth/register      # Create account
POST   /api/v1/auth/login         # Sign in
POST   /api/v1/auth/logout        # Sign out
POST   /api/v1/auth/refresh       # Refresh JWT token

GET    /api/v1/profile            # Get current user profile
PUT    /api/v1/profile            # Update profile

POST   /api/v1/auth/forgot-password  # Request password reset
POST   /api/v1/auth/reset-password   # Complete password reset

GET    /api/v1/auth/me            # Get current user (auth check)
```

**Request/Response Examples** (see contracts/ for full schemas):

```typescript
// POST /api/v1/auth/register
Request: {
  email: string;
  password: string;
  name: string;
  skill_level: "beginner" | "intermediate" | "advanced";
  learning_goals?: string;
  prior_experience?: string;
}
Response: {
  access_token: string;
  token_type: "bearer";
  user: UserProfile;
}

// POST /api/v1/auth/login
Request: {
  email: string;
  password: string;
  remember_me?: boolean;
}
Response: {
  access_token: string;
  token_type: "bearer";
  user: UserProfile;
}

// PUT /api/v1/profile
Request: {
  name?: string;
  skill_level?: "beginner" | "intermediate" | "advanced";
  learning_goals?: string;
  prior_experience?: string;
}
Response: {
  user: UserProfile;
}
```

### Developer Quickstart

See [quickstart.md](./quickstart.md) for complete setup instructions.

**Summary**:
1. Set up Neon Postgres database (free tier)
2. Configure environment variables (DATABASE_URL, JWT_SECRET, RESEND_API_KEY)
3. Run Alembic migrations to create auth tables
4. Install frontend dependencies (@better-auth/react)
5. Start backend and frontend development servers
6. Test authentication flow end-to-end

---

## Phase 2: Task Breakdown

Task generation will be handled by `/sp.tasks` command. Expected task categories:

1. **Database Setup** (5-7 tasks)
   - Create Neon Postgres database
   - Configure Alembic
   - Create migration for auth tables
   - Seed test users

2. **Backend Implementation** (15-20 tasks)
   - Implement User model and CRUD
   - Implement password hashing and verification
   - Implement JWT token generation and validation
   - Create registration endpoint
   - Create login endpoint
   - Create logout endpoint
   - Create profile endpoints
   - Implement password reset flow
   - Add rate limiting middleware
   - Add CSRF protection
   - Configure email service

3. **Frontend Implementation** (15-20 tasks)
   - Set up Better-auth provider
   - Create SignUpModal component
   - Create LoginModal component
   - Create ProfilePage component
   - Create PasswordResetModal component
   - Create AuthButton component
   - Integrate AuthButton into Navbar
   - Add protected route HOC
   - Update ChatWidget with user context
   - Add form validation with Zod

4. **Testing** (10-15 tasks)
   - Write unit tests for auth service
   - Write API integration tests
   - Write E2E tests for registration flow
   - Write E2E tests for login flow
   - Write E2E tests for password reset
   - Test rate limiting
   - Test CSRF protection
   - Test session persistence

5. **Documentation & Deployment** (5-7 tasks)
   - Update README with auth setup
   - Create API documentation
   - Configure environment variables for production
   - Deploy database migrations
   - Test production authentication flow

**Total Estimated Tasks**: 50-69 tasks

---

## Integration Points

### 1. Docusaurus Navigation
- Add "Sign In" button to navbar when logged out
- Add user menu (name, "My Profile", "Sign Out") when logged in
- Ensure authentication modals work with Docusaurus routing

### 2. RAG Chatbot
- Pass authenticated user context to chat API (user_id, skill_level)
- Backend stores chat history per user (future: personalized responses)
- Maintain unauthenticated chatbot functionality

### 3. Better-auth Client
- Better-auth manages client-side session state
- Calls FastAPI backend for authentication operations
- Stores JWT in httpOnly cookies (secure, XSS-protected)

### 4. Database
- All auth data in Neon Postgres (separate from Qdrant vector DB)
- Connection pooling for concurrent requests
- Migrations managed via Alembic

---

## Security Considerations

1. **Password Storage**: Bcrypt with salt rounds = 12 (Constitution compliance)
2. **Session Tokens**: JWT with 24hr expiry (default) or 30d (remember_me)
3. **Password Reset**: Tokens expire after 1hr, single-use only
4. **Rate Limiting**:
   - Login: 5 attempts per 15 min per IP (FR-029)
   - Password reset: 5 requests per hr per email (FR-023)
5. **CSRF Protection**: Double-submit cookie pattern
6. **XSS Prevention**: Input sanitization on all form fields (FR-027)
7. **HTTPS Only**: All auth cookies marked Secure (FR-025)
8. **Email Enumeration**: Generic messages for password reset (FR-024)

---

## Performance Optimization

1. **Database Indexing**:
   - Index on users.email (login queries)
   - Index on sessions.token_hash (session validation)
   - Index on password_reset_tokens.token_hash (reset queries)

2. **Connection Pooling**:
   - SQLAlchemy pool size = 10
   - Max overflow = 20
   - Pool timeout = 30s

3. **Caching** (future optimization):
   - Cache user profiles in Redis (reduce DB queries)
   - Cache session validation (reduce DB lookups)

4. **JWT Validation**:
   - Stateless validation (no DB query for every request)
   - Only DB query on profile updates or logout

---

## Rollback Plan

If authentication implementation fails or blocks progress:

1. **Feature Flag**: Disable auth UI via environment variable
2. **Database Rollback**: Alembic downgrade to remove auth tables
3. **Code Isolation**: All auth code in separate modules (easy to disable)
4. **Zero Impact**: Unauthenticated access continues working (FR-032, FR-034)

---

## Success Metrics (from spec.md)

- **SC-001**: Registration < 2 minutes ✅ Target: 60-90 seconds
- **SC-002**: Login < 10 seconds ✅ Target: 2-5 seconds
- **SC-003**: 100 concurrent users ✅ Connection pool + async FastAPI
- **SC-004**: Password reset email < 2 minutes ✅ Resend: typically <30 seconds
- **SC-005**: 95% first-try success ✅ Clear validation errors
- **SC-006**: Zero plaintext passwords ✅ Bcrypt hashing enforced
- **SC-007**: Zero enumeration attacks ✅ Generic reset messages
- **SC-008**: 99% session persistence ✅ JWT in httpOnly cookies
- **SC-009**: 100% profile update success ✅ Database constraints + validation
- **SC-010**: WCAG 2.1 AA ✅ Accessible forms with labels

---

## Next Steps

1. ✅ Specification complete (`spec.md`)
2. ✅ Architecture plan complete (this document)
3. ⏳ Research phase (Phase 0): Create `research.md`
4. ⏳ Data model phase (Phase 1): Create `data-model.md`, `contracts/`, `quickstart.md`
5. ⏳ Task generation (Phase 2): Run `/sp.tasks` to create `tasks.md`
6. ⏳ Implementation: Execute tasks in dependency order

**Ready for**: `/sp.plan` Phase 0 research generation
