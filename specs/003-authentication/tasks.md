# Implementation Tasks: User Authentication & Profile Management

**Feature**: 003-authentication
**Branch**: `003-authentication`
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)
**Generated**: 2025-12-21

---

## Task Summary

| Phase | User Story | Priority | Task Count | Can Parallelize |
|-------|------------|----------|------------|-----------------|
| Phase 1 | Setup | - | 8 | Yes (T003-T008) |
| Phase 2 | Foundational | - | 12 | Partial (T012-T019) |
| Phase 3 | US1: Registration | P1 | 11 | Yes (T023-T028) |
| Phase 4 | US2: Login | P1 | 10 | Yes (T034-T039) |
| Phase 5 | US5: Persistent Sessions | P2 | 6 | Yes (T044-T048) |
| Phase 6 | US3: Profile Management | P2 | 8 | Yes (T051-T055) |
| Phase 7 | US4: Password Reset | P3 | 10 | Yes (T060-T065) |
| Phase 8 | Polish & Cross-cutting | - | 7 | Yes (T071-T076) |
| **Total** | | | **72** | **~45 parallelizable** |

---

## Implementation Strategy

**MVP Scope** (Minimum Viable Product):
- Phase 1: Setup (required)
- Phase 2: Foundational (required)
- Phase 3: User Story 1 - Registration (P1)
- Phase 4: User Story 2 - Login (P1)

**MVP delivers**: Users can register and log in. This enables basic authentication and unlocks personalization development.

**Incremental Delivery**:
1. **Iteration 1** (MVP): Registration + Login (Phases 1-4) → Deploy
2. **Iteration 2**: Add Persistent Sessions (Phase 5) → Deploy
3. **Iteration 3**: Add Profile Management (Phase 6) → Deploy
4. **Iteration 4**: Add Password Reset (Phase 7) → Deploy
5. **Iteration 5**: Polish & Final Release (Phase 8) → Deploy

**Independent Testing**: Each user story phase includes acceptance criteria that can be tested independently without other stories being complete.

---

## Dependencies & Execution Order

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational: Database, Auth Core)
    ↓
┌───────────────┬─────────────────┬──────────────────┐
│ Phase 3 (US1) │ Phase 4 (US2)   │ Phase 5 (US5)    │
│ Registration  │ Login           │ Sessions         │
│ [Independent] │ [Needs Phase 3] │ [Needs Phase 4]  │
└───────────────┴─────────────────┴──────────────────┘
    ↓                   ↓                  ↓
┌───────────────┬──────────────────────────┘
│ Phase 6 (US3) │ Phase 7 (US4)
│ Profile Mgmt  │ Password Reset
│ [Needs login] │ [Independent]
└───────────────┴──────────────────┘
    ↓
Phase 8 (Polish)
```

**Story Dependencies**:
- US1 (Registration) → No dependencies (can implement first)
- US2 (Login) → Depends on US1 (needs User model)
- US5 (Sessions) → Depends on US2 (extends login)
- US3 (Profile) → Depends on US2 (needs authentication)
- US4 (Password Reset) → No dependencies (can implement anytime)

**Parallel Opportunities**:
- Within each phase, tasks marked [P] can run in parallel
- Phases 3, 4, 5 can be developed by different team members simultaneously (after Phase 2)
- Phase 7 (Password Reset) can be developed in parallel with Phases 5-6

---

## Phase 1: Project Setup

**Goal**: Initialize project infrastructure and dependencies

**Tasks**:

- [ ] T001 Create Neon Postgres database and copy connection string to rag-backend/.env
- [ ] T002 Update rag-backend/requirements.txt with authentication dependencies (SQLAlchemy, alembic, python-jose, passlib[bcrypt], slowapi, resend, psycopg2-binary)
- [ ] T003 [P] Initialize Alembic in rag-backend/ with `alembic init migrations`
- [ ] T004 [P] Configure Alembic env.py to use SQLAlchemy Base and DATABASE_URL from rag-backend/app/config.py
- [ ] T005 [P] Update textbook/package.json with frontend dependencies (@better-auth/react, react-hook-form, zod)
- [ ] T006 [P] Create rag-backend/app/database/ directory structure (__init__.py, connection.py)
- [ ] T007 [P] Create rag-backend/app/auth/ directory structure (__init__.py, models.py, schemas.py, service.py, security.py, dependencies.py, email.py)
- [ ] T008 [P] Update rag-backend/app/config.py with new environment variables (DATABASE_URL, JWT_SECRET_KEY, JWT_ALGORITHM, RESEND_API_KEY)

**Acceptance**: All dependencies installed, directory structure created, environment variables configured

---

## Phase 2: Foundational Infrastructure

**Goal**: Set up database models, connection pooling, and core auth utilities that all user stories depend on

**Tasks**:

- [ ] T009 Implement SQLAlchemy database connection with pooling (pool_size=10, max_overflow=20) in rag-backend/app/database/connection.py
- [ ] T010 Create Base class and get_db() dependency function in rag-backend/app/database/__init__.py
- [ ] T011 Define User SQLAlchemy model (id, email, password_hash, is_active, created_at, updated_at, last_login_at) in rag-backend/app/auth/models.py
- [ ] T012 [P] Define LearningProfile SQLAlchemy model (id, user_id, name, skill_level, learning_goals, prior_experience) in rag-backend/app/auth/models.py
- [ ] T013 [P] Define Session SQLAlchemy model (id, user_id, token_hash, expires_at, device_info) in rag-backend/app/auth/models.py
- [ ] T014 [P] Define PasswordResetToken SQLAlchemy model (id, email, token_hash, expires_at, used_at) in rag-backend/app/auth/models.py
- [ ] T015 [P] Create Pydantic schemas for User (UserCreate, UserResponse, UserProfile) in rag-backend/app/auth/schemas.py
- [ ] T016 [P] Create Pydantic schemas for LearningProfile (LearningProfileCreate, LearningProfileUpdate, LearningProfileResponse) in rag-backend/app/auth/schemas.py
- [ ] T017 [P] Create Pydantic schemas for authentication (LoginRequest, RegisterRequest, AuthResponse, Token) in rag-backend/app/auth/schemas.py
- [ ] T018 [P] Implement password hashing and verification functions (hash_password, verify_password) using passlib/bcrypt in rag-backend/app/auth/security.py
- [ ] T019 [P] Implement JWT token generation and validation functions (create_access_token, verify_token) using python-jose in rag-backend/app/auth/security.py
- [ ] T020 Create Alembic migration for all 4 tables (users, learning_profiles, sessions, password_reset_tokens) with `alembic revision --autogenerate -m "Create authentication tables"`
- [ ] T021 Run Alembic migration with `alembic upgrade head` and verify tables exist in Neon Postgres

**Acceptance**: Database tables created, SQLAlchemy models functional, password hashing and JWT utilities working

---

## Phase 3: User Story 1 - New User Registration (P1)

**User Story**: A learner discovers the textbook and wants to create an account to access personalized features.

**Independent Test Criteria**:
1. Signup page accessible at /signup or via navbar "Sign Up" button
2. Form validates email format and password length (8+ chars)
3. Submitting valid data creates user in database with hashed password
4. User automatically logged in after registration (receives JWT token)
5. Duplicate email shows error "This email is already registered"

**Tasks**:

- [ ] T022 [US1] Implement user registration logic in AuthService.register() in rag-backend/app/auth/service.py
- [ ] T023 [P] [US1] Create POST /api/v1/auth/register endpoint in rag-backend/app/api/routes/auth.py
- [ ] T024 [P] [US1] Add input validation for registration (email format, password length, required fields) in rag-backend/app/api/routes/auth.py
- [ ] T025 [P] [US1] Implement email uniqueness check and return 409 Conflict if duplicate in rag-backend/app/auth/service.py
- [ ] T026 [P] [US1] Auto-create LearningProfile on user registration in rag-backend/app/auth/service.py
- [ ] T027 [P] [US1] Return JWT token and user profile after successful registration in rag-backend/app/api/routes/auth.py
- [ ] T028 [P] [US1] Add rate limiting (3 registrations per hour per IP) to register endpoint in rag-backend/app/api/routes/auth.py
- [ ] T029 [US1] Create SignUpModal React component with form fields (email, password, name, skill_level, learning_goals, prior_experience) in textbook/src/components/SignUpModal/index.tsx
- [ ] T030 [US1] Implement form validation using react-hook-form and zod schema in textbook/src/components/SignUpModal/index.tsx
- [ ] T031 [US1] Connect SignUpModal to POST /api/v1/auth/register endpoint and handle success/error responses in textbook/src/components/SignUpModal/index.tsx
- [ ] T032 [US1] Add "Sign Up" button to Docusaurus navbar that opens SignUpModal in textbook/src/theme/Navbar/index.tsx

**Acceptance Testing** (US1):
```bash
# Manual Test
1. Click "Sign Up" in navbar
2. Fill form: test@example.com, TestPass123!, John Doe, Beginner
3. Submit
4. Verify: User logged in, name shows in navbar, JWT in localStorage

# API Test
curl -X POST http://localhost:8000/api/v1/auth/register \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"TestPass123!","name":"John Doe","skill_level":"beginner"}'
# Expected: 201 Created, returns access_token and user profile
```

---

## Phase 4: User Story 2 - Returning User Login (P1)

**User Story**: A returning learner wants to access their personalized textbook experience by logging in.

**Independent Test Criteria**:
1. Login page accessible at /login or via navbar "Sign In" button
2. Form validates email and password
3. Correct credentials create session and return JWT token
4. Incorrect credentials show error "Invalid email or password"
5. Logged-in user sees name in navbar with dropdown menu

**Tasks**:

- [ ] T033 [US2] Implement user login logic in AuthService.login() in rag-backend/app/auth/service.py
- [ ] T034 [P] [US2] Create POST /api/v1/auth/login endpoint in rag-backend/app/api/routes/auth.py
- [ ] T035 [P] [US2] Implement password verification and return JWT on success in rag-backend/app/api/routes/auth.py
- [ ] T036 [P] [US2] Add rate limiting (5 login attempts per 15 minutes per IP) to login endpoint in rag-backend/app/api/routes/auth.py
- [ ] T037 [P] [US2] Update last_login_at timestamp on successful login in rag-backend/app/auth/service.py
- [ ] T038 [P] [US2] Create Session record in database on login for auditing in rag-backend/app/auth/service.py
- [ ] T039 [P] [US2] Implement POST /api/v1/auth/logout endpoint that invalidates session in rag-backend/app/api/routes/auth.py
- [ ] T040 [US2] Create LoginModal React component with email and password fields in textbook/src/components/LoginModal/index.tsx
- [ ] T041 [US2] Connect LoginModal to POST /api/v1/auth/login endpoint and store JWT in localStorage/cookies in textbook/src/components/LoginModal/index.tsx
- [ ] T042 [US2] Create AuthProvider component that wraps app and provides authentication context in textbook/src/components/AuthProvider/index.tsx
- [ ] T043 [US2] Add user dropdown menu (name, "My Profile", "Sign Out") to Docusaurus navbar when logged in in textbook/src/theme/Navbar/index.tsx

**Acceptance Testing** (US2):
```bash
# Manual Test
1. Click "Sign In" in navbar
2. Enter: test@example.com, TestPass123!
3. Submit
4. Verify: Logged in, name shows in navbar dropdown
5. Click "Sign Out"
6. Verify: Logged out, navbar shows "Sign In" button

# API Test
curl -X POST http://localhost:8000/api/v1/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"TestPass123!"}'
# Expected: 200 OK, returns access_token
```

---

## Phase 5: User Story 5 - Persistent Sessions (P2)

**User Story**: A logged-in learner closes their browser and returns later, remaining logged in without re-entering credentials.

**Independent Test Criteria**:
1. User logs in without "Remember Me" → session lasts 24 hours
2. User logs in with "Remember Me" checked → session lasts 30 days
3. Closing and reopening browser preserves authentication within session duration
4. Sessions expire after 7 days of inactivity
5. Explicit logout clears session immediately

**Tasks**:

- [ ] T044 [P] [US5] Add "Remember Me" checkbox to LoginModal in textbook/src/components/LoginModal/index.tsx
- [ ] T045 [P] [US5] Implement JWT expiry logic: 24hr default, 30 days if remember_me=true in rag-backend/app/auth/security.py
- [ ] T046 [P] [US5] Store JWT in httpOnly cookie (secure, SameSite=Lax) instead of localStorage in textbook/src/components/AuthProvider/index.tsx
- [ ] T047 [P] [US5] Implement token refresh endpoint POST /api/v1/auth/refresh that extends valid tokens in rag-backend/app/api/routes/auth.py
- [ ] T048 [P] [US5] Add get_current_user() dependency that validates JWT from cookie in rag-backend/app/auth/dependencies.py
- [ ] T049 [US5] Create background task to cleanup expired sessions (runs daily) in rag-backend/app/auth/service.py

**Acceptance Testing** (US5):
```bash
# Manual Test
1. Log in with "Remember Me" checked
2. Close browser completely
3. Reopen browser and navigate to site
4. Verify: Still logged in (no login prompt)
5. Wait 31 days, refresh page
6. Verify: Session expired, prompted to log in

# API Test (verify JWT expiry)
# Login → Get token → Wait 25 hours → Use token
# Expected: Token rejected after 24hr expiry
```

---

## Phase 6: User Story 3 - Profile Management (P2)

**User Story**: A logged-in learner wants to update their learning profile to match their current skill level and goals.

**Independent Test Criteria**:
1. "My Profile" link accessible in user dropdown menu
2. Profile page shows current user data (name, email, skill level, goals, experience)
3. Updating fields and clicking "Save" persists changes to database
4. Profile updates visible immediately after save
5. Changing email to existing one shows error

**Tasks**:

- [ ] T050 [US3] Create GET /api/v1/profile endpoint that returns current user's profile in rag-backend/app/api/routes/profile.py
- [ ] T051 [P] [US3] Create PUT /api/v1/profile endpoint that updates profile fields in rag-backend/app/api/routes/profile.py
- [ ] T052 [P] [US3] Implement profile update logic in ProfileService.update() in rag-backend/app/auth/service.py
- [ ] T053 [P] [US3] Add validation for profile fields (skill_level enum, max lengths) in rag-backend/app/api/routes/profile.py
- [ ] T054 [P] [US3] Check email uniqueness when updating email field in rag-backend/app/auth/service.py
- [ ] T055 [P] [US3] Protect profile endpoints with get_current_user() dependency in rag-backend/app/api/routes/profile.py
- [ ] T056 [US3] Create ProfilePage component with editable form (name, skill_level, learning_goals, prior_experience) in textbook/src/components/ProfilePage/index.tsx
- [ ] T057 [US3] Connect ProfilePage to GET and PUT /api/v1/profile endpoints in textbook/src/components/ProfilePage/index.tsx
- [ ] T058 [US3] Add profile route /profile that renders ProfilePage in textbook/src/pages/profile.tsx

**Acceptance Testing** (US3):
```bash
# Manual Test
1. Log in
2. Click user dropdown → "My Profile"
3. Update skill level: Beginner → Intermediate
4. Update learning goals: "Learn ROS 2" → "Build autonomous robots"
5. Click "Save Profile"
6. Verify: Success message, changes saved
7. Refresh page
8. Verify: Updated values still displayed

# API Test
curl -X PUT http://localhost:8000/api/v1/profile \
  -H "Authorization: Bearer <token>" \
  -H "Content-Type: application/json" \
  -d '{"skill_level":"intermediate","learning_goals":"Build autonomous robots"}'
# Expected: 200 OK, returns updated profile
```

---

## Phase 7: User Story 4 - Password Reset (P3)

**User Story**: A learner forgot their password and needs to regain access via email-based password reset.

**Independent Test Criteria**:
1. "Forgot Password?" link accessible on login page
2. Entering email sends reset link (generic success message for all emails)
3. Reset link expires after 1 hour
4. Clicking valid link shows password reset form
5. Submitting new password updates database and allows login
6. Reset links are single-use only

**Tasks**:

- [ ] T059 [US4] Configure Resend email service with API key in rag-backend/app/config.py
- [ ] T060 [P] [US4] Implement email sending function send_password_reset_email() in rag-backend/app/auth/email.py
- [ ] T061 [P] [US4] Create POST /api/v1/auth/forgot-password endpoint that generates reset token and sends email in rag-backend/app/api/routes/password.py
- [ ] T062 [P] [US4] Create PasswordResetToken record in database with 1hr expiry in rag-backend/app/auth/service.py
- [ ] T063 [P] [US4] Add rate limiting (5 reset requests per hour per email) to forgot-password endpoint in rag-backend/app/api/routes/password.py
- [ ] T064 [P] [US4] Create POST /api/v1/auth/reset-password endpoint that validates token and updates password in rag-backend/app/api/routes/password.py
- [ ] T065 [P] [US4] Mark reset token as used (used_at timestamp) after successful password change in rag-backend/app/auth/service.py
- [ ] T066 [US4] Create PasswordResetModal component with email input form in textbook/src/components/PasswordResetModal/index.tsx
- [ ] T067 [US4] Create password reset page /reset-password?token=XYZ with new password form in textbook/src/pages/reset-password.tsx
- [ ] T068 [US4] Add "Forgot Password?" link to LoginModal that opens PasswordResetModal in textbook/src/components/LoginModal/index.tsx

**Acceptance Testing** (US4):
```bash
# Manual Test
1. Click "Sign In" → "Forgot Password?"
2. Enter: test@example.com
3. Submit
4. Check email inbox
5. Click reset link from email
6. Enter new password: NewPass123!
7. Submit
8. Verify: Success message
9. Log in with new password
10. Verify: Login successful

# API Test
# Request reset
curl -X POST http://localhost:8000/api/v1/auth/forgot-password \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com"}'

# Reset password (use token from email)
curl -X POST http://localhost:8000/api/v1/auth/reset-password \
  -H "Content-Type: application/json" \
  -d '{"token":"<reset-token>","new_password":"NewPass123!"}'
# Expected: 200 OK
```

---

## Phase 8: Polish & Cross-Cutting Concerns

**Goal**: Finalize security, performance, accessibility, and integration with existing features

**Tasks**:

- [ ] T069 Implement CSRF protection middleware using double-submit cookie pattern in rag-backend/app/main.py
- [ ] T070 Add CORS configuration for production domains in rag-backend/app/main.py
- [ ] T071 [P] Update ChatWidget to pass authenticated user context (user_id, skill_level) to chat API in textbook/src/components/ChatWidget/index.tsx
- [ ] T072 [P] Modify POST /api/v1/chat/query to accept optional user_id for personalized responses in rag-backend/app/api/routes/chat.py
- [ ] T073 [P] Add WCAG 2.1 AA compliance: aria-labels, keyboard navigation, focus indicators to all auth components in textbook/src/components/
- [ ] T074 [P] Style authentication components with CSS modules to match Docusaurus theme in textbook/src/components/*/styles.module.css
- [ ] T075 [P] Create backend integration tests for complete auth flows (register → login → profile → reset) in rag-backend/tests/integration/test_auth_flow.py
- [ ] T076 [P] Create Playwright E2E tests for frontend auth flows in textbook/tests/e2e/auth.spec.ts

**Acceptance**: All auth features integrated seamlessly, security hardened, accessibility verified, E2E tests passing

---

## Parallel Execution Examples

### Phase 2 (Foundational) - 3 Developers
**Dev 1**: T009-T010 (Database connection)
**Dev 2**: T011-T014 (SQLAlchemy models)
**Dev 3**: T015-T019 (Pydantic schemas + security utils)
→ Then all devs meet for T020-T021 (migration)

### Phase 3 (US1 Registration) - 2 Developers
**Backend Dev**: T022-T028 (API endpoints, rate limiting)
**Frontend Dev**: T029-T032 (SignUpModal, navbar integration)
→ Both streams can run in parallel

### Phase 4 (US2 Login) - 2 Developers
**Backend Dev**: T033-T039 (Login API, session management)
**Frontend Dev**: T040-T043 (LoginModal, AuthProvider, navbar)
→ Both streams can run in parallel

### Phases 5-7 (Sessions + Profile + Password Reset) - 3 Developers
**Dev 1**: Phase 5 (US5 Persistent Sessions) - T044-T049
**Dev 2**: Phase 6 (US3 Profile Management) - T050-T058
**Dev 3**: Phase 7 (US4 Password Reset) - T059-T068
→ All three can work independently

---

## Testing Strategy

**Unit Tests** (per user story):
- Phase 3 (US1): Test user registration logic, password hashing, email validation
- Phase 4 (US2): Test login logic, JWT generation, session creation
- Phase 6 (US3): Test profile update logic, email uniqueness check
- Phase 7 (US4): Test password reset token generation, email sending

**Integration Tests** (after Phase 8):
- T075: Complete auth flow (register → login → update profile → logout)
- Test rate limiting enforcement
- Test CSRF protection
- Test session expiry

**E2E Tests** (after Phase 8):
- T076: Playwright tests for all user journeys
- Test cross-browser compatibility (Chrome, Firefox, Safari)
- Test mobile responsiveness
- Test keyboard navigation (accessibility)

**Performance Tests**:
- Load test login endpoint: 100 concurrent users
- Verify session validation < 50ms
- Verify registration < 2 minutes (SC-001)

---

## Risk Mitigation

| Risk | Mitigation | Tasks |
|------|------------|-------|
| Neon Postgres downtime | Use connection pooling with retry logic | T009 |
| Email service (Resend) failure | Implement fallback to SendGrid | T059, T060 |
| JWT secret compromise | Implement key rotation strategy | T019, Document in quickstart.md |
| Rate limiting bypass | Use both IP and user-based limits | T028, T036, T063 |
| Breaking existing features | Maintain unauthenticated access, add feature flags | T071, T072 |
| CSRF attacks | Double-submit cookie + SameSite cookies | T069, T046 |

---

## Deployment Checklist

Before deploying to production:
- [ ] Set production DATABASE_URL (not free tier)
- [ ] Generate secure JWT_SECRET_KEY (32+ bytes)
- [ ] Configure CORS_ORIGINS for production domain
- [ ] Set RESEND_API_KEY with verified sending domain
- [ ] Enable HTTPS (required for secure cookies)
- [ ] Run all migrations: `alembic upgrade head`
- [ ] Test authentication flow end-to-end in staging
- [ ] Set up database backups (Neon snapshots)
- [ ] Configure monitoring (error tracking, performance)
- [ ] Review security headers (HSTS, CSP)

---

## Notes

- **No breaking changes**: All tasks designed to preserve existing unauthenticated textbook access (FR-032, FR-034)
- **Constitution compliant**: All 13 applicable gates passed (see plan.md Constitution Check)
- **Independent stories**: Each user story can be tested in isolation using acceptance criteria
- **Incremental delivery**: MVP (Phases 1-4) can be deployed first, remaining features added incrementally

---

**Last Updated**: 2025-12-21
**Total Tasks**: 72 (8 setup + 12 foundational + 52 implementation)
**Estimated Effort**: ~2-3 weeks for 2 developers (MVP in 1 week)
**Ready for Implementation**: ✅ YES
