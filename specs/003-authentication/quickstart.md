# Authentication Feature - Developer Quickstart

**Feature**: 003-authentication
**Date**: 2025-12-21
**Estimated Setup Time**: 30-45 minutes

This guide will get you up and running with the authentication system in under an hour.

---

## Prerequisites

- Python 3.11+ installed
- Node.js 18+ and npm installed
- Git installed
- Text editor (VS Code recommended)
- Terminal/command line access

---

## Step 1: Database Setup (Neon Postgres)

### 1.1 Create Free Neon Account

1. Visit https://neon.tech and sign up (free tier)
2. Create a new project: "ai-textbook-auth"
3. Copy the connection string (it looks like `postgres://user:pass@host/db`)

### 1.2 Save Connection String

```bash
# Backend .env file
cd rag-backend
echo "DATABASE_URL=postgresql://your-connection-string-here" >> .env
```

**Important**: Keep this connection string secret! Never commit to git.

---

## Step 2: Backend Setup

### 2.1 Install Dependencies

```bash
cd rag-backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # Windows: venv\Scripts\activate

# Install packages
pip install --upgrade pip
pip install -r requirements.txt
```

### 2.2 Configure Environment Variables

Create or update `rag-backend/.env`:

```bash
# Database
DATABASE_URL=postgresql://your-neon-connection-string

# JWT Secret (generate with: openssl rand -hex 32)
JWT_SECRET_KEY=your-secret-key-here-use-openssl-to-generate

# JWT Settings
JWT_ALGORITHM=RS256
JWT_ACCESS_TOKEN_EXPIRE_MINUTES=1440  # 24 hours
JWT_REFRESH_TOKEN_EXPIRE_DAYS=30

# Email Service (Resend)
RESEND_API_KEY=re_your-api-key-here

# CORS
CORS_ORIGINS=http://localhost:3000,http://127.0.0.1:3000

# Rate Limiting
RATELIMIT_ENABLED=true
```

### 2.3 Run Database Migrations

```bash
# Initialize Alembic (first time only)
alembic init migrations

# Create initial migration
alembic revision --autogenerate -m "Create authentication tables"

# Run migration
alembic upgrade head
```

Expected output:
```
INFO  [alembic.runtime.migration] Running upgrade  -> 001_auth_tables, Create authentication tables
```

### 2.4 Start Backend Server

```bash
# Development mode with auto-reload
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

Verify backend is running:
```bash
curl http://localhost:8000/api/v1/health
# Expected: {"status":"ok","timestamp":"2025-12-21T..."}
```

---

## Step 3: Email Service Setup (Resend)

### 3.1 Get Resend API Key

1. Visit https://resend.com and sign up (free tier)
2. Create API key in dashboard
3. Copy key (starts with `re_`)

### 3.2 Add to Environment

```bash
echo "RESEND_API_KEY=re_your_key_here" >> rag-backend/.env
```

### 3.3 Test Email Sending

```bash
cd rag-backend
python -c "
import resend
import os
resend.api_key = os.getenv('RESEND_API_KEY')
print(resend.Emails.send({
    'from': 'onboarding@resend.dev',
    'to': 'your-email@example.com',
    'subject': 'Test Email',
    'html': '<p>Authentication system working!</p>'
}))
"
```

---

## Step 4: Frontend Setup

### 4.1 Install Dependencies

```bash
cd textbook

# Install new packages
npm install @better-auth/react react-hook-form zod
```

### 4.2 Configure API Base URL

Update `textbook/static/config.js`:

```javascript
window.APP_CONFIG = {
  API_BASE_URL: process.env.REACT_APP_API_URL || 'http://localhost:8000'
};
```

### 4.3 Start Frontend Server

```bash
npm start
```

Frontend should open at http://localhost:3000

---

## Step 5: End-to-End Test

### 5.1 Test Registration Flow

1. Open http://localhost:3000
2. Click "Sign Up" button (will be in navbar)
3. Fill in form:
   - Email: test@example.com
   - Password: TestPass123!
   - Name: Test User
   - Skill Level: Beginner
   - Learning Goals: Learn ROS 2
4. Submit form
5. Verify:
   - ✅ Success message appears
   - ✅ User menu shows name "Test User"
   - ✅ Console shows JWT token

### 5.2 Test Login Flow

1. Click "Sign Out"
2. Click "Sign In"
3. Enter credentials:
   - Email: test@example.com
   - Password: TestPass123!
4. Submit
5. Verify logged in (name appears in navbar)

### 5.3 Test Profile Update

1. Click user menu → "My Profile"
2. Update skill level to "Intermediate"
3. Add learning goal: "Build autonomous robots"
4. Click "Save Profile"
5. Refresh page
6. Verify changes persisted

### 5.4 Test Password Reset

1. Log out
2. Click "Sign In" → "Forgot Password?"
3. Enter email: test@example.com
4. Check email for reset link
5. Click link, enter new password
6. Log in with new password

---

## Step 6: Verify Database

```bash
# Connect to Neon Postgres
psql $DATABASE_URL

# Check tables created
\dt

# Expected tables:
# - users
# - learning_profiles
# - sessions
# - password_reset_tokens

# Query test user
SELECT id, email, created_at FROM users;

# Exit
\q
```

---

## Troubleshooting

### Backend won't start

**Error**: `ModuleNotFoundError: No module named 'sqlalchemy'`

**Fix**:
```bash
cd rag-backend
source venv/bin/activate
pip install -r requirements.txt
```

### Database connection fails

**Error**: `could not connect to server: Connection refused`

**Fix**:
1. Check `DATABASE_URL` in `.env`
2. Verify Neon project is active
3. Check internet connection
4. Try connection string in browser/pgAdmin

### Migrations fail

**Error**: `alembic.util.exc.CommandError: Can't locate revision identified by '001_auth_tables'`

**Fix**:
```bash
# Delete existing migrations
rm -rf migrations/

# Re-initialize
alembic init migrations

# Create migration
alembic revision --autogenerate -m "Create auth tables"

# Run migration
alembic upgrade head
```

### JWT errors

**Error**: `JWTError: Invalid token`

**Fix**:
1. Regenerate JWT secret:
   ```bash
   openssl rand -hex 32
   ```
2. Update `.env` with new secret
3. Restart backend
4. Clear browser cookies

### CORS errors in browser

**Error**: `Access to fetch at 'http://localhost:8000' from origin 'http://localhost:3000' has been blocked by CORS policy`

**Fix**:
```bash
# Update CORS_ORIGINS in .env
CORS_ORIGINS=http://localhost:3000,http://127.0.0.1:3000

# Restart backend
```

### Email not sending

**Error**: `resend.exceptions.ValidationError: Invalid API key`

**Fix**:
1. Verify API key in Resend dashboard
2. Check `.env` has correct key (starts with `re_`)
3. Restart backend after updating `.env`

---

## Development Workflow

### Running Tests

```bash
# Backend unit tests
cd rag-backend
pytest tests/auth/

# Backend integration tests
pytest tests/integration/

# Frontend tests
cd textbook
npm test
```

### Database Migrations

```bash
# Create new migration after model changes
alembic revision --autogenerate -m "Description of change"

# Apply migration
alembic upgrade head

# Rollback last migration
alembic downgrade -1

# View migration history
alembic history
```

### Seeding Test Data

```bash
cd rag-backend
python -c "
from app.auth.models import User, LearningProfile
from app.auth.security import get_password_hash
from app.database.connection import get_db

db = next(get_db())

# Create test users
for i in range(5):
    user = User(
        email=f'user{i}@test.com',
        password_hash=get_password_hash('TestPass123!'),
    )
    db.add(user)
    db.commit()

    profile = LearningProfile(
        user_id=user.id,
        name=f'Test User {i}',
        skill_level='beginner',
        learning_goals='Learn robotics'
    )
    db.add(profile)
db.commit()
print('Created 5 test users')
"
```

---

## Next Steps

1. ✅ Backend running on port 8000
2. ✅ Frontend running on port 3000
3. ✅ Database migrations applied
4. ✅ Email service configured
5. ✅ Test user created

**You're ready to start implementing authentication features!**

Proceed to:
- [tasks.md](./tasks.md) - Implementation task list
- [data-model.md](./data-model.md) - Database schema reference
- [contracts/auth-api.yaml](./contracts/auth-api.yaml) - API documentation

---

## Useful Commands Reference

```bash
# Backend
cd rag-backend && source venv/bin/activate
uvicorn app.main:app --reload

# Frontend
cd textbook && npm start

# Database
psql $DATABASE_URL

# Migrations
alembic upgrade head
alembic downgrade -1

# Tests
pytest tests/auth/ -v
npm test

# Logs
tail -f rag-backend/logs/app.log
```

---

## Production Checklist

Before deploying to production:

- [ ] Change `JWT_SECRET_KEY` to production value
- [ ] Set `CORS_ORIGINS` to production domain
- [ ] Use production database (not free tier)
- [ ] Enable rate limiting (`RATELIMIT_ENABLED=true`)
- [ ] Configure email domain verification in Resend
- [ ] Enable HTTPS (required for secure cookies)
- [ ] Set up database backups
- [ ] Configure monitoring (error tracking, performance)
- [ ] Test password reset email delivery
- [ ] Review security headers (HSTS, CSP)

---

**Last Updated**: 2025-12-21
**Maintained By**: Authentication Team
