# 🔐 Environment Variables Configuration Guide

Complete guide to configuring all environment variables for development and production.

---

## Overview

Your project uses environment variables to:
- Store API keys securely
- Configure different environments (dev/prod)
- Control feature flags
- Set performance parameters

---

## Backend Environment Variables

### Step 1: Create `.env` File

```bash
cd backend
cp .env.example .env
nano .env  # or code .env
```

### Step 2: Configure All Variables

```bash
# ============================================================================
# OpenAI Configuration
# ============================================================================

# Get from: https://platform.openai.com/api-keys
OPENAI_API_KEY=sk-proj-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

# Models (don't change unless you have a reason)
OPENAI_EMBEDDING_MODEL=text-embedding-3-small  # 1536 dimensions, $0.02/1M tokens
OPENAI_CHAT_MODEL=gpt-4o-mini                  # Fast & cheap, $0.15/1M input
OPENAI_TEMPERATURE=0.1                         # Low randomness for consistency
OPENAI_MAX_TOKENS=500                          # Answer length limit

# ============================================================================
# Qdrant Configuration
# ============================================================================

# Get from: https://cloud.qdrant.io (see SETUP-QDRANT.md)
QDRANT_URL=https://abc123-def456.eu-central.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
QDRANT_COLLECTION_NAME=textbook_chunks

# ============================================================================
# Application Configuration
# ============================================================================

# Environment: development, staging, production
ENVIRONMENT=development

# Logging level: DEBUG, INFO, WARNING, ERROR
LOG_LEVEL=INFO

# Admin token for protected endpoints (min 32 chars)
# Generate with: openssl rand -hex 32
ADMIN_TOKEN=your-secure-random-token-here-min-32-characters

# CORS: Allowed origins (JSON array)
# Development:
ALLOWED_ORIGINS=["http://localhost:3000"]

# Production (update after deploying frontend):
# ALLOWED_ORIGINS=["https://your-project.vercel.app","https://yourusername.github.io"]

# ============================================================================
# Rate Limiting
# ============================================================================

# Per IP address limits
RATE_LIMIT_QUERIES_PER_MINUTE=20
RATE_LIMIT_QUERIES_PER_HOUR=100

# ============================================================================
# Performance Tuning
# ============================================================================

# Embedding cache size (LRU cache)
EMBEDDING_CACHE_SIZE=1000

# Vector search configuration
VECTOR_SEARCH_LIMIT=5          # Top K chunks to retrieve
MAX_CHUNK_SIZE=800             # Max tokens per chunk
CHUNK_OVERLAP=100              # Token overlap between chunks

# ============================================================================
# Phase 3 Bonus: Claude Subagents (+50 points)
# ============================================================================

# Get from: https://console.anthropic.com/settings/keys
# ANTHROPIC_API_KEY=sk-ant-xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

# Enable subagent enhancement
# ENABLE_SUBAGENT_ENHANCEMENT=true

# ============================================================================
# Phase 2 Bonus: Neon Postgres (for auth & personalization)
# ============================================================================

# Get from: https://neon.tech
# DATABASE_URL=postgresql://user:password@ep-xxx.region.aws.neon.tech/dbname?sslmode=require
# DATABASE_POOL_SIZE=10
# DATABASE_POOL_TIMEOUT=30
```

---

## Frontend Environment Variables

### Step 1: Create `.env.local` File

```bash
cd textbook
cp .env.local.example .env.local
nano .env.local  # or code .env.local
```

### Step 2: Configure Variables

**Development (`.env.local`):**
```bash
# API Configuration - Local Backend
REACT_APP_API_URL=http://localhost:8000

# Optional: Enable debug mode
# REACT_APP_DEBUG=true
```

**Production (`.env.production`):**
```bash
# API Configuration - Production Backend
REACT_APP_API_URL=https://your-backend.railway.app

# Analytics (optional)
# REACT_APP_GA_TRACKING_ID=G-XXXXXXXXXX
```

---

## How to Get Each API Key

### 1. OpenAI API Key

**Steps:**
1. Go to **https://platform.openai.com/api-keys**
2. Log in with your OpenAI account
3. Click **"+ Create new secret key"**
4. Name: `rag-chatbot-backend`
5. **Copy the key immediately** (shown only once!)
6. Paste into `.env` as `OPENAI_API_KEY`

**Format:** `sk-proj-...` (51 characters)

**Cost Estimate:**
- Embeddings: ~$0.003 one-time (ingestion)
- Queries: ~$1.20/month for 1000 queries

### 2. Qdrant Credentials

See **SETUP-QDRANT.md** for detailed steps.

**Quick:**
1. Sign up at **https://cloud.qdrant.io**
2. Create free cluster
3. Copy **Cluster URL** and **API Key**

### 3. Admin Token (Generate Locally)

**Option A: OpenSSL**
```bash
openssl rand -hex 32
# Output: a1b2c3d4e5f6...  (64 chars)
```

**Option B: Python**
```python
import secrets
print(secrets.token_hex(32))
```

**Option C: Online**
```
https://www.random.org/strings/?num=1&len=32&digits=on&loweralpha=on&unique=on
```

Paste the generated token into `ADMIN_TOKEN`.

### 4. Anthropic API Key (Phase 3 Bonus)

**Steps:**
1. Go to **https://console.anthropic.com/settings/keys**
2. Log in or create account
3. Click **"Create Key"**
4. Name: `rag-chatbot-subagent`
5. Copy key
6. Uncomment and set `ANTHROPIC_API_KEY` in `.env`

**Format:** `sk-ant-...`

**Cost Estimate:**
- ~500 tokens per enhancement
- ~$0.05 per 100 enhanced queries
- Much cheaper than you'd expect!

### 5. Neon Postgres (Phase 2 Bonus)

**Steps:**
1. Go to **https://neon.tech**
2. Sign up with GitHub (recommended)
3. Create project: `ai-textbook`
4. Copy connection string
5. Uncomment and set `DATABASE_URL` in `.env`

**Format:**
```
postgresql://user:password@ep-xxx.region.aws.neon.tech/dbname?sslmode=require
```

---

## Environment-Specific Configuration

### Development Setup

```bash
# backend/.env
ENVIRONMENT=development
LOG_LEVEL=DEBUG
ALLOWED_ORIGINS=["http://localhost:3000"]

# textbook/.env.local
REACT_APP_API_URL=http://localhost:8000
REACT_APP_DEBUG=true
```

### Production Setup

```bash
# Railway environment variables (set in dashboard)
ENVIRONMENT=production
LOG_LEVEL=INFO
ALLOWED_ORIGINS=["https://your-project.vercel.app"]

# Vercel environment variables (set in dashboard)
REACT_APP_API_URL=https://your-backend.railway.app
```

---

## Verification Checklist

### Backend Verification

```bash
cd backend
source venv/bin/activate

# Test environment loading
python -c "
from app.config import settings
print('✅ OpenAI Key:', settings.OPENAI_API_KEY[:10] + '...')
print('✅ Qdrant URL:', settings.QDRANT_URL)
print('✅ Environment:', settings.ENVIRONMENT)
print('✅ CORS Origins:', settings.ALLOWED_ORIGINS)
"
```

**Expected output:**
```
✅ OpenAI Key: sk-proj-ab...
✅ Qdrant URL: https://abc123...
✅ Environment: development
✅ CORS Origins: ['http://localhost:3000']
```

### Frontend Verification

```bash
cd textbook

# Check environment variables
npm run start

# In browser console (http://localhost:3000):
console.log('API URL:', process.env.REACT_APP_API_URL);
```

**Expected output:**
```
API URL: http://localhost:8000
```

---

## Security Best Practices

### 1. Never Commit Secrets

```bash
# Verify .gitignore is correct
cat .gitignore | grep .env

# Should show:
# .env
# .env.local
# .env*.local
```

### 2. Use Different Keys for Dev/Prod

```bash
# Development: Use project API keys
OPENAI_API_KEY=sk-proj-dev-...

# Production: Use organization API keys with limits
OPENAI_API_KEY=sk-proj-prod-...
```

### 3. Rotate Keys Regularly

**Every 90 days:**
1. Generate new API keys
2. Update `.env` and Railway/Vercel
3. Test thoroughly
4. Delete old keys

### 4. Use Secret Managers (Production)

**Railway:**
- Secrets are encrypted at rest
- Not visible in logs
- Can be updated without redeployment

**Vercel:**
- Environment variables encrypted
- Separate for preview/production
- Can be environment-specific

---

## Common Issues & Solutions

### Issue: "OPENAI_API_KEY not set"

**Solution:**
```bash
# Check .env exists
ls -la backend/.env

# Check variable is set
cat backend/.env | grep OPENAI_API_KEY

# Restart server after changing .env
# (Ctrl+C and run uvicorn again)
```

### Issue: "CORS error" in browser

**Solution:**
```bash
# Backend .env must include frontend origin
ALLOWED_ORIGINS=["http://localhost:3000"]

# Multiple origins (JSON array):
ALLOWED_ORIGINS=["http://localhost:3000","https://your-site.vercel.app"]

# Restart backend after changing
```

### Issue: Environment variables not loading

**Solution:**
```python
# Check pydantic-settings is installed
pip list | grep pydantic-settings

# Verify .env location
# Should be: backend/.env (same level as app/)

# Check for syntax errors in .env
# No spaces around =
# CORRECT: KEY=value
# WRONG:   KEY = value
```

### Issue: "Invalid API key" from OpenAI

**Solution:**
```bash
# Test key directly
curl https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY"

# If error: regenerate key at platform.openai.com
```

---

## Template Files Reference

### Minimal `.env` (Quick Start)

```bash
# Essentials only - get started in 5 minutes
OPENAI_API_KEY=sk-...
QDRANT_URL=https://...
QDRANT_API_KEY=...
QDRANT_COLLECTION_NAME=textbook_chunks
ADMIN_TOKEN=$(openssl rand -hex 32)
ALLOWED_ORIGINS=["http://localhost:3000"]
```

### Full `.env` (Production Ready)

See complete template in `backend/.env.example`

---

## Railway-Specific Configuration

When deploying to Railway, set these as **environment variables** in the dashboard (not in `.env` file):

```bash
# Copy these from your local .env
OPENAI_API_KEY
QDRANT_URL
QDRANT_API_KEY
QDRANT_COLLECTION_NAME
ADMIN_TOKEN

# Update these for production
ENVIRONMENT=production
ALLOWED_ORIGINS=["https://your-frontend.vercel.app"]
```

See **SETUP-RAILWAY.md** for detailed deployment steps.

---

## Vercel-Specific Configuration

Set in Vercel project settings → Environment Variables:

```bash
REACT_APP_API_URL=https://your-backend.railway.app
```

---

## Quick Commands Reference

```bash
# Generate admin token
openssl rand -hex 32

# Test backend env loading
cd backend && python -c "from app.config import settings; print(settings.OPENAI_API_KEY[:10])"

# Check frontend env
cd textbook && npm run start
# Then in browser: console.log(process.env.REACT_APP_API_URL)

# Restart backend after .env changes
# Ctrl+C, then:
uvicorn app.main:app --reload

# Restart frontend after .env changes
# Ctrl+C, then:
npm run start
```

---

**Status**: ✅ Environment variables configured
**Next**: Deploy to Railway (see SETUP-RAILWAY.md)
