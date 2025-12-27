# 🚂 Railway Deployment Guide

Complete guide to deploying your FastAPI backend to Railway.

---

## Why Railway?

✅ **Free tier**: 500 hours/month (~20 days continuous)
✅ **No cold starts**: Unlike Vercel/Render free tiers
✅ **Fast deployment**: Git push → live in 2 minutes
✅ **Zero config**: Automatically detects FastAPI
✅ **Built-in PostgreSQL**: Easy to add Neon later

**Alternative**: Render (15-min cold starts on free tier - not recommended)

---

## Prerequisites

- ✅ Backend code ready and tested locally
- ✅ GitHub account
- ✅ Backend code pushed to GitHub
- ✅ Environment variables documented

---

## Step 1: Prepare Backend for Deployment

### 1.1 Create `Procfile`

```bash
cd backend
nano Procfile  # Create new file
```

**Content:**
```
web: uvicorn app.main:app --host 0.0.0.0 --port $PORT
```

This tells Railway how to start your application.

### 1.2 Create `railway.json` (Optional but Recommended)

```bash
nano railway.json
```

**Content:**
```json
{
  "$schema": "https://railway.app/railway.schema.json",
  "build": {
    "builder": "NIXPACKS",
    "buildCommand": "pip install -r requirements.txt"
  },
  "deploy": {
    "startCommand": "uvicorn app.main:app --host 0.0.0.0 --port $PORT",
    "restartPolicyType": "ON_FAILURE",
    "restartPolicyMaxRetries": 10
  }
}
```

### 1.3 Create `runtime.txt` (Specify Python Version)

```bash
nano runtime.txt
```

**Content:**
```
python-3.11
```

### 1.4 Update `.gitignore`

Ensure these are ignored:
```bash
# Already added from earlier
__pycache__/
.env
venv/
*.pyc
```

### 1.5 Commit Changes

```bash
git add Procfile railway.json runtime.txt
git commit -m "Add Railway deployment configuration"
git push origin main
```

---

## Step 2: Create Railway Account

### 2.1 Sign Up

1. Go to **https://railway.app**
2. Click **"Start a New Project"** or **"Login"**
3. **Sign up with GitHub** (recommended)
4. Authorize Railway to access your repositories

### 2.2 Verify Email

1. Check your email inbox
2. Click verification link
3. You'll be redirected to Railway dashboard

---

## Step 3: Deploy Backend

### 3.1 Create New Project

1. In Railway dashboard, click **"+ New Project"**
2. Select **"Deploy from GitHub repo"**
3. Choose your backend repository
   - If you don't see it, click **"Configure GitHub App"**
   - Grant access to your repository
4. Select the repository: `ai-textbook-backend` (or your repo name)

### 3.2 Configure Deployment

Railway will automatically:
- ✅ Detect Python project
- ✅ Install dependencies from `requirements.txt`
- ✅ Run the start command from `Procfile`

**This takes about 2-3 minutes.**

### 3.3 Monitor Deployment

You'll see build logs in real-time:
```
Building...
 ✓ Installing Python 3.11
 ✓ Installing dependencies
 ✓ Running uvicorn app.main:app

Deployment successful! 🎉
```

---

## Step 4: Configure Environment Variables

### 4.1 Open Variables Tab

1. In your Railway project, click on the service
2. Click **"Variables"** tab
3. Click **"+ New Variable"**

### 4.2 Add All Environment Variables

Add these **one by one**:

```bash
# OpenAI Configuration
OPENAI_API_KEY=sk-proj-your-actual-key-here
OPENAI_EMBEDDING_MODEL=text-embedding-3-small
OPENAI_CHAT_MODEL=gpt-4o-mini
OPENAI_TEMPERATURE=0.1
OPENAI_MAX_TOKENS=500

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=textbook_chunks

# Application Configuration
ENVIRONMENT=production
LOG_LEVEL=INFO
ADMIN_TOKEN=your-secure-32-char-token
ALLOWED_ORIGINS=["https://your-frontend.vercel.app"]

# Rate Limiting
RATE_LIMIT_QUERIES_PER_MINUTE=20
RATE_LIMIT_QUERIES_PER_HOUR=100

# Performance
EMBEDDING_CACHE_SIZE=1000
VECTOR_SEARCH_LIMIT=5
MAX_CHUNK_SIZE=800
CHUNK_OVERLAP=100

# Optional: Subagents (Phase 3 bonus)
# ANTHROPIC_API_KEY=sk-ant-your-key
# ENABLE_SUBAGENT_ENHANCEMENT=true
```

**Tips:**
- Copy values from your local `backend/.env`
- For `ALLOWED_ORIGINS`, use your **deployed frontend URL**
- Use **JSON array format**: `["https://url1.com","https://url2.com"]`

### 4.3 Deploy with New Variables

Railway automatically **redeploys** when you add variables.

Wait ~2 minutes for deployment to complete.

---

## Step 5: Get Deployment URL

### 5.1 Generate Public Domain

1. In Railway service settings, click **"Settings"** tab
2. Scroll to **"Networking"** or **"Domains"**
3. Click **"Generate Domain"**

Railway will create a URL like:
```
https://ai-textbook-backend-production.up.railway.app
```

**Copy this URL** - you'll need it for frontend configuration.

### 5.2 (Optional) Add Custom Domain

If you have a custom domain:
1. Click **"+ Custom Domain"**
2. Enter your domain: `api.yourdomain.com`
3. Add CNAME record to your DNS:
   ```
   CNAME api.yourdomain.com → xxx.up.railway.app
   ```
4. Wait for DNS propagation (~5-10 minutes)

---

## Step 6: Verify Deployment

### 6.1 Test Health Endpoint

```bash
curl https://your-backend.up.railway.app/api/v1/health
```

**Expected response:**
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "openai_available": true,
  "version": "1.0.0"
}
```

### 6.2 Test RAG Query

```bash
curl -X POST https://your-backend.up.railway.app/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

**Expected:** JSON response with answer and sources in <3 seconds.

### 6.3 Check Swagger Docs

Open in browser:
```
https://your-backend.up.railway.app/docs
```

You should see FastAPI interactive documentation.

---

## Step 7: Configure CORS for Frontend

### 7.1 Update ALLOWED_ORIGINS

After deploying frontend to Vercel/GitHub Pages:

1. Go to Railway → Variables
2. Update `ALLOWED_ORIGINS`:
   ```json
   ["https://your-project.vercel.app","https://yourusername.github.io"]
   ```
3. Save (auto-redeploys)

### 7.2 Test CORS

```bash
# Test CORS preflight
curl -X OPTIONS https://your-backend.up.railway.app/api/v1/query \
  -H "Origin: https://your-frontend.vercel.app" \
  -H "Access-Control-Request-Method: POST" \
  -v

# Should see:
# < access-control-allow-origin: https://your-frontend.vercel.app
# < access-control-allow-methods: GET, POST
```

---

## Step 8: Update Frontend Configuration

### 8.1 Update Frontend Environment Variable

**For Vercel:**
1. Go to Vercel project → Settings → Environment Variables
2. Add variable:
   ```
   REACT_APP_API_URL=https://your-backend.up.railway.app
   ```
3. Redeploy frontend

**For GitHub Pages:**
Create `.env.production` in `textbook/`:
```bash
REACT_APP_API_URL=https://your-backend.up.railway.app
```

Rebuild and deploy:
```bash
npm run build
npm run deploy
```

---

## Monitoring & Maintenance

### View Logs

1. Railway dashboard → Your service
2. Click **"Deployments"** tab
3. Click latest deployment
4. View real-time logs

**Look for:**
```
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### Monitor Usage

**Check usage limits:**
1. Railway dashboard → Account settings
2. View **"Usage"** section

**Free Tier:**
- 500 hours/month
- $5 free credit
- 1 GB RAM
- 1 vCPU

**Your expected usage:**
- ~720 hours/month (if running 24/7)
- **Exceeds free tier by ~220 hours**

**Solutions:**
1. **Pause service** when not in use
2. **Use Render** as fallback (free but with cold starts)
3. **Upgrade** to Hobby plan ($5/month) before demo day

### Set Up Alerts

1. Railway → Project settings → Notifications
2. Enable email alerts for:
   - Deployment failures
   - Resource usage (>80%)
   - Service downtime

---

## Optimization Tips

### 1. Reduce Memory Usage

**In `app/config.py`:**
```python
# Reduce cache sizes
EMBEDDING_CACHE_SIZE=500  # Instead of 1000
VECTOR_SEARCH_LIMIT=3     # Instead of 5
```

### 2. Enable Gzip Compression

**In `app/main.py`:**
```python
from fastapi.middleware.gzip import GZipMiddleware

app.add_middleware(GZipMiddleware, minimum_size=1000)
```

### 3. Add Health Check Endpoint for Railway

Railway pings `/` by default. Make sure it returns quickly:

```python
# app/main.py
@app.get("/")
async def root():
    return {"status": "ok", "service": "RAG API"}
```

---

## Troubleshooting

### Issue: Build fails with "No module named 'app'"

**Solution:**
```bash
# Check directory structure
# Procfile should be in backend/
# app/ should be in backend/app/

# Correct structure:
backend/
├── app/
│   ├── __init__.py
│   ├── main.py
│   └── ...
├── Procfile
└── requirements.txt
```

### Issue: "Application startup failed"

**Solution:**
Check Railway logs for errors:
```
# Common issues:
1. Missing environment variable
2. Wrong Python version
3. Dependency installation failed

# Fix: Add all env vars in Railway dashboard
```

### Issue: CORS errors from frontend

**Solution:**
```bash
# Update ALLOWED_ORIGINS to include frontend domain
ALLOWED_ORIGINS=["https://your-site.vercel.app"]

# Restart deployment (edit any variable to trigger)
```

### Issue: Slow response times (>5s)

**Solution:**
```bash
# Check Railway region vs Qdrant region
# Should be in same or nearby regions

# Enable response caching
EMBEDDING_CACHE_SIZE=1000

# Reduce vector search
VECTOR_SEARCH_LIMIT=3
```

### Issue: 500 hours limit exceeded

**Solutions:**
1. **Pause service** when not actively developing
   - Railway dashboard → Service → Settings → Pause
2. **Use Render** for overflow (free with cold starts)
3. **Upgrade to Hobby** ($5/month) before presentation

---

## Cost Analysis

### Free Tier Usage

**Scenario 1: Development (Paused when not in use)**
- Active: ~8 hours/day × 30 days = 240 hours ✅
- Cost: $0

**Scenario 2: Always On (24/7)**
- Active: 720 hours/month ❌ Exceeds 500 hours
- Overage: 220 hours
- Extra cost: ~$11

**Scenario 3: Hackathon (1 week always-on)**
- Active: 168 hours ✅ Well under limit
- Cost: $0

### Recommendations

**For Hackathon:**
1. Deploy **1 week before** deadline
2. Run **24/7** during final week (168 hours)
3. Pause after submission
4. Resume before presentation
5. **Total cost: $0**

**After Hackathon:**
- Hobby plan: $5/month
- Or use Render free tier with cold starts

---

## Alternative: Render (Backup)

If Railway doesn't work:

1. Go to **https://render.com**
2. Create "Web Service"
3. Connect GitHub repo
4. Configure:
   - Environment: Python 3
   - Build: `pip install -r requirements.txt`
   - Start: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`
5. Add environment variables
6. Deploy

**Downside**: 15-minute cold start after inactivity

---

## Deployment Checklist

Before presenting:

- [ ] Backend deployed to Railway
- [ ] Health endpoint returns 200
- [ ] RAG query works (test with curl)
- [ ] CORS configured correctly
- [ ] Frontend connected to Railway backend
- [ ] Response time <3s verified
- [ ] Logs show no errors
- [ ] Usage under 500 hours (or upgraded plan)

---

## Quick Commands Reference

```bash
# Test health
curl https://your-backend.up.railway.app/api/v1/health

# Test query
curl -X POST https://your-backend.up.railway.app/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'

# Test CORS
curl -X OPTIONS https://your-backend.up.railway.app/api/v1/query \
  -H "Origin: https://your-frontend.com" \
  -H "Access-Control-Request-Method: POST" \
  -v

# Check response time
time curl -X POST https://your-backend.up.railway.app/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

---

**Status**: ✅ Backend deployed to Railway
**Next**: Deploy frontend to GitHub Pages/Vercel (see SETUP-FRONTEND-DEPLOY.md)
