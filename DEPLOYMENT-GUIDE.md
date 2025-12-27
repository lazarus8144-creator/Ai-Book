# 🚀 Production Deployment Guide

## Overview

This guide will help you deploy the Physical AI Textbook with RAG Chatbot to production using:
- **Backend**: Railway (free tier)
- **Frontend**: Vercel or GitHub Pages (free)
- **Database**: Qdrant Cloud (already configured)
- **LLM**: Groq API (free tier)

## ✅ Pre-Deployment Checklist

- [x] Backend running locally (`http://localhost:8000`)
- [x] Frontend running locally (`http://localhost:3000`)
- [x] Qdrant Cloud configured with data
- [x] Git repository initialized
- [ ] GitHub repository created
- [ ] Groq API key obtained
- [ ] Railway account created
- [ ] Vercel account created (optional)

---

## Step 1: Push to GitHub

### 1.1 Create GitHub Repository

1. Go to https://github.com/new
2. Create a new repository (e.g., `ai-textbook-chatbot`)
3. **Do NOT** initialize with README (we already have code)
4. Make it **public** (required for free Vercel deployment)

### 1.2 Push Code

```bash
# Add GitHub remote (replace with your username/repo)
git remote add origin https://github.com/YOUR_USERNAME/ai-textbook-chatbot.git

# Push code
git push -u origin main
```

---

## Step 2: Deploy Backend to Railway

### 2.1 Sign Up for Railway

1. Go to https://railway.app
2. Sign up with GitHub
3. You get **$5 free credit/month** (enough for this app!)

### 2.2 Create New Project

1. Click "New Project"
2. Select "Deploy from GitHub repo"
3. Choose your repository
4. Railway will detect the FastAPI app automatically

### 2.3 Configure Root Directory

Since the backend is in `/backend` subfolder:

1. Go to your Railway project → Settings
2. Set **Root Directory**: `backend`
3. Railway will now look for `Procfile`, `requirements.txt`, etc. in `/backend`

### 2.4 Add Environment Variables

In Railway dashboard → Variables, add these:

```env
# === Required ===
GROQ_API_KEY=gsk_your_groq_api_key_here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=textbook_chunks

# === Application ===
ENVIRONMENT=production
LOG_LEVEL=INFO
ADMIN_TOKEN=your-secure-random-32-char-token

# === CORS (Update after Vercel deployment) ===
ALLOWED_ORIGINS=["https://your-textbook.vercel.app"]

# === Rate Limiting ===
RATE_LIMIT_QUERIES_PER_MINUTE=20
RATE_LIMIT_QUERIES_PER_HOUR=100

# === Performance ===
EMBEDDING_CACHE_SIZE=1000
VECTOR_SEARCH_LIMIT=5
MAX_CHUNK_SIZE=800
CHUNK_OVERLAP=100
```

### 2.5 Get Groq API Key

1. Go to https://console.groq.com
2. Sign up (free tier: 14,400 requests/day!)
3. Create API key
4. Copy the key (starts with `gsk_`)
5. Add to Railway as `GROQ_API_KEY`

### 2.6 Deploy & Test

1. Railway will auto-deploy (takes ~3-5 minutes)
2. Get your Railway URL: `https://your-app-production.up.railway.app`
3. Test health endpoint:

```bash
curl https://your-app-production.up.railway.app/api/v1/health
```

Expected response:
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "groq_available": true,
  "embeddings_loaded": true,
  "version": "1.0.0-free"
}
```

4. Test RAG query:

```bash
curl -X POST https://your-app-production.up.railway.app/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

---

## Step 3: Deploy Frontend to Vercel

### 3.1 Sign Up for Vercel

1. Go to https://vercel.com/signup
2. Sign up with GitHub
3. Free tier is perfect for this app!

### 3.2 Import Project

1. Click "Add New..." → "Project"
2. Import your GitHub repository
3. Configure project settings:

```
Framework Preset: Docusaurus
Root Directory: textbook
Build Command: npm run build
Output Directory: build
Install Command: npm install
```

### 3.3 Add Environment Variable

Before deploying, add this environment variable in Vercel:

```
REACT_APP_API_URL=https://your-app-production.up.railway.app
```

(Replace with your actual Railway URL from Step 2.6)

### 3.4 Deploy

1. Click "Deploy"
2. Wait 2-3 minutes
3. You'll get a URL like: `https://ai-textbook-chatbot.vercel.app`

---

## Step 4: Update CORS in Railway

Now that you have your Vercel URL, update the backend CORS:

1. Go to Railway → Your Project → Variables
2. Update `ALLOWED_ORIGINS`:

```env
ALLOWED_ORIGINS=["https://ai-textbook-chatbot.vercel.app"]
```

3. Railway will auto-redeploy

---

## Step 5: Test Production System

### 5.1 Test Frontend

1. Open your Vercel URL: `https://your-textbook.vercel.app`
2. Look for the chatbot button (bottom right)
3. Click to open chatbot
4. Ask a question: "What is ROS 2?"
5. Verify you get an answer with sources

### 5.2 Test Backend Directly

```bash
# Health check
curl https://your-railway-app.up.railway.app/api/v1/health

# API docs
open https://your-railway-app.up.railway.app/docs

# Test query
curl -X POST https://your-railway-app.up.railway.app/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "How do I debug ROS 2 nodes?"}'
```

---

## Alternative: Deploy Frontend to GitHub Pages

If you prefer GitHub Pages over Vercel:

### Update `textbook/docusaurus.config.ts`:

```typescript
const config: Config = {
  // ...
  url: 'https://YOUR_USERNAME.github.io',
  baseUrl: '/ai-textbook-chatbot/',  // Your repo name
  organizationName: 'YOUR_USERNAME',
  projectName: 'ai-textbook-chatbot',
  // ...
};
```

### Deploy:

```bash
cd textbook
GIT_USER=YOUR_USERNAME npm run deploy
```

---

## 📊 Cost Breakdown (FREE!)

| Service | Free Tier | Usage | Cost |
|---------|-----------|-------|------|
| Railway | $5/month credit | Backend hosting | **$0** |
| Groq | 14,400 req/day | LLM API | **$0** |
| Qdrant Cloud | 1GB free | Vector DB | **$0** |
| Vercel | Unlimited | Frontend hosting | **$0** |
| sentence-transformers | Local | Embeddings | **$0** |
| **Total** | | | **$0/month** |

---

## 🔧 Troubleshooting

### Backend Issues

**"Qdrant not connected"**
- Check `QDRANT_URL` and `QDRANT_API_KEY` in Railway variables
- Verify Qdrant cluster is running in Qdrant Cloud dashboard

**"Groq API error"**
- Verify `GROQ_API_KEY` is correct
- Check you haven't exceeded free tier limits (14.4k/day)
- View logs: Railway dashboard → Deployments → View Logs

**"Health check timeout"**
- sentence-transformers model takes ~30s to load on first request
- Railway's health check waits 5 minutes (configured in `railway.json`)
- Check logs for "Application startup complete"

### Frontend Issues

**"Chatbot not appearing"**
- Check browser console for errors
- Verify `REACT_APP_API_URL` is set correctly in Vercel
- Redeploy: Vercel → Deployments → Redeploy

**"CORS error"**
- Update `ALLOWED_ORIGINS` in Railway to include your Vercel URL
- Must be exact match including `https://`

**"Network error when querying"**
- Test backend health directly: `curl https://your-backend/api/v1/health`
- Check Railway logs for errors
- Verify API URL in frontend environment variables

### Performance Issues

**"Slow first response"**
- sentence-transformers loads on first request (~30s)
- Subsequent requests are fast (<2s)
- Consider keeping Railway app "awake" with uptime monitor

**"Railway out of credit"**
- Check usage: Railway → Project → Usage
- Free tier = $5/month (~500 hours)
- Optimize: Use sleep schedules if not 24/7

---

## 📈 Monitoring

### Railway Metrics
- Dashboard → Metrics tab
- Track: CPU, memory, network
- Free tier: 512MB RAM, 1 vCPU (sufficient!)

### Groq Usage
- https://console.groq.com → Usage
- Monitor requests/day (limit: 14,400)
- Track which models used

### Vercel Analytics
- Dashboard → Your Project → Analytics
- Track visitors, page views, performance

---

## 🎯 Next Steps

1. **Custom Domain** (optional)
   - Add custom domain in Vercel settings
   - Update CORS in Railway

2. **Monitoring** (optional)
   - Add uptime monitoring (UptimeRobot, Better Uptime)
   - Set up error tracking (Sentry)

3. **Content Updates**
   - Edit textbook content in `textbook/docs/`
   - Run `backend/scripts/ingest_docs.py` to update vector DB
   - Commit and push → auto-deploys

4. **Analytics** (optional)
   - Add Google Analytics to Docusaurus
   - Track chatbot usage in backend logs

---

## 📝 Deployment Checklist

Backend (Railway):
- [ ] Project deployed from GitHub
- [ ] Root directory set to `backend`
- [ ] All environment variables configured
- [ ] Groq API key added
- [ ] Qdrant credentials added
- [ ] Health check passing
- [ ] Test query successful
- [ ] Railway URL noted

Frontend (Vercel):
- [ ] Project imported from GitHub
- [ ] Root directory set to `textbook`
- [ ] `REACT_APP_API_URL` configured
- [ ] Build successful
- [ ] Deployment live
- [ ] Chatbot widget visible
- [ ] Test query from chatbot works

Integration:
- [ ] CORS updated in Railway with Vercel URL
- [ ] End-to-end test: Question → Answer → Sources
- [ ] All 4 modules accessible
- [ ] Dark mode works
- [ ] Mobile responsive

---

## 🆘 Need Help?

- **Railway Docs**: https://docs.railway.app
- **Vercel Docs**: https://vercel.com/docs
- **Groq Docs**: https://console.groq.com/docs
- **Qdrant Docs**: https://qdrant.tech/documentation

---

**Generated with** [Claude Code](https://claude.com/claude-code)

Good luck with your deployment! 🚀
