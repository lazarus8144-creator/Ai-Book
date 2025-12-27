# ✅ Quick Deployment Checklist

## 🎯 What You Need (5 minutes setup)

1. **GitHub Account** - https://github.com/signup
2. **Railway Account** - https://railway.app (sign up with GitHub)
3. **Groq API Key** - https://console.groq.com (free, no credit card!)
4. **Vercel Account** - https://vercel.com (sign up with GitHub)

Your Qdrant database is already configured! ✅

---

## 📋 Deployment Steps (Copy-Paste Ready)

### Step 1: Create GitHub Repo (2 minutes)

1. Go to: https://github.com/new
2. Repository name: `ai-textbook-chatbot`
3. Make it **Public**
4. Click "Create repository"

### Step 2: Push Code (30 seconds)

```bash
# Replace YOUR_USERNAME with your GitHub username
git remote add origin https://github.com/YOUR_USERNAME/ai-textbook-chatbot.git
git push -u origin main
```

### Step 3: Get Groq API Key (1 minute)

1. Go to: https://console.groq.com
2. Sign up (email + password)
3. Click "API Keys" → "Create API Key"
4. Copy the key (starts with `gsk_`)
5. **Save it somewhere** - you'll need it for Railway

### Step 4: Deploy Backend to Railway (3 minutes)

1. Go to: https://railway.app/new
2. Click "Deploy from GitHub repo"
3. Select `ai-textbook-chatbot`
4. Go to Settings → Set **Root Directory** to: `backend`
5. Go to Variables → Click "Add Variable" → "Add All" → Paste:

```env
GROQ_API_KEY=gsk_PASTE_YOUR_KEY_HERE
QDRANT_URL=YOUR_QDRANT_URL
QDRANT_API_KEY=YOUR_QDRANT_KEY
QDRANT_COLLECTION_NAME=textbook_chunks
ENVIRONMENT=production
LOG_LEVEL=INFO
ADMIN_TOKEN=change-this-to-random-32-chars
ALLOWED_ORIGINS=["https://your-app.vercel.app"]
RATE_LIMIT_QUERIES_PER_MINUTE=20
RATE_LIMIT_QUERIES_PER_HOUR=100
EMBEDDING_CACHE_SIZE=1000
VECTOR_SEARCH_LIMIT=5
MAX_CHUNK_SIZE=800
CHUNK_OVERLAP=100
```

6. **Important**: Update these values:
   - `QDRANT_URL` - your Qdrant cluster URL (from backend/.env)
   - `QDRANT_API_KEY` - your Qdrant API key (from backend/.env)
   - `ADMIN_TOKEN` - generate random 32 characters

7. Wait for deployment (~3 minutes)

8. Copy your Railway URL (looks like: `https://xxx.up.railway.app`)

9. Test it:
```bash
curl https://YOUR_RAILWAY_URL/api/v1/health
```

Should return:
```json
{"status":"healthy","qdrant_connected":true,"groq_available":true}
```

### Step 5: Deploy Frontend to Vercel (2 minutes)

1. Go to: https://vercel.com/new
2. Click "Import" on your `ai-textbook-chatbot` repo
3. Configure:
   - **Framework Preset**: Other
   - **Root Directory**: `textbook`
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`

4. Add Environment Variable:
   - Name: `REACT_APP_API_URL`
   - Value: `https://YOUR_RAILWAY_URL` (from Step 4.8)

5. Click "Deploy"

6. Wait 2-3 minutes

7. Copy your Vercel URL (looks like: `https://ai-textbook-chatbot.vercel.app`)

### Step 6: Update CORS (1 minute)

1. Go back to Railway → Your Project → Variables
2. Find `ALLOWED_ORIGINS`
3. Update to: `["https://YOUR_VERCEL_URL"]`
4. Railway will auto-redeploy

### Step 7: Test Everything! (1 minute)

1. Open your Vercel URL in browser
2. Click the chatbot button (bottom-right)
3. Ask: "What is ROS 2?"
4. You should get an answer with source links!

---

## 🎉 You're Done!

Your AI textbook with RAG chatbot is now live at:
- **Frontend**: `https://your-app.vercel.app`
- **Backend**: `https://your-app.up.railway.app`
- **API Docs**: `https://your-app.up.railway.app/docs`

---

## 🚨 Troubleshooting

**Backend health check fails:**
```bash
# Check Railway logs
# Railway Dashboard → Deployments → Click latest → View Logs

# Common issues:
# 1. Missing environment variables
# 2. Wrong QDRANT_URL or QDRANT_API_KEY
# 3. First startup takes 30-60s (loading embedding model)
```

**Chatbot not responding:**
```bash
# 1. Check browser console (F12) for errors
# 2. Verify REACT_APP_API_URL in Vercel settings
# 3. Test backend directly:
curl -X POST https://YOUR_RAILWAY_URL/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question":"test"}'
```

**CORS error:**
```bash
# Verify ALLOWED_ORIGINS in Railway exactly matches Vercel URL
# Must include https:// and no trailing slash
```

---

## 📊 What You're Getting (ALL FREE!)

✅ **Backend** - FastAPI + RAG pipeline on Railway ($5/mo credit)
✅ **Frontend** - Docusaurus textbook on Vercel (unlimited)
✅ **LLM** - Groq API (14,400 free requests/day)
✅ **Embeddings** - Local sentence-transformers (unlimited)
✅ **Vector DB** - Qdrant Cloud (1GB free)

**Total cost: $0/month** 🎉

---

## 📝 Quick Reference

| What | Where | URL |
|------|-------|-----|
| Backend Code | `/backend` | https://github.com/YOUR_USERNAME/ai-textbook-chatbot |
| Frontend Code | `/textbook` | Same repo |
| Backend Deploy | Railway | https://railway.app |
| Frontend Deploy | Vercel | https://vercel.com |
| Backend API | Production | https://YOUR_APP.up.railway.app |
| Frontend Site | Production | https://YOUR_APP.vercel.app |
| API Docs | Swagger | https://YOUR_APP.up.railway.app/docs |
| Groq Console | API Keys | https://console.groq.com |
| Qdrant Dashboard | Data | https://cloud.qdrant.io |

---

**Time to deploy: ~10 minutes**
**Monthly cost: $0**
**Difficulty: Easy** 🟢

**Questions? Check DEPLOYMENT-GUIDE.md for detailed troubleshooting!**
