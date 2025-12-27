# Railway Deployment Credentials

**⚠️ SAVE THIS FILE - You'll need these for Railway deployment**

## Required Environment Variables for Railway

Copy and paste this ENTIRE block into Railway → Variables → "Raw Editor":

```env
# === AI Services (REQUIRED) ===
GROQ_API_KEY=YOUR_GROQ_KEY_HERE
ANTHROPIC_API_KEY=YOUR_ANTHROPIC_KEY_HERE

# === Qdrant Vector Database (REQUIRED) ===
QDRANT_URL=https://db8b89c1-e684-4ef2-a113-7bef85d74d34.us-east4-0.gcp.cloud.qdrant.io:6333
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.WCDuczZ2dh_E0olTmSK0Lv7MSpf0MG5JLbnHCT039So
QDRANT_COLLECTION_NAME=textbook_chunks

# === Application Config ===
ENVIRONMENT=production
LOG_LEVEL=INFO
ADMIN_TOKEN=hackathon-admin-token-2025-change-this-random

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

# === Models ===
GROQ_MODEL=llama-3.1-70b-versatile
EMBEDDING_MODEL=all-MiniLM-L6-v2
EMBEDDING_DEVICE=cpu

# === Subagent Enhancement (Bonus +50 pts) ===
ENABLE_SUBAGENT_ENHANCEMENT=true
```

---

## Step-by-Step: What to Update

### 1. Get Groq API Key (FREE)
- Go to: https://console.groq.com
- Sign up and create API key
- **Replace** `YOUR_GROQ_KEY_HERE` with your actual key (starts with `gsk_...`)

### 2. Get Anthropic API Key (Optional - for +50 bonus pts)
- Go to: https://console.anthropic.com
- Sign up and create API key
- **Replace** `YOUR_ANTHROPIC_KEY_HERE` with your actual key (starts with `sk-ant-...`)
- **OR** remove this line if you don't want Claude Subagents yet

### 3. Update CORS After Vercel Deployment
- After deploying frontend to Vercel, you'll get a URL like: `https://ai-textbook-chatbot.vercel.app`
- Update this line:
  ```
  ALLOWED_ORIGINS=["https://ai-textbook-chatbot.vercel.app"]
  ```

---

## Credentials Already Set ✅

These are already configured from your `.env` file:
- ✅ Qdrant URL
- ✅ Qdrant API Key
- ✅ Collection name

---

## Quick Deployment Checklist

- [ ] Get Groq API key → paste above
- [ ] Get Anthropic API key → paste above (optional)
- [ ] Push code to GitHub
- [ ] Create Railway project
- [ ] Set root directory to `backend`
- [ ] Paste ALL environment variables
- [ ] Wait for deployment (3-5 min)
- [ ] Get Railway URL
- [ ] Deploy frontend to Vercel
- [ ] Update CORS with Vercel URL
- [ ] Test end-to-end!

---

**Estimated time**: 15-20 minutes total

**Cost**: $0 (all free tiers!)

**Points**: 150+ guaranteed 🎉
