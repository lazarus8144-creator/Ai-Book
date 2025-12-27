# Railway Deployment Guide

## Prerequisites

1. **Railway Account**: Sign up at https://railway.app
2. **Groq API Key**: Get free API key at https://console.groq.com
3. **Qdrant Cloud**: Already configured (from .env)

## Deployment Steps

### 1. Install Railway CLI (Optional)

```bash
npm install -g @railway/cli
railway login
```

### 2. Deploy via Railway Dashboard (Recommended)

#### Option A: Deploy from GitHub

1. Push your code to GitHub
2. Go to https://railway.app/new
3. Click "Deploy from GitHub repo"
4. Select your repository
5. Railway will auto-detect the Python app

#### Option B: Deploy from CLI

```bash
cd backend
railway init
railway up
```

### 3. Configure Environment Variables

In Railway dashboard, go to your project → Variables and add:

```env
# Required
GROQ_API_KEY=gsk_your_groq_api_key_here
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=textbook_chunks

# Application
ENVIRONMENT=production
LOG_LEVEL=INFO
ADMIN_TOKEN=your-secure-random-token-min-32-chars

# CORS - Update with your Vercel domain
ALLOWED_ORIGINS=["https://your-textbook.vercel.app"]

# Rate Limiting
RATE_LIMIT_QUERIES_PER_MINUTE=20
RATE_LIMIT_QUERIES_PER_HOUR=100

# Performance
EMBEDDING_CACHE_SIZE=1000
VECTOR_SEARCH_LIMIT=5
MAX_CHUNK_SIZE=800
CHUNK_OVERLAP=100
```

### 4. Get Your Railway URL

After deployment, Railway will provide a URL like:
```
https://your-app-production.up.railway.app
```

Test it:
```bash
curl https://your-app-production.up.railway.app/api/v1/health
```

### 5. Update Frontend

Update `textbook/.env.local`:
```env
REACT_APP_API_URL=https://your-app-production.up.railway.app
```

## Free Tier Limits

Railway free tier includes:
- $5 credit per month
- ~500 hours of runtime
- Perfect for this chatbot!

## Troubleshooting

### Check Logs
```bash
railway logs
```

### Common Issues

1. **Port Error**: Railway auto-sets `$PORT` - the Procfile handles this
2. **Health Check Fails**: Increase `healthcheckTimeout` in railway.json
3. **Out of Memory**: The sentence-transformers model needs ~2GB RAM (Railway provides 8GB)

## Cost Optimization

Using Groq (free) + sentence-transformers (local) means:
- ✅ No OpenAI costs
- ✅ Unlimited embeddings (local)
- ✅ Free LLM API (Groq)
- ✅ Only cost: Railway hosting (~$5/month or free tier)

## Monitoring

Check your deployment:
- Health: `https://your-app.railway.app/api/v1/health`
- API Docs: `https://your-app.railway.app/docs`
- Metrics: Railway dashboard → Metrics tab
