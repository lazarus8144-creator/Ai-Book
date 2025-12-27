# 🚀 Quick Start Guide: RAG Chatbot Implementation

This guide will get your RAG chatbot up and running in **under 30 minutes**.

---

## ✅ Prerequisites

- Python 3.10+ installed
- Node.js 18+ installed
- OpenAI API key (from hackathon credits)
- Qdrant Cloud account (free tier)

---

## 📦 Step 1: Backend Setup (10 minutes)

### 1.1 Create Qdrant Cloud Cluster

```bash
# 1. Sign up at https://cloud.qdrant.io (free)
# 2. Create cluster:
#    - Name: textbook-rag
#    - Tier: Free (1GB)
#    - Region: Closest to you
# 3. Copy cluster URL and API key
```

### 1.2 Configure Backend

```bash
cd backend

# Create virtual environment
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
pip install -r requirements-dev.txt  # For testing

# Create .env file
cp .env.example .env

# Edit .env with your credentials
nano .env
```

**Required `.env` values:**
```bash
OPENAI_API_KEY=sk-your-key-here
QDRANT_URL=https://xxx-yyy.qdrant.io
QDRANT_API_KEY=your-qdrant-key
QDRANT_COLLECTION_NAME=textbook_chunks
ALLOWED_ORIGINS=["http://localhost:3000"]

# Optional: Enable subagent enhancement (+50 points)
# ANTHROPIC_API_KEY=your-anthropic-key
# ENABLE_SUBAGENT_ENHANCEMENT=true
```

### 1.3 Test Backend

```bash
# Start server
uvicorn app.main:app --reload --port 8000

# Open http://localhost:8000/docs in browser
# You should see FastAPI Swagger documentation
```

---

## 📚 Step 2: Document Ingestion (5 minutes)

```bash
# In a new terminal (backend/ directory)
source venv/bin/activate

# Run ingestion script
python scripts/ingest_docs.py \
    --docs-path ../textbook/docs \
    --batch-size 50 \
    --verbose

# Expected output:
# ======================================================================
# 📚 Document Ingestion Pipeline
# ======================================================================
#
# 🔍 Searching for markdown files in: ../textbook/docs
# ✅ Found 30 files to process
#
# 📄 Processing files into chunks...
# Processing files: 100%|██████████| 30/30 [00:15<00:00]
# ✅ Created 2156 total chunks
#
# 🧠 Generating embeddings (batch size: 50)...
# Embedding batches: 100%|██████████| 44/44 [01:23<00:00]
# ✅ Generated 2156 embeddings
#
# ☁️  Uploading to Qdrant...
# Uploading batches: 100%|██████████| 44/44 [00:12<00:00]
#
# ======================================================================
# ✅ Ingestion Complete!
# ======================================================================
# 📁 Files processed: 30
# 📦 Chunks created: 2156
# 🎯 Vectors in Qdrant: 2156
# ⏱️  Time taken: 112.3s
# ======================================================================
```

### 2.1 Test RAG Query

```bash
# Test with curl
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'

# Expected response:
# {
#   "answer": "ROS 2 (Robot Operating System 2) is an open-source framework...",
#   "sources": [
#     {
#       "module": "module-1-ros2",
#       "title": "Introduction to ROS 2",
#       "url": "https://your-project.vercel.app/module-1-ros2/01-introduction",
#       "score": 0.89
#     }
#   ],
#   "response_time_ms": 2341,
#   "tokens_used": 456
# }
```

---

## 🎨 Step 3: Frontend Integration (10 minutes)

### 3.1 Configure Frontend

```bash
cd ../textbook

# Install dependencies (if not already done)
npm install

# Create environment file
cp .env.local.example .env.local

# Edit .env.local
nano .env.local
```

**`.env.local` content:**
```bash
REACT_APP_API_URL=http://localhost:8000
```

### 3.2 Add ChatbotWidget

The ChatbotWidget component has already been created in your earlier response. Just verify these files exist:

```bash
# Check files exist
ls src/components/ChatbotWidget/index.tsx
ls src/components/ChatbotWidget/styles.module.css
ls src/theme/Root.tsx
```

If not, copy them from the code provided in the comprehensive implementation guide.

### 3.3 Test Frontend

```bash
# Start development server
npm run start

# Open http://localhost:3000 in browser
# You should see:
# 1. Textbook loads normally
# 2. Chatbot floating button in bottom-right (💬)
# 3. Click to open chat window
# 4. Ask "What is ROS 2?" and get a response
```

---

## 🧪 Step 4: Testing (5 minutes)

### 4.1 Manual Testing Checklist

- [ ] Chatbot opens/closes smoothly
- [ ] Ask "What is ROS 2?" - gets correct answer
- [ ] Sources shown with clickable links
- [ ] Links navigate to correct chapters
- [ ] Dark mode toggle works (chatbot adapts)
- [ ] Mobile responsive (resize browser to 320px)
- [ ] Ask off-topic question - gets "not covered" response

### 4.2 Performance Testing

```bash
# Test response time
time curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'

# Should complete in <3 seconds

# Run Lighthouse audit
npm run build
npx lighthouse http://localhost:3000 --view

# Target scores:
# - Performance: >90
# - Accessibility: >90
```

---

## 🎁 Step 5: Enable Bonus Features (Optional)

### Option A: Claude Subagents (+50 points)

```bash
# 1. Get Anthropic API key from https://console.anthropic.com

# 2. Add to backend/.env
echo 'ANTHROPIC_API_KEY=your-anthropic-key' >> .env
echo 'ENABLE_SUBAGENT_ENHANCEMENT=true' >> .env

# 3. Install anthropic library
pip install anthropic==0.18.1

# 4. Restart backend server
# (Ctrl+C and run uvicorn again)

# 5. Test subagent enhancement
python scripts/test_subagent.py --compare
```

**What you get:**
- ✅ Technical accuracy verification (0-100 score)
- ✅ Automatic code examples when relevant
- ✅ Follow-up question suggestions
- ✅ Enhanced answer quality

### Option B: Text Selection Query (Quick Win)

Already implemented in the ChatbotWidget component!

**How to use:**
1. Go to any textbook page
2. Select text with your mouse (e.g., "ROS 2 node")
3. Click "Ask about selection" button that appears
4. Chatbot opens with pre-filled question

### Option C: Personalization Buttons (+50 points)

See the `PersonalizeButton` component code provided earlier. Requires:
- Backend endpoint: `POST /api/v1/personalize`
- User profile storage (Neon Postgres in Phase 2)

---

## 🚀 Step 6: Deployment (Railway + GitHub Pages)

### 6.1 Deploy Backend to Railway

```bash
# 1. Push backend to GitHub
cd backend
git init
git add .
git commit -m "Initial backend implementation"
git remote add origin https://github.com/yourusername/ai-textbook-backend.git
git push -u origin main

# 2. Go to https://railway.app
# 3. Click "New Project" → "Deploy from GitHub"
# 4. Select your backend repo
# 5. Add environment variables in Railway dashboard:
#    - OPENAI_API_KEY
#    - QDRANT_URL
#    - QDRANT_API_KEY
#    - ALLOWED_ORIGINS=["https://your-project.vercel.app"]
#    - All other .env variables

# 6. Wait for deployment (~2 minutes)
# 7. Test health endpoint:
curl https://your-backend.railway.app/api/v1/health
```

### 6.2 Deploy Frontend to GitHub Pages

```bash
cd ../textbook

# 1. Update .env for production
# Create .env.production
echo 'REACT_APP_API_URL=https://your-backend.railway.app' > .env.production

# 2. Build production bundle
npm run build

# 3. Deploy to GitHub Pages
npm run deploy  # Or use your existing GitHub Pages workflow
```

---

## 📊 Success Validation

### Phase 1 Checklist (100 points)

- [ ] ✅ RAG chatbot responds in <3s (p95)
- [ ] ✅ Answers cite textbook sources with URLs
- [ ] ✅ Mobile responsive (320px-2048px)
- [ ] ✅ Dark mode support
- [ ] ✅ Accessibility score ≥90 (Lighthouse)
- [ ] ✅ Deployed to production

### Phase 3 Bonus Checklist (50 points)

- [ ] ✅ Claude Subagent enhancement working
- [ ] ✅ Accuracy scores displayed (>80 average)
- [ ] ✅ Code examples added when relevant
- [ ] ✅ Follow-up questions suggested
- [ ] ✅ Metrics tracked and visible

---

## 🔥 Common Issues & Solutions

### Issue: Ingestion fails with "OpenAI API error"

**Solution:**
```bash
# Check API key is correct
echo $OPENAI_API_KEY

# Test API directly
curl https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY"
```

### Issue: Chatbot shows "CORS error"

**Solution:**
```bash
# Backend .env must include frontend origin
ALLOWED_ORIGINS=["http://localhost:3000","https://your-project.vercel.app"]

# Restart backend after changing .env
```

### Issue: No relevant results found

**Solution:**
```bash
# Re-run ingestion with verbose flag
python scripts/ingest_docs.py --docs-path ../textbook/docs --verbose

# Check Qdrant dashboard for vector count
# Should have ~2000+ vectors
```

### Issue: Response time >3s

**Solutions:**
1. Reduce `VECTOR_SEARCH_LIMIT` from 5 to 3 in `.env`
2. Reduce `OPENAI_MAX_TOKENS` from 500 to 300
3. Check Railway server location (should be close to Qdrant region)

---

## 📚 Additional Resources

- **RAG Spec**: `specs/002-rag-chatbot/spec.md`
- **Architecture Plan**: `specs/002-rag-chatbot/plan.md`
- **Implementation Tasks**: `specs/002-rag-chatbot/tasks.md`
- **Constitution**: `.specify/memory/constitution.md`

---

## 🎯 Next Steps

1. ✅ Complete Phase 1 (RAG chatbot) - **Required for 100 points**
2. 🚀 Deploy to production (Railway + GitHub Pages)
3. 🎁 Add bonus features:
   - Claude Subagents (+50 points) ← **Easiest bonus**
   - Content Personalization (+50 points)
   - Urdu Translation (+50 points)
4. 🧪 Run full E2E test suite
5. 📹 Record demo video for presentation

---

**Time to Complete Phase 1**: ~3-4 hours
**Total Possible Score**: 300 points (100 required + 200 bonus)

**Good luck! 🚀**
