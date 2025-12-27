# 🚀 START HERE: 5-Step Quick Start

Follow these 5 steps to get your RAG chatbot running in **under 1 hour**.

---

## ✅ **STEP 1: Read QUICKSTART.md** (5 minutes)

```bash
cat QUICKSTART.md
```

This gives you the big picture. Then come back here.

---

## 🔷 **STEP 2: Set Up Qdrant Cloud** (15 minutes)

### 2.1 Create Account
1. Open browser: **https://cloud.qdrant.io**
2. Click **"Sign Up"**
3. Use **GitHub** (fastest) or Google/Email
4. Verify email if needed

### 2.2 Create Cluster
1. Click **"+ New Cluster"** or **"Create Cluster"**
2. **Name**: `textbook-rag`
3. **Plan**: Select **"Free Tier"** (1 GB storage)
4. **Region**: Choose closest to you (e.g., US East, EU West, Asia Pacific)
5. Click **"Create"**
6. Wait 30-60 seconds until status = "Running" ✅

### 2.3 Get Credentials
1. Click on your cluster name
2. **Copy Cluster URL**:
   ```
   Example: https://abc123-def456.eu-central.aws.cloud.qdrant.io:6333
   ```

3. Click **"API Keys"** → **"Create API Key"**
4. Name: `rag-chatbot`
5. Click **"Create"**
6. **COPY THE KEY IMMEDIATELY** (shown only once!)
   ```
   Example: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
   ```

### 2.4 Save Credentials
Keep these in a note - you'll use them in Step 3.

**✅ Checkpoint**: You have Qdrant URL and API Key

---

## 🔐 **STEP 3: Configure Environment** (20 minutes)

### 3.1 Get OpenAI API Key

1. Go to **https://platform.openai.com/api-keys**
2. Log in with your account
3. Click **"+ Create new secret key"**
4. Name: `rag-chatbot-backend`
5. **Copy the key** (shown only once!)
   ```
   Starts with: sk-proj-...
   ```

### 3.2 Edit Backend .env File

```bash
cd backend
nano .env  # or: code .env
```

**Replace these 3 values:**
```bash
# Line 4: Paste your OpenAI key
OPENAI_API_KEY=sk-proj-YOUR-ACTUAL-KEY-HERE

# Line 14: Paste your Qdrant URL
QDRANT_URL=https://YOUR-CLUSTER.qdrant.io:6333

# Line 15: Paste your Qdrant API key
QDRANT_API_KEY=YOUR-QDRANT-KEY-HERE
```

**Save and exit** (Ctrl+X, then Y, then Enter in nano)

### 3.3 Verify Configuration

```bash
# Test that .env loads correctly
python3 -c "
from app.config import settings
print('✅ OpenAI Key:', settings.OPENAI_API_KEY[:10] + '...')
print('✅ Qdrant URL:', settings.QDRANT_URL)
print('✅ Environment:', settings.ENVIRONMENT)
"
```

**Expected output:**
```
✅ OpenAI Key: sk-proj-ab...
✅ Qdrant URL: https://abc123...
✅ Environment: development
```

**✅ Checkpoint**: Environment configured correctly

---

## 📦 **STEP 4: Run Ingestion** (5 minutes)

### 4.1 Install Dependencies

```bash
cd backend  # Make sure you're in backend/

# Create virtual environment
python3 -m venv venv

# Activate it
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install packages
pip install -r requirements.txt

# This takes ~2-3 minutes
```

### 4.2 Run Ingestion Script

```bash
# Still in backend/, with venv activated
python scripts/ingest_docs.py \
    --docs-path ../textbook/docs \
    --batch-size 50 \
    --verbose
```

**What you'll see:**
```
======================================================================
📚 Document Ingestion Pipeline
======================================================================

🔍 Searching for markdown files in: ../textbook/docs
✅ Found 30 files to process

📄 Processing files into chunks...
Processing files: 100%|████████████████| 30/30 [00:15<00:00]
✅ Created 2156 total chunks
📊 Avg chunk size: 487 tokens

🧠 Generating embeddings (batch size: 50)...
Embedding batches: 100%|████████████████| 44/44 [01:23<00:00]
✅ Generated 2156 embeddings

☁️  Uploading to Qdrant...
Uploading batches: 100%|████████████████| 44/44 [00:12<00:00]

======================================================================
✅ Ingestion Complete!
======================================================================
📁 Files processed: 30
📦 Chunks created: 2156
🎯 Vectors in Qdrant: 2156
⏱️  Time taken: 112.3s
======================================================================

🎉 All done! Your RAG chatbot is ready to use.
```

**If you get errors:**
- "OPENAI_API_KEY not set" → Check Step 3.2
- "Connection refused" → Check Qdrant cluster is "Running"
- "Rate limit" → Wait 1 minute, try again

**✅ Checkpoint**: 2000+ chunks ingested successfully

---

## 🧪 **STEP 5: Test Locally** (2 minutes)

### 5.1 Start Backend Server

```bash
# In backend/, with venv activated
uvicorn app.main:app --reload --port 8000
```

**Expected output:**
```
🚀 Starting RAG API in development mode
📚 Qdrant: https://abc123...
🔧 CORS origins: ['http://localhost:3000']
INFO:     Started server process
INFO:     Uvicorn running on http://0.0.0.0:8000
```

**Keep this terminal running!**

### 5.2 Test Health Endpoint

Open **new terminal**:

```bash
curl http://localhost:8000/api/v1/health
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

### 5.3 Test RAG Query

```bash
curl -X POST http://localhost:8000/api/v1/query \
  -H "Content-Type: application/json" \
  -d '{"question": "What is ROS 2?"}'
```

**Expected response (in <3 seconds):**
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is an open-source framework for robot development...",
  "sources": [
    {
      "module": "module-1-ros2",
      "title": "Introduction to ROS 2",
      "url": "https://your-project.vercel.app/module-1-ros2/01-introduction",
      "score": 0.89
    }
  ],
  "response_time_ms": 2341,
  "tokens_used": 456
}
```

### 5.4 Test in Browser

Open browser: **http://localhost:8000/docs**

You'll see FastAPI interactive documentation. Try:
1. Click **"POST /api/v1/query"**
2. Click **"Try it out"**
3. Enter question: `What is ROS 2?`
4. Click **"Execute"**
5. See response!

**✅ Checkpoint**: RAG chatbot working locally!

---

## 🎉 **YOU'RE DONE!**

**What you've accomplished:**
- ✅ Qdrant Cloud cluster running
- ✅ Environment configured
- ✅ 2000+ document chunks ingested
- ✅ Backend API running locally
- ✅ RAG queries working in <3s

---

## 🚀 **What's Next?**

### Option A: Add Frontend (Recommended)

```bash
# New terminal
cd ../textbook

# Create environment file
echo 'REACT_APP_API_URL=http://localhost:8000' > .env.local

# Start frontend
npm run start

# Open http://localhost:3000
# Click chatbot button in bottom-right
# Ask "What is ROS 2?"
```

### Option B: Deploy to Production

Follow: `docs/SETUP-RAILWAY.md`

### Option C: Enable Subagents (+50 points)

1. Get Anthropic API key from https://console.anthropic.com
2. Add to `backend/.env`:
   ```bash
   ANTHROPIC_API_KEY=sk-ant-your-key
   ENABLE_SUBAGENT_ENHANCEMENT=true
   ```
3. Install: `pip install anthropic==0.18.1`
4. Restart backend (Ctrl+C, then uvicorn again)
5. Test: `python scripts/test_subagent.py --compare`

---

## 🐛 **Troubleshooting**

### "ModuleNotFoundError: No module named 'app'"

**Solution:**
```bash
# Make sure you're in backend/ directory
cd backend

# Make sure venv is activated (you should see (venv) in prompt)
source venv/bin/activate
```

### "OPENAI_API_KEY not set"

**Solution:**
```bash
# Check .env file exists
ls -la .env

# Check it has your key
grep OPENAI_API_KEY .env

# Should show: OPENAI_API_KEY=sk-proj-...
# If it shows YOUR_OPENAI_KEY_HERE, edit .env again
```

### "Connection to Qdrant failed"

**Solution:**
```bash
# Check Qdrant cluster status at https://cloud.qdrant.io
# Status should be "Running" (green)

# Check URL in .env is correct
grep QDRANT_URL .env

# Should match the URL from Qdrant dashboard
```

### "Rate limit exceeded"

**Solution:**
```bash
# Wait 60 seconds
sleep 60

# Try again with smaller batch
python scripts/ingest_docs.py \
    --docs-path ../textbook/docs \
    --batch-size 25  # Smaller batches
```

---

## 📚 **Reference**

- **Full guides**: `docs/` directory
- **Implementation checklist**: `docs/IMPLEMENTATION-CHECKLIST.md`
- **Bonus features**: `docs/BONUS-FEATURES.md`
- **Deployment**: `docs/SETUP-RAILWAY.md`

---

**Time to complete**: ~45 minutes
**Difficulty**: Easy (copy-paste commands)
**Result**: Working RAG chatbot locally

**Great job! You're ready to add the frontend or deploy! 🎯**
