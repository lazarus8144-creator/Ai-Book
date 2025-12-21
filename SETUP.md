# RAG Chatbot Setup Guide

Complete setup instructions for the Physical AI & Humanoid Robotics textbook with RAG chatbot.

## Prerequisites

- Node.js 20+ (for Docusaurus textbook)
- Python 3.11+ (for FastAPI backend)
- OpenAI API key with credits
- Qdrant Cloud account (free tier)

---

## Part 1: Backend Setup (RAG Chatbot API)

### 1. Navigate to backend directory

```bash
cd rag-backend
```

### 2. Create Python virtual environment

```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Configure environment variables

Create `.env` file from the example:

```bash
cp .env.example .env
```

Edit `.env` and add your credentials:

```env
# OpenAI Configuration
OPENAI_API_KEY=sk-your-actual-api-key-here
OPENAI_MODEL=gpt-4o-mini
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=robotics_textbook

# CORS Configuration
CORS_ORIGINS=http://localhost:3000,https://your-vercel-domain.vercel.app

# Environment
ENVIRONMENT=development
```

**How to get credentials:**

- **OpenAI API Key**: https://platform.openai.com/api-keys
- **Qdrant Cloud**: https://cloud.qdrant.io/ (sign up for free tier)
  - Create a cluster
  - Copy the URL and API key from the cluster dashboard

### 5. Run the backend server

```bash
python app/main.py
```

Or with uvicorn directly:

```bash
uvicorn app.main:app --reload --port 8000
```

The API will be available at: http://localhost:8000

- API Docs: http://localhost:8000/docs
- Health Check: http://localhost:8000/api/v1/health

### 6. Ingest textbook content

Run the ingestion script to load textbook content into Qdrant:

```bash
python scripts/ingest_docs.py --docs-path ../textbook/docs
```

Expected output:
```
✅ Ingestion Complete!
Files Processed: 30
Chunks Created: 250-400 (depends on content)
Embeddings Generated: 250-400
Vectors Upserted: 250-400
```

**Verify ingestion:**

```bash
curl http://localhost:8000/api/v1/collections/stats
```

Should return vector count > 0.

### 7. Test the chatbot API

```bash
curl -X POST http://localhost:8000/api/v1/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "mode": "full_book"
  }'
```

Should return an answer with citations.

---

## Part 2: Frontend Setup (Docusaurus Textbook)

### 1. Navigate to textbook directory

```bash
cd ../textbook
```

### 2. Install dependencies

```bash
npm install
```

### 3. Configure API URL (optional)

For local development, the ChatWidget defaults to `http://localhost:8000`.

For production, set the environment variable:

```bash
# Create .env.local file
echo "REACT_APP_API_URL=https://your-backend-url.railway.app" > .env.local
```

### 4. Run the development server

```bash
npm start
```

The textbook will open at: http://localhost:3000

### 5. Test the ChatWidget

1. Look for the **💬 chat button** in the bottom-right corner
2. Click to open the chat widget
3. Ask a question: "What is ROS 2?"
4. Verify you get an answer with source citations

**Test selected text mode:**

1. Highlight any paragraph on the page
2. Click "Selected Text" mode in the chat widget
3. Ask "Explain this in simpler terms"
4. Verify the answer is based only on selected text

---

## Part 3: Run Tests

### Backend tests

```bash
cd rag-backend
pytest tests/ -v
```

### Frontend tests (if applicable)

```bash
cd textbook
npm test
```

---

## Part 4: Deployment

### Deploy Backend to Railway

1. Install Railway CLI:
   ```bash
   npm install -g @railway/cli
   ```

2. Login to Railway:
   ```bash
   railway login
   ```

3. Deploy from backend directory:
   ```bash
   cd rag-backend
   railway init
   railway up
   ```

4. Set environment variables in Railway dashboard:
   - `OPENAI_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `CORS_ORIGINS` (add your Vercel domain)
   - `ENVIRONMENT=production`

5. Get your deployment URL (e.g., `https://your-app.railway.app`)

### Deploy Frontend to Vercel

1. Install Vercel CLI (optional):
   ```bash
   npm install -g vercel
   ```

2. Deploy from root directory:
   ```bash
   cd ..  # Back to project root
   vercel
   ```

3. Set environment variable in Vercel dashboard:
   - `REACT_APP_API_URL`: Your Railway backend URL

4. Redeploy to apply env var:
   ```bash
   vercel --prod
   ```

### Update CORS in Backend

After deployment, update `CORS_ORIGINS` in Railway to include your Vercel domain:

```
https://your-textbook.vercel.app
```

---

## Part 5: Verification Checklist

- [ ] Backend server runs on `localhost:8000`
- [ ] API docs accessible at `/docs`
- [ ] Health check returns `status: "healthy"`
- [ ] Ingestion script completes without errors
- [ ] Collection stats show vectors > 0
- [ ] Chat API returns answers with citations
- [ ] Textbook runs on `localhost:3000`
- [ ] ChatWidget appears in bottom-right
- [ ] Full book mode works (answers from any chapter)
- [ ] Selected text mode works (answers from selection only)
- [ ] Citations link to correct textbook sections
- [ ] Backend deployed to Railway/Render
- [ ] Frontend deployed to Vercel
- [ ] Production chatbot works end-to-end

---

## Troubleshooting

### Backend won't start

- Check `.env` file exists and has valid credentials
- Verify Python 3.11+ installed: `python --version`
- Check port 8000 not in use: `lsof -i :8000`

### Ingestion fails

- Verify `textbook/docs` directory exists
- Check OpenAI API key has credits
- Verify Qdrant URL and API key are correct
- Check network connectivity to Qdrant Cloud

### ChatWidget not appearing

- Check browser console for errors
- Verify API URL is correct (check `.env.local`)
- Test CORS: Open browser dev tools → Network tab → Check chat request

### Chat returns errors

- Check backend logs: Look for error messages in terminal
- Verify Qdrant has vectors: `GET /api/v1/collections/stats`
- Check OpenAI API quota not exceeded
- Test API directly with curl (see above)

### CORS errors in production

- Verify `CORS_ORIGINS` in backend includes your Vercel domain
- Use full URL with `https://` prefix
- Redeploy backend after updating CORS

---

## Architecture Overview

```
┌─────────────────────────────────────────┐
│         Docusaurus Frontend             │
│    (React, TypeScript, ChatWidget)      │
│         http://localhost:3000           │
└──────────────┬──────────────────────────┘
               │ POST /api/v1/chat/query
               ▼
┌─────────────────────────────────────────┐
│        FastAPI Backend (Python)         │
│   AgentOrchestrator → BookSearchAgent   │
│         http://localhost:8000           │
└──────────────┬──────────────────────────┘
               │
      ┌────────┴────────┐
      ▼                 ▼
┌──────────┐      ┌──────────┐
│  OpenAI  │      │  Qdrant  │
│   API    │      │  Cloud   │
│ (GPT-4o) │      │ (Vectors)│
└──────────┘      └──────────┘
```

---

## Cost Estimates

### OpenAI API

- Embeddings: ~$0.002 per 1K tokens (text-embedding-3-small)
- Completions: ~$0.15 per 1M input tokens (gpt-4o-mini)
- **Estimated**: $10-30/month for 1000 queries

### Qdrant Cloud (Free Tier)

- 1 GB storage
- ~500K vectors (1536 dimensions)
- Unlimited queries
- **Cost**: Free

### Hosting

- Railway: Free tier (500 hours/month) or $5/month
- Vercel: Free tier for static sites
- **Cost**: $0-5/month

**Total estimated cost**: $10-35/month

---

## Next Steps

1. **Test with real users**: Share textbook link and collect feedback
2. **Monitor costs**: Check OpenAI usage dashboard daily
3. **Optimize chunking**: Adjust `CHUNK_SIZE` if answers are too narrow/broad
4. **Add analytics**: Track popular questions and click-through on citations
5. **Improve prompts**: Refine system prompts based on answer quality

---

## Support

- Backend issues: Check `rag-backend/README.md`
- Frontend issues: Check `textbook/README.md`
- Spec questions: See `specs/002-rag-chatbot/spec.md`

**Project Structure**:
- `/rag-backend` - FastAPI RAG API
- `/textbook` - Docusaurus static site
- `/specs` - Feature specifications
- `/history` - Prompt history records

For deployment help, see Railway/Vercel documentation.
