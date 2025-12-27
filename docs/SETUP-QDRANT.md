# 🔷 Qdrant Cloud Setup Guide

Complete guide to setting up Qdrant Cloud for your RAG chatbot vector database.

---

## Step 1: Create Qdrant Cloud Account

### 1.1 Sign Up

1. Go to **https://cloud.qdrant.io**
2. Click **"Sign Up"** or **"Get Started"**
3. Choose sign-up method:
   - **GitHub** (recommended - fastest)
   - **Google**
   - **Email** (requires verification)
4. Complete authentication

### 1.2 Verify Email (if using email signup)

1. Check your inbox for verification email
2. Click verification link
3. You'll be redirected to Qdrant dashboard

---

## Step 2: Create Your First Cluster

### 2.1 Navigate to Clusters

1. After login, you'll see the main dashboard
2. Click **"Create Cluster"** or **"+ New Cluster"**

### 2.2 Configure Cluster Settings

**Cluster Name:**
```
textbook-rag
```
(or any name you prefer)

**Plan Selection:**
```
✅ Free Tier
   - 1 GB vector storage
   - Shared CPU
   - Perfect for this project (~15 MB usage expected)
```

**Region Selection:**
Choose the region **closest to your users**:
- **US East** (N. Virginia) - Good for US users
- **EU West** (Frankfurt) - Good for European users
- **Asia Pacific** (Singapore) - Good for Asian users

> 💡 **Tip**: Choose the same region where you'll deploy your Railway backend for lowest latency.

**Configuration Summary:**
- Cluster Type: **Free**
- Region: **[Your choice]**
- Expected usage: **~15-20 MB** (out of 1 GB limit)

### 2.3 Create Cluster

1. Click **"Create"**
2. Wait 30-60 seconds for cluster provisioning
3. Status will change from "Provisioning" → "Running"

---

## Step 3: Get Connection Credentials

### 3.1 Access Cluster Details

1. Click on your cluster name in the dashboard
2. You'll see the cluster details page

### 3.2 Copy Cluster URL

Look for **"Cluster URL"** or **"Endpoint"**:
```
Example: https://abc123-def456.eu-central.aws.cloud.qdrant.io:6333
```

**Copy this entire URL** - you'll need it for `QDRANT_URL` in `.env`

### 3.3 Generate API Key

1. Look for **"API Keys"** section
2. Click **"Create API Key"** or **"Generate Key"**
3. (Optional) Give it a name: `rag-chatbot-backend`
4. Click **"Create"**
5. **IMPORTANT**: Copy the API key immediately - it won't be shown again!

```
Example: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Save this API key** - you'll need it for `QDRANT_API_KEY` in `.env`

---

## Step 4: Configure Backend Environment

### 4.1 Update `.env` File

```bash
cd backend
nano .env  # or use your preferred editor
```

**Add these lines:**
```bash
# Qdrant Configuration
QDRANT_URL=https://abc123-def456.eu-central.aws.cloud.qdrant.io:6333
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
QDRANT_COLLECTION_NAME=textbook_chunks
```

Replace with **YOUR actual values**:
- `QDRANT_URL` → Your cluster URL (from Step 3.2)
- `QDRANT_API_KEY` → Your API key (from Step 3.3)

### 4.2 Verify Connection

**Test Script:**
```python
# backend/scripts/test_qdrant_connection.py
from qdrant_client import QdrantClient
from app.config import settings

try:
    client = QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY
    )

    # Try to list collections
    collections = client.get_collections()
    print(f"✅ Connected successfully!")
    print(f"📦 Collections: {len(collections.collections)}")

except Exception as e:
    print(f"❌ Connection failed: {e}")
    print("\nCheck:")
    print("1. QDRANT_URL is correct")
    print("2. QDRANT_API_KEY is correct")
    print("3. Cluster status is 'Running' in Qdrant dashboard")
```

**Run test:**
```bash
cd backend
source venv/bin/activate
python scripts/test_qdrant_connection.py
```

**Expected output:**
```
✅ Connected successfully!
📦 Collections: 0
```

---

## Step 5: Run First Ingestion

### 5.1 Ingest Documents

```bash
python scripts/ingest_docs.py \
    --docs-path ../textbook/docs \
    --batch-size 50 \
    --verbose
```

This will:
1. ✅ Create collection `textbook_chunks` automatically
2. ✅ Process ~30 markdown files
3. ✅ Create ~2000 chunks
4. ✅ Generate embeddings via OpenAI
5. ✅ Upload vectors to Qdrant

**Expected time:** 2-3 minutes

### 5.2 Verify in Qdrant Dashboard

1. Go back to Qdrant Cloud dashboard
2. Click on your cluster
3. Click **"Collections"** in the left sidebar
4. You should see:
   - Collection name: `textbook_chunks`
   - Vectors count: ~2000+
   - Status: Active

---

## Step 6: Monitor Usage

### 6.1 Check Storage Usage

1. In Qdrant dashboard, go to cluster overview
2. Look for **"Storage Used"**
3. You should see: **~15-20 MB / 1 GB**

**Usage breakdown:**
```
Vectors: 2000+
Vector size: 1536 dimensions × 4 bytes = 6.1 KB per vector
Total: ~2000 × 6.1 KB = ~12.2 MB
Metadata: ~2-3 MB
Total: ~15 MB (1.5% of 1 GB limit)
```

### 6.2 Set Up Alerts (Optional)

Qdrant Free Tier alerts:
- **80% usage** (800 MB) - Warning
- **95% usage** (950 MB) - Critical

You're using ~15 MB, so you have **plenty of headroom**.

---

## Troubleshooting

### Issue: "Connection refused" or "Unauthorized"

**Solution:**
```bash
# Check cluster status in dashboard
# Status should be "Running", not "Stopped" or "Provisioning"

# Verify credentials in .env
cat .env | grep QDRANT
```

### Issue: Collection not created

**Solution:**
```python
# Manually create collection
from qdrant_client import QdrantClient
from qdrant_client.models import VectorParams, Distance

client = QdrantClient(
    url="your-url",
    api_key="your-key"
)

client.create_collection(
    collection_name="textbook_chunks",
    vectors_config=VectorParams(
        size=1536,
        distance=Distance.COSINE
    )
)
```

### Issue: Slow ingestion

**Solution:**
```bash
# Increase batch size
python scripts/ingest_docs.py \
    --docs-path ../textbook/docs \
    --batch-size 100  # Larger batches
```

### Issue: "Rate limit exceeded"

**Solution:**
```python
# Add delay in ingest_docs.py
await asyncio.sleep(0.5)  # Increase from 0.1 to 0.5
```

---

## Security Best Practices

### 1. Protect API Key

```bash
# NEVER commit .env to git
echo '.env' >> .gitignore

# Use environment variables in production
export QDRANT_API_KEY='...'
```

### 2. Rotate API Keys Periodically

1. In Qdrant dashboard → API Keys
2. Create new key
3. Update `.env` with new key
4. Delete old key

### 3. Restrict CORS (when available)

In Qdrant settings, restrict access to:
- Your Railway backend domain
- Localhost (for development)

---

## Next Steps

✅ Qdrant cluster running
✅ Collection created with vectors
✅ Connection verified

**Now you can:**
1. Test RAG queries locally
2. Deploy backend to Railway (see SETUP-RAILWAY.md)
3. Connect frontend to backend

---

## Cost Monitoring

### Free Tier Limits

| Resource | Limit | Your Usage | % Used |
|----------|-------|------------|--------|
| Storage | 1 GB | ~15 MB | 1.5% |
| Requests | Unlimited* | - | - |
| Collections | Unlimited | 1 | - |

*Subject to fair use policy

### When to Upgrade?

You'll need paid tier if:
- Vector count exceeds ~160,000 (1 GB / 6 KB per vector)
- You need dedicated CPU
- You need SLA guarantees

**For this hackathon project**: Free tier is MORE than sufficient.

---

## Quick Reference

```bash
# Qdrant Dashboard
https://cloud.qdrant.io

# Check connection
python scripts/test_qdrant_connection.py

# Re-ingest documents
python scripts/ingest_docs.py --docs-path ../textbook/docs

# Update single file
python scripts/ingest_docs.py --single-file path/to/file.md

# Check collection stats
python -c "
from qdrant_client import QdrantClient
from app.config import settings
client = QdrantClient(url=settings.QDRANT_URL, api_key=settings.QDRANT_API_KEY)
info = client.get_collection('textbook_chunks')
print(f'Vectors: {info.vectors_count}')
"
```

---

**Status**: ✅ Qdrant Cloud configured
**Next**: Configure environment variables (see SETUP-ENV.md)
