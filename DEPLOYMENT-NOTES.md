# Deployment Configuration Notes

## Frontend API URL Configuration

The ChatWidget needs to know where your backend API is deployed. There are two methods:

### Method 1: Edit static/config.js (Recommended)

Before deploying to production, edit `textbook/static/config.js`:

```javascript
window.RAG_API_URL = 'https://your-backend.railway.app';
```

Then build and deploy:
```bash
cd textbook
npm run build
```

### Method 2: Environment Variable (Vercel)

If deploying to Vercel, you can inject the URL at build time:

1. In Vercel dashboard, add environment variable:
   - Name: `RAG_API_URL`
   - Value: `https://your-backend.railway.app`

2. Update `textbook/static/config.js` to read from env:
```javascript
window.RAG_API_URL = process.env.RAG_API_URL || 'http://localhost:8000';
```

## CORS Configuration

**IMPORTANT**: After deploying frontend, update backend CORS:

In your Railway/Render backend deployment:
1. Set environment variable `CORS_ORIGINS`
2. Value: `https://your-textbook.vercel.app`
3. Redeploy backend

Without this, the ChatWidget will get CORS errors!

## Complete Deployment Checklist

### Backend (Railway/Render)
- [ ] Deploy backend to Railway/Render
- [ ] Set environment variables:
  - `OPENAI_API_KEY`
  - `QDRANT_URL`
  - `QDRANT_API_KEY`
  - `ENVIRONMENT=production`
  - `CORS_ORIGINS=https://your-frontend-url.vercel.app`
- [ ] Copy backend URL (e.g., `https://your-app.railway.app`)
- [ ] Test health endpoint: `curl https://your-app.railway.app/api/v1/health`
- [ ] Run ingestion (can be done locally or via API)

### Frontend (Vercel)
- [ ] Edit `textbook/static/config.js` with backend URL
- [ ] Deploy to Vercel: `vercel --prod`
- [ ] Copy frontend URL
- [ ] Update backend CORS with frontend URL
- [ ] Test ChatWidget on deployed site

### Verification
- [ ] Open deployed frontend
- [ ] Click 💬 button
- [ ] Ask: "What is ROS 2?"
- [ ] Verify answer appears with citations
- [ ] Click citation link → navigates to correct page
- [ ] Test selected text mode

## Cost Monitoring

- OpenAI: https://platform.openai.com/usage
- Qdrant: https://cloud.qdrant.io/ (check storage)
- Railway: Dashboard shows usage
- Vercel: Free tier (100GB bandwidth)

Set up alerts:
- OpenAI: Set usage limit ($50/month recommended)
- Railway: Monitor usage to stay in free tier

## Troubleshooting

### ChatWidget shows "Failed to get response"
1. Check browser console for CORS errors
2. Verify backend URL in config.js
3. Verify backend CORS includes frontend domain
4. Test backend directly: `curl https://your-backend/api/v1/health`

### Backend shows "Qdrant not configured"
1. Check Qdrant URL and API key
2. Verify Qdrant cluster is running
3. Check backend logs for connection errors

### No vectors in Qdrant
1. Run ingestion script (can run locally pointing to production Qdrant)
2. Or trigger via API: `POST /api/v1/ingest/documents`
3. Check collection stats: `GET /api/v1/collections/stats`

## Production Best Practices

1. **Security**:
   - Never commit .env files
   - Use environment variables for all secrets
   - Enable HTTPS only (Railway/Vercel do this automatically)

2. **Monitoring**:
   - Check backend logs daily
   - Monitor OpenAI costs weekly
   - Set up alerts for errors

3. **Performance**:
   - Monitor response times (should be <3s)
   - Check Qdrant query performance
   - Consider caching common queries (future enhancement)

4. **Updates**:
   - When textbook content changes, re-run ingestion
   - Keep dependencies updated (security patches)
   - Test locally before deploying to production
