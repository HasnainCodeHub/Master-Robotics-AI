# Railway Deployment - Quick Start Guide

## 5-Minute Railway Deployment

### Step 1: Create Railway Project (2 min)

1. Go to [railway.app](https://railway.app) and sign in
2. Click "New Project"
3. Select "Deploy from GitHub repo"
4. Choose: `HasnainCodeHub/Master-Robotics-AI`
5. Select root directory: `/backend`
6. Railway will auto-detect Dockerfile and start building

### Step 2: Add PostgreSQL (30 seconds)

1. In your Railway project, click "+ New"
2. Select "Database" → "Add PostgreSQL"
3. Railway automatically sets `DATABASE_URL`

### Step 3: Set Environment Variables (2 min)

In Railway dashboard → Your Service → Variables:

**Click "Raw Editor" and paste:**

```bash
# Qdrant Cloud (sign up at cloud.qdrant.io)
QDRANT_URL=https://your-cluster-url.qdrant.io
QDRANT_API_KEY=your_api_key_here
QDRANT_COLLECTION=textbook_chunks

# Google Gemini (get from makersuite.google.com/app/apikey)
GOOGLE_API_KEY=your_gemini_api_key_here

# JWT Secret (generate below)
JWT_SECRET=REPLACE_WITH_GENERATED_SECRET

# Environment
ENV=production
LOG_LEVEL=INFO

# CORS - Your frontend URL
CORS_ORIGINS=https://hasnaincodehub.github.io
```

**Generate JWT_SECRET:**
```bash
python -c "import secrets; print(secrets.token_hex(32))"
```
Copy the output and replace `REPLACE_WITH_GENERATED_SECRET`

### Step 4: Deploy & Verify (1 min)

1. Railway will auto-deploy after setting variables
2. Wait for build to complete (2-3 minutes)
3. Copy your Railway URL (e.g., `https://master-robotics-ai-production.up.railway.app`)
4. Test health: Visit `https://your-url.railway.app/api/health`

**Expected Response:**
```json
{
  "status": "healthy",
  "postgres_connected": true,
  "qdrant_connected": true,
  "timestamp": "2025-12-27T..."
}
```

## Post-Deployment Setup

### Run Database Migrations

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Link to project
cd backend
railway link

# Run migrations
railway run alembic upgrade head
```

### Seed Qdrant with Content

**Important**: You must seed Qdrant before the chatbot will work.

```bash
# Set environment variables locally
export QDRANT_URL=https://your-cluster.qdrant.io
export QDRANT_API_KEY=your_api_key
export GOOGLE_API_KEY=your_gemini_key

# Run seeding script
python scripts/ingest_content.py
```

**Or via Railway:**
```bash
railway run python scripts/ingest_content.py
```

## Update Frontend Configuration

Update your frontend to use the Railway backend URL:

**File**: `docs/docusaurus.config.ts`

```typescript
customFields: {
  apiBaseUrl: 'https://your-app.railway.app',
  isProduction: true,
},
```

## Verify Deployment Checklist

- [ ] Health endpoint returns `"status": "healthy"`
- [ ] API docs accessible at `/docs`
- [ ] Can create user via `/api/auth/signup`
- [ ] Can login via `/api/auth/signin`
- [ ] Can send chat message (requires Qdrant seeding)
- [ ] Frontend can connect (no CORS errors)

## Common Issues & Fixes

### ❌ `ModuleNotFoundError: No module named 'email_validator'`
**Fixed**: Added `email-validator>=2.1.0` to requirements.txt

### ❌ Health check failing
**Fix**: Verify DATABASE_URL and QDRANT_URL are set correctly

### ❌ CORS errors
**Fix**: Add frontend URL to CORS_ORIGINS:
```
CORS_ORIGINS=https://hasnaincodehub.github.io
```

### ❌ JWT errors
**Fix**: Ensure JWT_SECRET is set (32+ characters)

### ❌ Chatbot returns empty results
**Fix**: Run Qdrant seeding script (see above)

## Quick Links

- **Railway Dashboard**: https://railway.app/dashboard
- **Qdrant Cloud**: https://cloud.qdrant.io
- **Gemini API Keys**: https://makersuite.google.com/app/apikey
- **Deployment Logs**: Railway → Your Service → Deployments → View Logs
- **Full Guide**: See `RAILWAY_DEPLOYMENT.md`

## Cost Summary

- **Railway**: $5/month free credit (sufficient for testing)
- **Qdrant Cloud**: Free tier (1GB cluster)
- **Gemini API**: Free tier (15 RPM, 1500 RPD)
- **PostgreSQL**: Included in Railway free tier

**Total**: $0 for development/testing with free tiers

## Next Steps After Deployment

1. Test all API endpoints via Swagger (`/docs`)
2. Create a test account
3. Ask a question to verify RAG pipeline works
4. Deploy frontend to GitHub Pages with backend URL
5. Test end-to-end user flow
6. Monitor logs for any errors
7. Set up custom domain (optional)

Your backend is now ready for Railway deployment! 🚀
