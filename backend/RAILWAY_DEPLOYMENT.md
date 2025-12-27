# Railway Deployment Guide

Complete guide for deploying the Physical AI & Humanoid Robotics backend to Railway.

## Prerequisites

1. **Railway Account**: Sign up at [railway.app](https://railway.app)
2. **Qdrant Cloud Account**: Sign up at [cloud.qdrant.io](https://cloud.qdrant.io)
3. **Google Cloud Account**: For Gemini API key
4. **GitHub Repository**: Code pushed to GitHub

## Step 1: Set Up Qdrant Cloud

1. Go to [cloud.qdrant.io](https://cloud.qdrant.io)
2. Create a new cluster (Free tier available)
3. Note your:
   - **Cluster URL** (e.g., `https://xxx.qdrant.io`)
   - **API Key** (from cluster settings)
4. Create collection:
   ```bash
   # Use Qdrant dashboard or API to create collection
   # Collection name: textbook_chunks
   # Vector size: 768 (Gemini text-embedding-004)
   # Distance: Cosine
   ```

## Step 2: Get Gemini API Key

1. Go to [Google AI Studio](https://makersuite.google.com/app/apikey)
2. Create API key
3. Note your **API Key**

## Step 3: Create Railway Project

### Option A: Deploy from GitHub (Recommended)

1. **Login to Railway**: Visit [railway.app](https://railway.app)

2. **New Project**:
   - Click "New Project"
   - Select "Deploy from GitHub repo"
   - Authorize Railway to access your GitHub
   - Select repository: `HasnainCodeHub/Master-Robotics-AI`
   - Select branch: `main`
   - Railway will auto-detect the Dockerfile

3. **Add PostgreSQL Database**:
   - In your project dashboard, click "+ New"
   - Select "Database" → "Add PostgreSQL"
   - Railway will provision a database and set `DATABASE_URL` automatically

### Option B: Deploy with Railway CLI

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Initialize in backend directory
cd backend
railway init

# Link to project (create if doesn't exist)
railway link

# Add PostgreSQL
railway add postgresql

# Deploy
railway up
```

## Step 4: Configure Environment Variables

In Railway dashboard → Your Service → Variables tab, add:

### Required Variables

```bash
# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_COLLECTION=textbook_chunks

# Google Gemini API
GOOGLE_API_KEY=your_gemini_api_key_here

# JWT Authentication (CRITICAL - Generate secure key)
JWT_SECRET=<generate-with-command-below>

# Environment
ENV=production
LOG_LEVEL=INFO

# CORS (Your frontend URL)
CORS_ORIGINS=https://hasnaincodehub.github.io,https://*.github.io

# RAG Configuration (Optional - defaults are set)
EMBEDDING_MODEL=text-embedding-004
EMBEDDING_DIMENSION=768
RETRIEVAL_THRESHOLD=0.7
RETRIEVAL_LIMIT=5
```

### Generate JWT_SECRET

Run this command locally and copy the output:
```bash
python -c "import secrets; print(secrets.token_hex(32))"
```

Example output: `a8f5f167f44f4964e6c998dee827110c47f1d5f0e7a2e8b9c6d4a3b2c1d0e9f8`

### Auto-Set Variables

Railway automatically sets:
- `DATABASE_URL` - From PostgreSQL addon
- `PORT` - Railway assigns port dynamically

## Step 5: Run Database Migrations

After first deployment:

### Option A: Using Railway CLI

```bash
cd backend
railway run alembic upgrade head
```

### Option B: Using Railway Dashboard

1. Go to your service → Settings
2. Under "Deploy Triggers", add a "Custom Start Command" (one-time):
   ```
   alembic upgrade head && uvicorn app.main:app --host 0.0.0.0 --port $PORT
   ```
3. Deploy once, then remove the migration part

### Option C: Run Locally with Production DB

```bash
# Export Railway DATABASE_URL
export DATABASE_URL=$(railway variables get DATABASE_URL)

# Run migrations
alembic upgrade head
```

## Step 6: Seed Qdrant with Textbook Content

You need to populate Qdrant with textbook embeddings:

### Method 1: Local Script (Recommended)

```bash
cd backend

# Set environment variables
export QDRANT_URL=https://your-cluster.qdrant.io
export QDRANT_API_KEY=your_api_key
export GOOGLE_API_KEY=your_gemini_key

# Run seeding script
python scripts/ingest_content.py
```

### Method 2: Via Railway

```bash
railway run python scripts/ingest_content.py
```

**Note**: Seeding needs to happen ONCE before the chatbot can work.

## Step 7: Verify Deployment

### Check Health Endpoint

```bash
curl https://your-app.railway.app/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-27T...",
  "version": "1.0.0",
  "database": "connected",
  "qdrant": "connected"
}
```

### Check API Documentation

Visit: `https://your-app.railway.app/docs`

You should see the interactive Swagger UI with all endpoints.

### Test Authentication

```bash
# Signup
curl -X POST https://your-app.railway.app/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "Test1234",
    "profile": {
      "software_level": "intermediate",
      "robotics_level": "beginner",
      "hardware_access": "simulation_only"
    }
  }'

# Expected: Returns auth token
```

### Test RAG Chat

```bash
# Get token from signup response, then:
curl -X POST https://your-app.railway.app/api/chat \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_TOKEN_HERE" \
  -d '{
    "question": "What is Physical AI?"
  }'

# Expected: Returns grounded answer with sources
```

## Step 8: Connect Frontend

Update your frontend configuration to use the Railway backend URL:

### In Docusaurus Config

File: `docs/docusaurus.config.ts`

```typescript
customFields: {
  apiBaseUrl: process.env.API_BASE_URL || 'https://your-app.railway.app',
  isProduction: process.env.NODE_ENV === 'production',
},
```

### For GitHub Pages Deployment

Set in GitHub Actions workflow or build command:
```bash
API_BASE_URL=https://your-app.railway.app npm run build
```

## Troubleshooting

### Issue: Health check failing

**Symptoms**: Service keeps restarting, health check timeout

**Solutions**:
1. Check logs: Railway dashboard → Deployments → View Logs
2. Verify DATABASE_URL is set
3. Verify QDRANT_URL and QDRANT_API_KEY are correct
4. Check if migrations ran successfully
5. Increase healthcheckTimeout in railway.toml

### Issue: CORS errors from frontend

**Symptoms**: Browser console shows CORS policy errors

**Solutions**:
1. Add your frontend URL to `CORS_ORIGINS`:
   ```
   CORS_ORIGINS=https://hasnaincodehub.github.io,http://localhost:3000
   ```
2. Include protocol (https://) and no trailing slash
3. Redeploy backend after changing CORS_ORIGINS

### Issue: Database connection errors

**Symptoms**: SQLAlchemy connection errors in logs

**Solutions**:
1. Verify DATABASE_URL starts with `postgres://` or `postgresql://`
2. Railway's Postgres URL is auto-converted to `postgresql+asyncpg://`
3. Check PostgreSQL addon is running in Railway project
4. Run migrations: `railway run alembic upgrade head`

### Issue: Qdrant collection not found

**Symptoms**: "Collection textbook_chunks not found" error

**Solutions**:
1. Create collection in Qdrant Cloud dashboard
2. Or run initialization script:
   ```bash
   railway run python scripts/ingest_content.py
   ```
3. Verify QDRANT_COLLECTION=textbook_chunks in env vars

### Issue: Import errors or module not found

**Symptoms**: Python import errors in logs

**Solutions**:
1. Verify PYTHONPATH=/app is set in Dockerfile (✅ already set)
2. Check all dependencies are in requirements.txt
3. Rebuild: Railway dashboard → Deployments → Redeploy

### Issue: JWT authentication errors

**Symptoms**: "Could not validate credentials" errors

**Solutions**:
1. Verify JWT_SECRET is set and matches between deployments
2. Generate new secret: `python -c "import secrets; print(secrets.token_hex(32))"`
3. Ensure it's at least 32 characters long
4. Restart service after changing JWT_SECRET

## Production Checklist

Before going live:

- [ ] JWT_SECRET is set to a strong random value (32+ chars)
- [ ] ENV=production
- [ ] CORS_ORIGINS includes only your production frontend URL
- [ ] DATABASE_URL is using Railway's PostgreSQL (SSL enabled)
- [ ] QDRANT_URL and QDRANT_API_KEY are from Qdrant Cloud
- [ ] GOOGLE_API_KEY is valid and has quota
- [ ] Database migrations ran successfully (`alembic upgrade head`)
- [ ] Qdrant collection is seeded with textbook content
- [ ] Health check endpoint returns 200 OK
- [ ] Swagger docs are accessible at /docs
- [ ] Test signup, signin, chat, and search endpoints
- [ ] Frontend can communicate with backend (no CORS errors)
- [ ] Logs show no errors (check Railway dashboard)

## Monitoring & Maintenance

### View Logs

```bash
# Real-time logs via CLI
railway logs

# Or view in Railway dashboard → Deployments → Logs
```

### Database Backups

Railway PostgreSQL automatically creates backups. To export:

```bash
# Get DATABASE_URL
railway variables get DATABASE_URL

# Dump database
pg_dump "$(railway variables get DATABASE_URL)" > backup.sql
```

### Update Deployment

```bash
# Commit changes and push
git add .
git commit -m "Update backend"
git push origin main

# Railway auto-deploys on push to main branch
```

### Scale Resources

Railway dashboard → Service → Settings → Resources:
- Adjust CPU/RAM as needed
- Free tier: 512MB RAM, 0.5 vCPU
- Paid tier: Up to 32GB RAM, 8 vCPU

## Cost Estimation

### Free Tier Limits
- Railway: $5/month free credit
- Qdrant Cloud: 1GB free cluster
- Gemini API: Free tier (15 RPM, 1500 RPD)
- PostgreSQL: 1GB storage on Railway

### Expected Usage
- Small to medium traffic: Free tier sufficient
- 100-500 users: ~$10-20/month
- 1000+ users: Consider upgrading Railway plan

## Support

- Railway Docs: https://docs.railway.app
- Railway Discord: https://discord.gg/railway
- Qdrant Docs: https://qdrant.tech/documentation
- Project Issues: https://github.com/HasnainCodeHub/Master-Robotics-AI/issues
