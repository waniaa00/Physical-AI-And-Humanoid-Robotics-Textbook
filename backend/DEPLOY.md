# Quick Deployment Guide

## Step-by-Step: Deploy Backend to Hugging Face Spaces

### 1. Create Your Space

1. Visit: https://huggingface.co/new-space
2. Settings:
   - Name: `humanoid-robotics-api`
   - SDK: **Docker**
   - Visibility: Public or Private
3. Click **Create Space**

### 2. Clone the Space Repository

```bash
# Replace <your-username> with your HF username
git clone https://huggingface.co/spaces/<your-username>/humanoid-robotics-api
cd humanoid-robotics-api
```

### 3. Copy Backend Files

```bash
# From your project root, copy backend files to the Space
cp backend/Dockerfile .
cp backend/requirements-hf.txt requirements.txt
cp backend/main.py .
cp backend/agent_config.py .
cp backend/tools.py .
cp backend/models.py .
cp -r backend/models ./
cp -r backend/services ./
cp -r backend/middleware ./
cp -r backend/utils ./
cp -r backend/repositories ./
cp -r backend/database ./
cp backend/README-HF.md README.md
```

### 4. Add Environment Variables

In your Space settings (⚙️ Settings → Repository secrets), add:

| Variable | Example Value |
|----------|---------------|
| `OPENAI_API_KEY` | `sk-proj-...` |
| `COHERE_KEY` | `...` |
| `QDRANT_URL` | `https://....cloud.qdrant.io:6333` |
| `QDRANT_KEY` | `...` |
| `NEON_URL` | `postgresql://...neon.tech/...` |
| `BETTERAUTH_SECRET` | Generate with: `python -c "import secrets; print(secrets.token_urlsafe(32))"` |
| `CORS_ORIGINS` | `https://your-frontend.vercel.app` |
| `QDRANT_COLLECTION_NAME` | `humanoid_robotics_book` |

### 5. Push to Deploy

```bash
git add .
git commit -m "Deploy Humanoid Robotics RAG API"
git push
```

### 6. Monitor Build

1. Go to your Space page
2. Click on **Logs** tab
3. Wait for build to complete (3-5 minutes)
4. Status should show **Running ✓**

### 7. Test Your API

```bash
# Health check
curl https://<your-username>-humanoid-robotics-api.hf.space/health

# Expected response:
# {"status":"healthy"}
```

### 8. Update Frontend

In your Vercel frontend environment variables:

```bash
vercel env add REACT_APP_API_URL
# Enter: https://<your-username>-humanoid-robotics-api.hf.space

# Or update docusaurus.config.ts:
customFields: {
  backendUrl: 'https://<your-username>-humanoid-robotics-api.hf.space',
}
```

Redeploy frontend:
```bash
cd website
vercel --prod
```

## Done! 🎉

Your backend is now running on Hugging Face Spaces!

**API URL**: `https://<your-username>-humanoid-robotics-api.hf.space`

**API Docs**: `https://<your-username>-humanoid-robotics-api.hf.space/docs`

## Troubleshooting

**Build fails?**
- Check Logs tab for errors
- Verify all environment variables are set
- Ensure Docker SDK is selected

**API not responding?**
- Check Space is in "Running" state
- Verify port 7860 is exposed
- Review application logs

**CORS errors?**
- Add frontend URL to `CORS_ORIGINS`
- Restart Space after env variable changes

**Need help?** See [README-HF.md](./README-HF.md) for detailed troubleshooting.
