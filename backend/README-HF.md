# Hugging Face Spaces Deployment Guide

This guide explains how to deploy the Humanoid Robotics RAG Chatbot backend to Hugging Face Spaces.

## Prerequisites

1. **Hugging Face Account**: Create one at https://huggingface.co/join
2. **External Services**:
   - OpenAI API key (for GPT-4o agent)
   - Cohere API key (for embeddings)
   - Qdrant Cloud instance (for vector storage)
   - Neon PostgreSQL database (for user data)

## Deployment Steps

### 1. Create a New Space

1. Go to https://huggingface.co/new-space
2. Choose:
   - **Space name**: `humanoid-robotics-api` (or your preferred name)
   - **License**: MIT
   - **Space SDK**: Docker
   - **Visibility**: Public or Private (your choice)
3. Click **Create Space**

### 2. Configure Environment Variables

In your Space settings, add these **Secrets** (Environment Variables):

```bash
# Required API Keys
OPENAI_API_KEY=sk-proj-your-openai-key-here
COHERE_KEY=your-cohere-api-key-here
QDRANT_URL=https://your-cluster.cloud.qdrant.io:6333
QDRANT_KEY=your-qdrant-api-key-here

# Database
NEON_URL=postgresql://user:password@ep-xxx.region.aws.neon.tech/db

# Authentication (generate a secure random string)
BETTERAUTH_SECRET=your-secure-random-secret-min-32-chars

# CORS Configuration (add your frontend URLs)
CORS_ORIGINS=https://website-wania-akrams-projects.vercel.app,http://localhost:3000

# Collection name in Qdrant
QDRANT_COLLECTION_NAME=humanoid_robotics_book
```

**To generate BETTERAUTH_SECRET**:
```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

### 3. Upload Files to Your Space

You can either:

**Option A: Use Git (Recommended)**

```bash
# Clone your Hugging Face Space repository
git clone https://huggingface.co/spaces/<your-username>/humanoid-robotics-api
cd humanoid-robotics-api

# Copy backend files
cp -r ../backend/* .

# Commit and push
git add .
git commit -m "Initial backend deployment"
git push
```

**Option B: Use Web Interface**

1. In your Space, click **Files and versions**
2. Click **Add file** → **Upload files**
3. Upload all backend files (main.py, agent_config.py, tools.py, requirements-hf.txt, Dockerfile, etc.)
4. Make sure to upload the entire `models/`, `services/`, `middleware/`, `utils/` directories

### 4. Required Files

Ensure these files are in your Space root:

```
humanoid-robotics-api/
├── Dockerfile                  # Docker configuration
├── requirements-hf.txt         # Python dependencies
├── main.py                     # FastAPI application
├── agent_config.py             # Agent configuration
├── tools.py                    # RAG tools
├── models/                     # Data models
├── services/                   # Business logic
├── middleware/                 # Auth middleware
├── utils/                      # Utilities
├── repositories/               # Database repositories
├── database/                   # Database setup
└── README.md                   # Documentation
```

### 5. Verify Deployment

1. Wait for the Space to build (this takes 3-5 minutes)
2. Check the **Logs** tab for any errors
3. Once running, your API will be available at:
   ```
   https://huggingface.co/spaces/<your-username>/humanoid-robotics-api
   ```

### 6. Test the API

```bash
# Health check
curl https://<your-username>-humanoid-robotics-api.hf.space/health

# Test chat endpoint (requires authentication)
curl -X POST https://<your-username>-humanoid-robotics-api.hf.space/agent/chat \
  -H "Content-Type: application/json" \
  -d '{
    "message": "What is inverse kinematics?",
    "session_id": "test-session",
    "mode": "full-book"
  }'
```

### 7. Update Frontend

Update your Vercel frontend to use the new HF Spaces backend URL:

In `website/docusaurus.config.ts`:

```typescript
customFields: {
  backendUrl: 'https://<your-username>-humanoid-robotics-api.hf.space',
},
```

## Troubleshooting

### Build Fails

1. **Check logs**: Go to Logs tab in your Space
2. **Common issues**:
   - Missing environment variables
   - Typo in requirements-hf.txt
   - Port mismatch (must use 7860 for HF Spaces)

### API Returns 500 Errors

1. Check if all environment variables are set correctly
2. Verify Qdrant collection exists
3. Check OpenAI API key is valid
4. Review logs for specific error messages

### CORS Errors from Frontend

1. Add your frontend URL to `CORS_ORIGINS` environment variable
2. Make sure to restart the Space after changing env vars

### Database Connection Issues

1. Verify Neon database URL is correct
2. Check that the database is accessible from external IPs
3. Ensure PostgreSQL connection string format is correct

## Production Considerations

1. **API Rate Limits**: Monitor OpenAI and Cohere usage
2. **Scaling**: HF Spaces has resource limits; consider upgrading if needed
3. **Monitoring**: Use HF Space logs for debugging
4. **Security**: Keep API keys in Secrets, never commit them
5. **Backups**: Regularly backup your Neon database

## Cost Considerations

- **Hugging Face Spaces**: Free tier available, paid for more resources
- **OpenAI API**: Pay per token (~$0.01/1K tokens for GPT-4o)
- **Cohere**: Free tier available, then pay per use
- **Qdrant Cloud**: Free tier with 1GB storage
- **Neon**: Free tier with 512MB storage

## Alternative: Deploy to Railway

If Hugging Face Spaces doesn't work for you, consider Railway:

```bash
# Install Railway CLI
npm install -g railway

# Login
railway login

# Initialize project
railway init

# Add environment variables
railway variables set OPENAI_API_KEY=sk-...

# Deploy
railway up
```

## Support

For issues:
1. Check Space logs
2. Review environment variable configuration
3. Verify all external services are accessible
4. Open an issue in the GitHub repository

## Resources

- [Hugging Face Spaces Documentation](https://huggingface.co/docs/hub/spaces)
- [Docker Deployment Guide](https://huggingface.co/docs/hub/spaces-sdks-docker)
- [OpenAI Agents Documentation](https://platform.openai.com/docs/guides/agents)
