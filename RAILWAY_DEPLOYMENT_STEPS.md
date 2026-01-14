# Railway Deployment Steps

Follow these steps to deploy your RAG Chatbot Backend to Railway:

## 1. Log in to Railway
```bash
railway login
```

## 2. Initialize Railway project (if not already done)
```bash
cd D:\Ai-Native-Hackathon\hackathon-1-book
railway init
```
Choose "From Git Repository" if you've already connected your GitHub repo, or "Empty Project" if starting fresh.

## 3. Link to your existing Railway project (if you created one via the web UI)
```bash
railway link
```
Select your project from the list.

## 4. Set environment variables
```bash
railway vars set GEMINI_API_KEY="AIzaSyD0o0tDAEeK2p8rmQHNRWuYWOCE2TawPhw"
railway vars set QDRANT_API_KEY="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.olEYwCiKE7GisUDolxHj5_UcYfd-E6k2exYmH2Gyuus"
railway vars set QDRANT_URL="https://d3f1d927-c80d-4047-8db9-4ebf400052c3.europe-west3-0.gcp.cloud.qdrant.io"
railway vars set NEON_DATABASE_URL="postgresql://neondb_owner:npg_UDK4NtZSFPw2@ep-dawn-wind-ah8t3fs1-pooler.c-3.us-east-1.aws.neon.tech/neondb?sslmode=require&channel_binding=require"
railway vars set JWT_SECRET_KEY="your-jwt-secret-key-here"
railway vars set SESSION_SECRET="your-session-secret-here"
railway vars set GEMINI_CHAT_MODEL="models/gemini-flash-latest"
railway vars set GEMINI_EMBEDDING_MODEL="models/text-embedding-004"
railway vars set QDRANT_VECTOR_SIZE="768"
railway vars set QDRANT_COLLECTION_NAME="book_chunks"
railway vars set QDRANT_VECTOR_NAME="bookvector"
railway vars set QDRANT_DISTANCE_METRIC="Cosine"
railway vars set ENVIRONMENT="production"
railway vars set LOG_LEVEL="INFO"
railway vars set CHUNK_SIZE="600"
railway vars set CHUNK_OVERLAP="100"
railway vars set TOP_K_RESULTS="8"
railway vars set RELEVANCE_THRESHOLD="0.7"
railway vars set MAX_CONTEXT_LENGTH="4000"
```

## 5. Deploy to Railway
```bash
railway up
```

## 6. Alternative: Deploy via the Railway dashboard
If you prefer using the dashboard:
1. Go to https://railway.app
2. Make sure your GitHub repo is connected
3. Your project should already be visible if you connected it
4. Go to the "Settings" tab of your Railway project
5. Add the environment variables listed above in the "Variables" tab
6. The project should automatically deploy when you push to GitHub, or you can manually trigger a deployment

## 7. Check deployment status
```bash
railway logs
```

## 8. Open your deployed application
```bash
railway open
```

## Troubleshooting

If you encounter the vector dimension error (which we saw in Docker):
- The issue is that your Qdrant collection was created with 512 dimensions but the embedding model generates 768 dimensions
- To fix this, you may need to either:
  1. Change your embedding model to one that generates 512-dimension vectors, or
  2. Recreate your Qdrant collection with 768 dimensions

For option 1, you could change the embedding model to one that matches the collection:
- Instead of `models/text-embedding-004` (768 dims), use a model that outputs 512 dimensions, or
- Change the QDRANT_VECTOR_SIZE to 512 and make sure to use an appropriate embedding model

The Railway deployment will use the same Dockerfile that we've been working with, so the lazy initialization fix is already included.