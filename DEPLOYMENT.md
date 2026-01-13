# Deployment Guide for RAG Chatbot for Textbook

## Overview
This guide provides instructions for deploying the RAG (Retrieval-Augmented Generation) chatbot for textbook Q&A.

## Prerequisites

### External Services Required
1. **Google Gemini API** - For embeddings and chat completion
2. **Qdrant Cloud** - For vector storage and retrieval
3. **Neon Postgres** - For chat history persistence

### System Requirements
- Python 3.9+
- Node.js 18+ (for Docusaurus frontend)
- Docker (optional, for containerized deployment)

## Environment Configuration

### Copy Environment Template
```bash
cp .env.example .env
```

### Required Environment Variables

#### Gemini API Configuration
```
GEMINI_API_KEY=your-gemini-api-key-here
GEMINI_CHAT_MODEL=gemini-2.5-flash
GEMINI_EMBEDDING_MODEL=embedding-001
```

#### Qdrant Configuration
```
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_URL=your-qdrant-cloud-url
QDRANT_COLLECTION_NAME=book_chunks
```

#### Neon Postgres Configuration
```
NEON_DATABASE_URL=postgresql://username:password@ep-xxxxxx.us-east-1.aws.neon.tech/dbname
NEON_POOL_MIN=1
NEON_POOL_MAX=10
```

#### Application Configuration
```
FASTAPI_HOST=0.0.0.0
FASTAPI_PORT=8000
FASTAPI_DEBUG=false
CORS_ORIGINS=["https://yourdomain.com", "https://your-subdomain.yourdomain.com"]
CHATBOT_API_URL=https://yourdomain.com
CHATBOT_WIDGET_ENABLED=true
```

#### Security Configuration
```
JWT_SECRET_KEY=your-jwt-secret-key-here
JWT_EXPIRATION_HOURS=24
SESSION_SECRET=your-session-secret-here
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=3600
```

## Backend Deployment

### Installation
```bash
# Clone the repository
git clone <repository-url>
cd <repository-name>

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### Database Setup
```bash
# The application will automatically create required tables on first run
# Make sure your Neon Postgres connection is configured in .env
```

### Indexing Textbook Content
```bash
# Run the indexing process to populate the vector database
python -m src.indexing.pipeline "my-website/docs"
```

### Starting the Service
```bash
# Start the FastAPI server
python -m src.main
```

## Frontend Integration (Docusaurus)

### Installation
```bash
cd my-website
npm install
```

### Building and Serving
```bash
# Build the website
npm run build

# Serve the website
npm run serve
```

### Adding the Chat Widget
The chat widget is automatically integrated into the Docusaurus site via the components created in `my-website/src/components/rag-chatbot/`.

## Production Deployment

### Using Docker
```Dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .

EXPOSE 8000

CMD ["python", "-m", "src.main"]
```

### Docker Compose Example
```yaml
version: '3.8'

services:
  rag-chatbot-api:
    build: .
    ports:
      - "8000:8000"
    environment:
      - GEMINI_API_KEY=${GEMINI_API_KEY}
      - QDRANT_API_KEY=${QDRANT_API_KEY}
      - QDRANT_URL=${QDRANT_URL}
      - NEON_DATABASE_URL=${NEON_DATABASE_URL}
    depends_on:
      - postgres

  docusaurus:
    build:
      context: ./my-website
      dockerfile: Dockerfile
    ports:
      - "3000:3000"
    depends_on:
      - rag-chatbot-api
```

## Scaling Recommendations

### API Rate Limits
- Default rate limit: 100 requests per hour per IP
- Adjust `RATE_LIMIT_REQUESTS` and `RATE_LIMIT_WINDOW` as needed

### Vector Database
- Monitor Qdrant Cloud usage and scale as needed
- Consider using Qdrant's clustering features for high availability

### Database Connection Pooling
- Configure `NEON_POOL_MIN` and `NEON_POOL_MAX` based on expected load
- Default is 1-10 connections

## Monitoring and Maintenance

### Health Checks
- API health: `GET /health`
- Frontend health: `GET /` (should return service information)

### Logging
- Application logs are output in JSON format
- Configure log aggregation as needed for your infrastructure

### Backup Procedures
- Database backups should be managed through Neon Postgres dashboard
- Consider backing up vector database separately if needed

## Troubleshooting

### Common Issues

1. **API Keys Not Working**
   - Verify all API keys are correctly set in environment
   - Check for typos in the keys

2. **Database Connection Failures**
   - Verify Neon Postgres connection string is correct
   - Check that the database is accessible from the deployed environment

3. **Vector Database Issues**
   - Ensure Qdrant Cloud is accessible
   - Verify collection exists and has proper permissions

4. **Indexing Problems**
   - Check that `my-website/docs` directory exists and is readable
   - Verify Gemini API is accessible for embedding generation

### Performance Tuning
- Increase connection pool size for high traffic
- Adjust rate limiting based on usage patterns
- Monitor token usage for Gemini API

## Security Considerations

- Never commit API keys to version control
- Use environment variables for all sensitive configuration
- Implement proper CORS policies for your domain
- Regularly rotate API keys
- Monitor API usage for unusual patterns