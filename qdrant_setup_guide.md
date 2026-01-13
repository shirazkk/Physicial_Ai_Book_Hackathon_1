# Qdrant Configuration Guide for Physical-AI Book RAG System

This guide explains how to configure the RAG system to work with your Qdrant collection.

## Your Qdrant Collection Details

- **Collection Name**: `physical-ai-book`
- **Vector Name**: `bookvector`
- **Vector Dimensions**: 512
- **Distance Metric**: Cosine
- **Tenant Field**: `book-id`

## Required Environment Variables

Update your `.env` file with the following settings to match your Qdrant setup:

```env
# Qdrant Configuration - Updated for your collection
QDRANT_API_KEY=your-actual-api-key-here
QDRANT_URL=https://your-cluster-url.qdrant.tech  # Replace with your actual URL
QDRANT_COLLECTION_NAME=physical-ai-book  # Your collection name

# Vector configuration (matches your setup)
QDRANT_VECTOR_NAME=bookvector  # Your vector field name
QDRANT_VECTOR_SIZE=512  # Your dimension size
QDRANT_DISTANCE_METRIC=Cosine  # Your distance metric
```

## System Integration Notes

### 1. Vector Storage Configuration
The system has been updated to:
- Use `bookvector` as the named vector field (instead of default unnamed vector)
- Store embeddings with 512 dimensions (matching your setup)
- Use Cosine distance metric (matching your setup)
- Include `book-id` in the payload for tenant field filtering

### 2. Indexing Process
When you run the indexing:
```bash
python -m src.indexing.pipeline "my-website/docs"
```

The system will:
- Generate 512-dimensional embeddings using Gemini
- Store them in the `bookvector` named vector field
- Include document metadata with the `book-id` field
- Use your existing `physical-ai-book` collection

### 3. Search Process
When searching, the system will:
- Query the `bookvector` named vector field
- Return results from your `physical-ai-book` collection
- Apply any filters based on the `book-id` tenant field

## Verification Steps

1. **Check Environment Variables**:
   - Verify `QDRANT_COLLECTION_NAME` is set to `physical-ai-book`
   - Verify `QDRANT_API_KEY` and `QDRANT_URL` are correct

2. **Test Connection**:
   ```bash
   # You can test basic connectivity with a simple script
   python -c "
   from qdrant_client import QdrantClient
   client = QdrantClient(url='YOUR_URL', api_key='YOUR_API_KEY')
   collections = client.get_collections()
   print('Collections:', [c.name for c in collections.collections])
   "
   ```

3. **Run Indexing**:
   ```bash
   python -m src.indexing.pipeline "my-website/docs"
   ```

4. **Monitor in Qdrant Dashboard**:
   - Points should appear in your `physical-ai-book` collection
   - Vectors should be stored in the `bookvector` field
   - Payload should include `book-id` and other metadata

## Troubleshooting

### Common Issues:

1. **"Collection not found" Error**:
   - Make sure your collection name matches exactly: `physical-ai-book`
   - Verify the QDRANT_COLLECTION_NAME in your environment

2. **"Wrong vector dimensions" Error**:
   - Ensure you're using 512-dimensional embeddings
   - Check that the vector size configuration is correct

3. **Authentication Issues**:
   - Verify your API key is correct
   - Ensure your Qdrant URL is properly formatted

4. **Named Vector Issues**:
   - The system now uses `bookvector` as the named vector
   - This matches your Qdrant collection configuration

## Expected Behavior

Once properly configured:
- Documents will be chunked and converted to 512-dim embeddings
- Embeddings will be stored in the `bookvector` field in your collection
- Searches will query the `bookvector` field using cosine similarity
- Results will include document content and metadata including `book-id`

Your RAG system is now configured to work with your Qdrant collection `physical-ai-book`!