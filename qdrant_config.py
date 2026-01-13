"""
Configuration for Qdrant vector database to match the physical-ai-book collection.
"""

# Qdrant Configuration - Updated to match your collection
QDRANT_CONFIG = {
    # Collection name as created in Qdrant dashboard
    "collection_name": "physical-ai-book",

    # Vector configuration matching your setup
    "vector_params": {
        "bookvector": {  # This matches your vector name
            "size": 512,  # Matches your 512 dimensions
            "distance": "Cosine"  # Matches your cosine distance
        }
    },

    # Tenant field name as specified
    "tenant_field": "book-id",

    # Additional configuration
    "shard_number": 1,
    "replication_factor": 1,
    "write_consistency_factor": 1,
    "on_disk_payload": False
}

# Mapping for document chunk storage
PAYLOAD_SCHEMA = {
    "text_content": "text",  # The actual text content
    "source_metadata": "json",  # JSON with file_path, module, chapter, etc.
    "chunk_id": "keyword",  # Unique chunk identifier
    "book_id": "keyword",  # Book identifier for tenant field
    "created_at": "float",  # Timestamp
    "metadata": "json"  # Additional metadata
}

# Configuration for the Qdrant client
CLIENT_CONFIG = {
    "prefer_grpc": False,  # Use HTTP for broader compatibility
    "timeout": 30,  # 30 second timeout
    "https": True,  # Use HTTPS (assumes cloud instance)
    "host": None,  # Will be loaded from environment
    "api_key": None,  # Will be loaded from environment
}

# Collection creation parameters
COLLECTION_PARAMS = {
    "vectors_config": {
        "bookvector": {
            "size": 512,
            "distance": "Cosine"
        }
    },
    "shard_number": 1,
    "replication_factor": 1
}

# Payload index configuration
PAYLOAD_INDEX_CONFIG = {
    "book-id": "keyword",  # Index the tenant field for fast filtering
    "chunk_id": "keyword",
    "module": "keyword",
    "chapter": "keyword",
    "file_path": "keyword"
}