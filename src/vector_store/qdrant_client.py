from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from src.models.document_chunk import DocumentChunk
from src.utils.config import get_settings
from src.utils.exceptions import VectorStorageError
from src.utils.logging import get_logger

logger = get_logger(__name__)


class QdrantVectorStore:
    """
    Client for interacting with Qdrant vector database for storing and retrieving document embeddings.
    """

    def __init__(self,
                 host: Optional[str] = None,
                 api_key: Optional[str] = None,
                 collection_name: Optional[str] = None,
                 vector_size: int = 512):  # Changed to 512 to match your Qdrant configuration
        """
        Initialize the Qdrant client.

        Args:
            host: Qdrant host URL. If None, will be loaded from config.
            api_key: Qdrant API key. If None, will be loaded from config.
            collection_name: Name of the collection to use. If None, will be loaded from config.
            vector_size: Size of the embedding vectors.
        """
        settings = get_settings()

        self.host = host or settings.qdrant_url
        self.api_key = api_key or settings.qdrant_api_key
        self.collection_name = collection_name or settings.qdrant_collection_name
        self.vector_size = vector_size

        # Initialize Qdrant client
        self.client = QdrantClient(
            url=self.host,
            api_key=self.api_key,
            prefer_grpc=False  # Use HTTP for broader compatibility
        )

        self.logger = get_logger(__name__)
        self._ensure_collection_exists()

    def _is_valid_uuid(self, val):
        """Check if a string is a valid UUID."""
        import uuid
        try:
            uuid.UUID(val)
            return True
        except ValueError:
            return False

    def _ensure_collection_exists(self):
        """
        Ensure the collection exists in Qdrant, create it if it doesn't.
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name not in collection_names:
                # Create the collection with appropriate vector parameters
                # Using the configuration that matches your Qdrant setup
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config={
                        "bookvector": VectorParams(
                            size=self.vector_size,  # Using configured vector size
                            distance=Distance.COSINE  # Cosine distance as per your setup
                        )
                    }
                )
                self.logger.info(f"Created new Qdrant collection: {self.collection_name} with bookvector configuration (size: {self.vector_size})")
            else:
                self.logger.info(f"Using existing Qdrant collection: {self.collection_name}")

        except Exception as e:
            self.logger.error(f"Error ensuring collection exists: {str(e)}")
            raise VectorStorageError(f"Failed to ensure collection exists: {str(e)}")

    def store_embedding(self,
                       chunk_id: str,
                       embedding: List[float],
                       text_content: str,
                       metadata: Optional[Dict[str, Any]] = None) -> bool:
        """
        Store a single embedding in Qdrant.

        Args:
            chunk_id: Unique identifier for the chunk
            embedding: The embedding vector
            text_content: Original text content of the chunk
            metadata: Additional metadata to store with the embedding

        Returns:
            True if successful, False otherwise
        """
        try:
            # Prepare the point to be stored with your specific vector configuration
            # Qdrant expects IDs to be either integers or UUIDs, so we'll convert the chunk_id if needed
            point_id = chunk_id
            # Qdrant requires IDs to be either integers or valid UUIDs, so we'll convert if needed
            if not self._is_valid_uuid(chunk_id) and not chunk_id.isdigit():
                # Convert to integer ID based on hash for compatibility
                import hashlib
                # Create a deterministic integer ID from the chunk_id
                hash_object = hashlib.md5(chunk_id.encode())
                # Convert to a positive integer
                point_id = int(hash_object.hexdigest()[:8], 16) % (2**31)  # Limit to 32-bit positive integer

            points = [
                models.PointStruct(
                    id=point_id,
                    vector={"bookvector": embedding},  # Using your specific vector name
                    payload={
                        'text_content': text_content,
                        'metadata': metadata or {},
                        'chunk_id': chunk_id,  # Keep original chunk_id in payload
                        'book-id': metadata.get('book-id', 'default-book') if metadata else 'default-book'  # Using your tenant field
                    }
                )
            ]

            # Upload the point to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            self.logger.debug(f"Stored embedding for chunk: {chunk_id}")
            return True

        except Exception as e:
            self.logger.error(f"Error storing embedding for chunk {chunk_id}: {str(e)}")
            raise VectorStorageError(f"Failed to store embedding: {str(e)}")

    def store_embeddings_batch(self,
                              chunks: List[DocumentChunk]) -> bool:
        """
        Store multiple embeddings in Qdrant.

        Args:
            chunks: List of DocumentChunk objects to store

        Returns:
            True if successful, False otherwise
        """
        try:
            points = []
            for chunk in chunks:
                # Qdrant expects IDs to be either integers or UUIDs, so we'll convert the chunk_id if needed
                point_id = chunk.chunk_id
                # Qdrant requires IDs to be either integers or valid UUIDs, so we'll convert if needed
                if not self._is_valid_uuid(chunk.chunk_id) and not chunk.chunk_id.isdigit():
                    # Convert to integer ID based on hash for compatibility
                    import hashlib
                    # Create a deterministic integer ID from the chunk_id
                    hash_object = hashlib.md5(chunk.chunk_id.encode())
                    # Convert to a positive integer
                    point_id = int(hash_object.hexdigest()[:8], 16) % (2**31)  # Limit to 32-bit positive integer

                point = models.PointStruct(
                    id=point_id,
                    vector={"bookvector": chunk.embedding},  # Using your specific vector name
                    payload={
                        'text_content': chunk.text_content,
                        'metadata': chunk.source_metadata,
                        'chunk_id': chunk.chunk_id,  # Keep original chunk_id in payload
                        'book-id': chunk.source_metadata.get('book-id', 'default-book') if chunk.source_metadata else 'default-book'  # Using your tenant field
                    }
                )
                points.append(point)

            # Upload all points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            self.logger.info(f"Stored {len(chunks)} embeddings in batch")
            return True

        except Exception as e:
            self.logger.error(f"Error storing embeddings batch: {str(e)}")
            raise VectorStorageError(f"Failed to store embeddings batch: {str(e)}")

    def search_similar(self,
                      query_embedding: List[float],
                      limit: int = 10,
                      metadata_filter: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings in Qdrant.

        Args:
            query_embedding: The embedding vector to search for similar items
            limit: Maximum number of results to return
            metadata_filter: Optional filter for metadata fields

        Returns:
            List of similar chunks with their content and metadata
        """
        try:
            # Prepare the search filter if provided
            qdrant_filter = None
            if metadata_filter:
                conditions = []
                for key, value in metadata_filter.items():
                    conditions.append(
                        models.FieldCondition(
                            key=f"metadata.{key}",
                            match=models.MatchValue(value=value)
                        )
                    )
                if conditions:
                    qdrant_filter = models.Filter(must=conditions)

            # Perform the search using your specific vector name
            search_kwargs = {
                "collection_name": self.collection_name,
                "query_vector": ("bookvector", query_embedding),  # Specify the named vector
                "limit": limit,
                "with_payload": True,
                "with_vectors": False
            }

            # Add filter if it exists (different versions of Qdrant client may use different parameter names)
            if qdrant_filter is not None:
                search_kwargs["query_filter"] = qdrant_filter  # Some versions use query_filter

            search_results = self.client.search(**search_kwargs)

            # Process results
            results = []
            for hit in search_results:
                result = {
                    'chunk_id': hit.id,
                    'text_content': hit.payload.get('text_content', ''),
                    'metadata': hit.payload.get('metadata', {}),
                    'score': hit.score  # Similarity score
                }
                results.append(result)

            self.logger.debug(f"Search returned {len(results)} results")
            return results

        except Exception as e:
            self.logger.error(f"Error searching for similar embeddings: {str(e)}")
            raise VectorStorageError(f"Failed to search for similar embeddings: {str(e)}")

    def retrieve_by_id(self, chunk_id: str) -> Optional[Dict[str, Any]]:
        """
        Retrieve a specific chunk by its ID.

        Args:
            chunk_id: ID of the chunk to retrieve

        Returns:
            Chunk data if found, None otherwise
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id],
                with_payload=True,
                with_vectors=False
            )

            if records:
                record = records[0]
                return {
                    'chunk_id': record.id,
                    'text_content': record.payload.get('text_content', ''),
                    'metadata': record.payload.get('metadata', {}),
                }

            return None

        except Exception as e:
            self.logger.error(f"Error retrieving chunk {chunk_id}: {str(e)}")
            raise VectorStorageError(f"Failed to retrieve chunk: {str(e)}")

    def delete_by_id(self, chunk_id: str) -> bool:
        """
        Delete a specific chunk by its ID.

        Args:
            chunk_id: ID of the chunk to delete

        Returns:
            True if successful, False otherwise
        """
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[chunk_id]
                )
            )

            self.logger.debug(f"Deleted chunk: {chunk_id}")
            return True

        except Exception as e:
            self.logger.error(f"Error deleting chunk {chunk_id}: {str(e)}")
            raise VectorStorageError(f"Failed to delete chunk: {str(e)}")

    def delete_collection(self) -> bool:
        """
        Delete the entire collection.

        Returns:
            True if successful, False otherwise
        """
        try:
            self.client.delete_collection(self.collection_name)
            self.logger.info(f"Deleted collection: {self.collection_name}")
            return True

        except Exception as e:
            self.logger.error(f"Error deleting collection {self.collection_name}: {str(e)}")
            raise VectorStorageError(f"Failed to delete collection: {str(e)}")

    def get_collection_info(self) -> Dict[str, Any]:
        """
        Get information about the collection.

        Returns:
            Dictionary with collection information
        """
        try:
            info = self.client.get_collection(self.collection_name)
            return {
                'name': info.config.params.vectors.size,
                'vector_size': info.config.params.vectors.size,
                'distance': info.config.params.vectors.distance,
                'point_count': info.points_count
            }

        except Exception as e:
            self.logger.error(f"Error getting collection info: {str(e)}")
            raise VectorStorageError(f"Failed to get collection info: {str(e)}")


def get_qdrant_client() -> QdrantVectorStore:
    """
    Get a singleton instance of the Qdrant client.

    Returns:
        QdrantVectorStore instance
    """
    settings = get_settings()
    return QdrantVectorStore(
        host=settings.qdrant_url,
        api_key=settings.qdrant_api_key,
        collection_name=settings.qdrant_collection_name,
        vector_size=settings.qdrant_vector_size
    )


if __name__ == "__main__":
    # Example usage
    import os

    # You would need to set your Qdrant credentials in the environment
    # os.environ["QDRANT_URL"] = "your-qdrant-url"
    # os.environ["QDRANT_API_KEY"] = "your-qdrant-api-key"

    try:
        client = get_qdrant_client()

        # Example: Store a sample embedding
        sample_embedding = [0.1, 0.2, 0.3] * 256  # Example embedding (768 dimensions)
        success = client.store_embedding(
            chunk_id="test_chunk_001",
            embedding=sample_embedding,
            text_content="This is a test chunk for the RAG system.",
            metadata={"module": "test", "chapter": "intro"}
        )

        print(f"Storage successful: {success}")

        # Example: Search for similar content
        results = client.search_similar(
            query_embedding=sample_embedding,
            limit=5
        )

        print(f"Search results: {len(results)} found")
        for result in results:
            print(f"ID: {result['chunk_id']}, Score: {result['score']}")

    except Exception as e:
        print(f"Error: {e}")
        print("Make sure your QDRANT_URL and QDRANT_API_KEY are set in the environment")