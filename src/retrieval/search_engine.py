from typing import List, Dict, Any, Optional
from src.vector_store.qdrant_client import QdrantVectorStore, get_qdrant_client
from src.embedding.gemini_client import GeminiEmbeddingClient, get_gemini_embedding_client
from src.utils.exceptions import RetrievalError
from src.utils.logging import get_logger
from src.utils.token_counter import get_token_counter

logger = get_logger(__name__)


class RetrievalEngine:
    """
    Engine for retrieving relevant document chunks based on user queries.
    """

    def __init__(self,
                 vector_store: Optional[QdrantVectorStore] = None,
                 embedding_client: Optional[GeminiEmbeddingClient] = None):
        """
        Initialize the retrieval engine.

        Args:
            vector_store: Qdrant vector store client. If None, will create default.
            embedding_client: Gemini embedding client. If None, will create default.
        """
        self.vector_store = vector_store or get_qdrant_client()
        self.embedding_client = embedding_client or get_gemini_embedding_client()
        self.token_counter = get_token_counter()
        self.logger = get_logger(__name__)

    def retrieve_relevant_chunks(self,
                                query: str,
                                top_k: int = 5,
                                min_score: float = 0.5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant document chunks for a given query.

        Args:
            query: User's query/question
            top_k: Number of top results to return
            min_score: Minimum similarity score threshold

        Returns:
            List of relevant chunks with their content and metadata
        """
        try:
            if not query.strip():
                raise RetrievalError("Query cannot be empty")

            self.logger.debug(f"Retrieving chunks for query: '{query[:50]}...'")

            # Generate embedding for the query
            query_embedding = self.embedding_client.generate_embedding(query)

            # Search in Qdrant for similar chunks
            search_results = self.vector_store.search_similar(
                query_embedding=query_embedding,
                limit=top_k * 2  # Get more results to allow filtering
            )

            # Filter results based on minimum score
            filtered_results = [
                result for result in search_results
                if result['score'] >= min_score
            ]

            # Sort by score in descending order and limit to top_k
            sorted_results = sorted(filtered_results, key=lambda x: x['score'], reverse=True)[:top_k]

            self.logger.debug(f"Retrieved {len(sorted_results)} relevant chunks for query")

            return sorted_results

        except Exception as e:
            self.logger.error(f"Error retrieving chunks for query '{query[:30]}...': {str(e)}")
            raise RetrievalError(f"Failed to retrieve chunks: {str(e)}")

    def retrieve_with_selected_text_only(self,
                                       selected_text: str,
                                       top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve using only the user-provided selected text (for "Selected Text Only" mode).

        Args:
            selected_text: Text selected by the user
            top_k: Number of results to return (will only return the selected text as a chunk)

        Returns:
            List containing the selected text as a single chunk
        """
        if not selected_text.strip():
            raise RetrievalError("Selected text cannot be empty in Selected Text Only mode")

        # In "Selected Text Only" mode, we return the selected text as a single chunk
        # without querying the vector database
        result = {
            'chunk_id': 'selected_text_only',
            'text_content': selected_text,
            'metadata': {'source': 'user_selected'},
            'score': 1.0  # Highest possible score since it's exactly what the user provided
        }

        self.logger.debug("Returning selected text as context in Selected Text Only mode")

        return [result]

    def retrieve_and_rank(self,
                         query: str,
                         top_k: int = 5,
                         min_score: float = 0.3,
                         max_tokens: int = 2048) -> List[Dict[str, Any]]:
        """
        Retrieve relevant chunks and rank them, ensuring the total context fits within token limits.

        Args:
            query: User's query/question
            top_k: Number of top results to initially retrieve
            min_score: Minimum similarity score threshold
            max_tokens: Maximum total tokens for context

        Returns:
            List of relevant chunks that fit within token limits
        """
        try:
            # First, retrieve the most relevant chunks
            all_chunks = self.retrieve_relevant_chunks(query, top_k, min_score)

            # Calculate token counts for each chunk
            for chunk in all_chunks:
                chunk['token_count'] = self.token_counter.count_tokens(chunk['text_content'])

            # Sort by score and add chunks until we reach the token limit
            selected_chunks = []
            total_tokens = 0

            for chunk in all_chunks:
                chunk_tokens = chunk['token_count']

                # Check if adding this chunk would exceed the limit
                if total_tokens + chunk_tokens <= max_tokens:
                    selected_chunks.append(chunk)
                    total_tokens += chunk_tokens
                else:
                    # If this is the first chunk and it's too big, truncate it
                    if len(selected_chunks) == 0:
                        max_chunk_tokens = max_tokens
                        truncated_content = self.token_counter.truncate_text(
                            chunk['text_content'],
                            max_chunk_tokens
                        )

                        truncated_chunk = {
                            'chunk_id': chunk['chunk_id'],
                            'text_content': truncated_content,
                            'metadata': chunk['metadata'],
                            'score': chunk['score'],
                            'token_count': max_chunk_tokens
                        }

                        selected_chunks.append(truncated_chunk)
                        total_tokens = max_chunk_tokens
                        break
                    else:
                        # We can't add this chunk without exceeding the limit
                        break

            self.logger.debug(f"Selected {len(selected_chunks)} chunks within {max_tokens} token limit")

            return selected_chunks

        except Exception as e:
            self.logger.error(f"Error during ranked retrieval: {str(e)}")
            raise RetrievalError(f"Failed during ranked retrieval: {str(e)}")

    def find_chunks_by_metadata(self,
                               metadata_filters: Dict[str, Any],
                               limit: int = 10) -> List[Dict[str, Any]]:
        """
        Find chunks based on specific metadata filters.

        Args:
            metadata_filters: Dictionary of metadata fields and values to match
            limit: Maximum number of results to return

        Returns:
            List of matching chunks
        """
        try:
            # This would require more sophisticated filtering in Qdrant
            # For now, we'll search for all chunks and filter them in memory
            # In a production system, this would be done with Qdrant's filter capabilities

            # Generate a neutral query embedding to get all documents
            # This is a workaround; in practice, you'd use Qdrant's scroll functionality
            all_results = []  # Placeholder - would need proper implementation

            # For now, return empty list as this is an advanced feature
            self.logger.warning("Metadata-based search not fully implemented in this version")
            return []

        except Exception as e:
            self.logger.error(f"Error during metadata-based retrieval: {str(e)}")
            raise RetrievalError(f"Failed during metadata-based retrieval: {str(e)}")


def get_retrieval_engine() -> RetrievalEngine:
    """
    Get a singleton instance of the retrieval engine.

    Returns:
        RetrievalEngine instance
    """
    return RetrievalEngine()


if __name__ == "__main__":
    # Example usage
    engine = get_retrieval_engine()

    try:
        # Example query
        query = "What is the main concept discussed in the introduction?"

        results = engine.retrieve_relevant_chunks(query, top_k=3, min_score=0.1)

        print(f"Query: {query}")
        print(f"Found {len(results)} relevant chunks:")

        for i, result in enumerate(results):
            print(f"\n{i+1}. Score: {result['score']:.3f}")
            print(f"   Content: {result['text_content'][:100]}...")
            print(f"   Metadata: {result['metadata']}")

    except Exception as e:
        print(f"Error: {e}")