from typing import List, Optional
import google.generativeai as genai
from src.utils.config import get_settings
from src.utils.exceptions import EmbeddingGenerationError
from src.utils.logging import get_logger

logger = get_logger(__name__)


class GeminiEmbeddingClient:
    """
    Client for generating embeddings using the Google Gemini API.
    """

    def __init__(self, api_key: Optional[str] = None, model_name: Optional[str] = None):
        """
        Initialize the Gemini embedding client.

        Args:
            api_key: Gemini API key. If None, will be loaded from config.
            model_name: Name of the embedding model to use. If None, will be loaded from config.
        """
        settings = get_settings()

        self.api_key = api_key or settings.gemini_api_key
        self.model_name = model_name or settings.gemini_embedding_model

        # Configure the API key
        genai.configure(api_key=self.api_key)

        # Initialize the embedding model
        self.embedding_model = self.model_name
        self.logger = get_logger(__name__)

    def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text.

        Args:
            text: Text to generate embedding for

        Returns:
            List of floats representing the embedding vector
        """
        try:
            if not text.strip():
                raise EmbeddingGenerationError("Cannot generate embedding for empty text")

            # Generate the embedding using the Gemini API
            result = genai.embed_content(
                model=self.embedding_model,
                content=text,
                task_type="retrieval_document"  # Optimal for document retrieval
            )

            embedding = result['embedding']
            self.logger.debug(f"Generated embedding with {len(embedding)} dimensions for text of length {len(text)}")
            return embedding

        except Exception as e:
            self.logger.error(f"Error generating embedding for text: {str(e)}")
            raise EmbeddingGenerationError(f"Failed to generate embedding: {str(e)}")

    def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts.

        Args:
            texts: List of texts to generate embeddings for

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        embeddings = []

        for i, text in enumerate(texts):
            try:
                embedding = self.generate_embedding(text)
                embeddings.append(embedding)

                self.logger.debug(f"Generated embedding {i+1}/{len(texts)}: {len(embedding)} dimensions")
            except Exception as e:
                self.logger.error(f"Failed to generate embedding for text {i+1}: {str(e)}")
                # Raise the exception to indicate failure for the entire batch
                raise EmbeddingGenerationError(f"Failed to generate embedding at index {i}: {str(e)}")

        return embeddings

    def generate_embeddings_for_chunks(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for document chunks, optimized for the RAG use case.

        Args:
            texts: List of chunk texts to generate embeddings for

        Returns:
            List of embedding vectors
        """
        try:
            # For document retrieval, we use the "retrieval_document" task type
            # This is optimized for document chunks in a retrieval system
            embeddings = []
            for text in texts:
                result = genai.embed_content(
                    model=self.embedding_model,
                    content=text,
                    task_type="retrieval_document"
                )
                embedding = result['embedding']
                embeddings.append(embedding)

            self.logger.info(f"Generated {len(embeddings)} embeddings for {len(texts)} chunks")
            return embeddings

        except Exception as e:
            self.logger.error(f"Error generating embeddings for chunks: {str(e)}")
            raise EmbeddingGenerationError(f"Failed to generate embeddings for chunks: {str(e)}")


def get_gemini_embedding_client() -> GeminiEmbeddingClient:
    """
    Get a singleton instance of the Gemini embedding client.

    Returns:
        GeminiEmbeddingClient instance
    """
    settings = get_settings()
    return GeminiEmbeddingClient(
        api_key=settings.gemini_api_key,
        model_name=settings.gemini_embedding_model
    )


# Convenience function for generating a single embedding
def generate_single_embedding(text: str) -> List[float]:
    """
    Generate embedding for a single text using the default client.

    Args:
        text: Text to generate embedding for

    Returns:
        Embedding vector as a list of floats
    """
    client = get_gemini_embedding_client()
    return client.generate_embedding(text)


# Convenience function for generating embeddings in batch
def generate_embeddings_batch(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for a batch of texts using the default client.

    Args:
        texts: List of texts to generate embeddings for

    Returns:
        List of embedding vectors
    """
    client = get_gemini_embedding_client()
    return client.generate_embeddings_batch(texts)


if __name__ == "__main__":
    # Example usage
    import os

    # You would need to set your API key in the environment
    # os.environ["GEMINI_API_KEY"] = "your-api-key-here"

    try:
        client = get_gemini_embedding_client()

        sample_texts = [
            "This is the first document chunk.",
            "This is the second document chunk with different content.",
            "Here's a third chunk for testing purposes."
        ]

        print("Generating embeddings for sample texts...")
        embeddings = client.generate_embeddings_batch(sample_texts)

        print(f"Generated {len(embeddings)} embeddings")
        for i, embedding in enumerate(embeddings):
            print(f"Text {i+1}: {sample_texts[i][:30]}...")
            print(f"Embedding dimensions: {len(embedding)}")
            print(f"First 5 values: {embedding[:5]}")
            print()

    except Exception as e:
        print(f"Error: {e}")
        print("Make sure your GEMINI_API_KEY is set in the environment")