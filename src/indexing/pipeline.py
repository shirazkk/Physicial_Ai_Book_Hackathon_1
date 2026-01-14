from typing import List, Optional, Dict
from src.models.document_chunk import DocumentChunk
from src.embedding.gemini_client import GeminiEmbeddingClient, get_gemini_embedding_client
from src.vector_store.qdrant_client import QdrantVectorStore, get_qdrant_client
from src.utils.exceptions import RAGChatException
from src.utils.logging import get_logger
from src.ingestion.document_scanner import get_all_markdown_files
from src.ingestion.content_extractor import ContentExtractor
from src.ingestion.chunker import Chunker

logger = get_logger(__name__)


class IndexingPipeline:
    """
    Full indexing pipeline to convert document chunks to embeddings and store in Qdrant.
    """

    def __init__(self,
                 embedding_client: Optional[GeminiEmbeddingClient] = None,
                 vector_store: Optional[QdrantVectorStore] = None):
        """
        Initialize the indexing pipeline.

        Args:
            embedding_client: Gemini embedding client. If None, will create default.
            vector_store: Qdrant vector store client. If None, will create default.
        """
        self.embedding_client = embedding_client or get_gemini_embedding_client()
        self.vector_store = vector_store or get_qdrant_client()
        self.content_extractor = ContentExtractor()
        self.chunker = Chunker()
        self.logger = get_logger(__name__)

    def index_document(self, file_path: str) -> bool:
        """
        Index a single document by converting it to chunks, generating embeddings, and storing in Qdrant.

        Args:
            file_path: Path to the document to index

        Returns:
            True if successful, False otherwise
        """
        try:
            self.logger.info(f"Starting indexing for document: {file_path}")

            # Extract content with metadata
            content_data = self.content_extractor.extract_content_with_metadata(file_path)

            # Create source metadata for chunks
            source_metadata = {
                'file_path': content_data['file_path'],
                'file_name': content_data['file_name'],
                'module': content_data['metadata'].get('module', ''),
                'chapter': content_data['metadata'].get('chapter', ''),
                'original_word_count': content_data['structural_info']['word_count'],
                'original_char_count': content_data['structural_info']['character_count'],
                'headings': content_data['structural_info']['headings']
            }

            # Chunk the document
            chunks = self.chunker.chunk_document(
                text=content_data['content'],
                source_metadata=source_metadata
            )

            self.logger.info(f"Chunked document into {len(chunks)} chunks")

            # Generate embeddings for all chunks
            chunk_texts = [chunk.text_content for chunk in chunks]
            embeddings = self.embedding_client.generate_embeddings_for_chunks(chunk_texts)

            # Update chunks with embeddings
            for chunk, embedding in zip(chunks, embeddings):
                chunk.embedding = embedding

            # Store embeddings in Qdrant
            success = self.vector_store.store_embeddings_batch(chunks)

            if success:
                self.logger.info(f"Successfully indexed document: {file_path} ({len(chunks)} chunks)")
            else:
                self.logger.error(f"Failed to store embeddings for document: {file_path}")
                return False

            return True

        except Exception as e:
            self.logger.error(f"Error indexing document {file_path}: {str(e)}")
            raise RAGChatException(f"Failed to index document {file_path}: {str(e)}")

    def index_documents_batch(self, file_paths: List[str]) -> Dict[str, bool]:
        """
        Index multiple documents in batch.

        Args:
            file_paths: List of file paths to index

        Returns:
            Dictionary mapping file paths to success status
        """
        results = {}

        for i, file_path in enumerate(file_paths):
            try:
                self.logger.info(f"Processing document {i+1}/{len(file_paths)}: {file_path}")

                success = self.index_document(file_path)
                results[file_path] = success

                if success:
                    self.logger.debug(f"Successfully indexed: {file_path}")
                else:
                    self.logger.warning(f"Failed to index: {file_path}")

            except Exception as e:
                self.logger.error(f"Error processing {file_path}: {str(e)}")
                results[file_path] = False

        # Log summary
        successful = sum(1 for success in results.values() if success)
        total = len(results)
        self.logger.info(f"Batch indexing complete: {successful}/{total} documents indexed successfully")

        return results

    def index_from_docs_directory(self, docs_directory: str = "my-website/docs") -> Dict[str, bool]:
        """
        Index all Markdown files from a specified directory.

        Args:
            docs_directory: Directory to scan for Markdown files

        Returns:
            Dictionary mapping file paths to success status
        """
        self.logger.info(f"Scanning directory for Markdown files: {docs_directory}")

        # Get all markdown files
        markdown_files = get_all_markdown_files(docs_directory)

        if not markdown_files:
            self.logger.warning(f"No Markdown files found in {docs_directory}")
            return {}

        self.logger.info(f"Found {len(markdown_files)} Markdown files to index")

        # Index all files
        return self.index_documents_batch(markdown_files)

    def update_index(self, file_path: str) -> bool:
        """
        Update the index for a specific document (delete old chunks, add new ones).

        Args:
            file_path: Path to the document to update

        Returns:
            True if successful, False otherwise
        """
        try:
            # In a real implementation, we'd need a way to identify all chunks from this document
            # For now, we'll just re-index the document
            return self.index_document(file_path)
        except Exception as e:
            self.logger.error(f"Error updating index for {file_path}: {str(e)}")
            raise RAGChatException(f"Failed to update index for {file_path}: {str(e)}")

    def refresh_index(self, docs_directory: str = "my-website/docs") -> Dict[str, bool]:
        """
        Refresh the entire index by clearing and re-indexing all documents.

        Args:
            docs_directory: Directory to scan for Markdown files

        Returns:
            Dictionary mapping file paths to success status
        """
        self.logger.info("Refreshing entire index...")

        try:
            # Get all markdown files
            markdown_files = get_all_markdown_files(docs_directory)

            if not markdown_files:
                self.logger.warning(f"No Markdown files found in {docs_directory}")
                return {}

            # For a complete refresh, we might want to clear the collection first
            # Note: This is destructive, so in production you might want a safer approach
            # self.vector_store.delete_collection()  # Uncomment if you want to clear everything
            # self.vector_store._ensure_collection_exists()  # Recreate collection

            # Index all files
            results = self.index_documents_batch(markdown_files)

            successful = sum(1 for success in results.values() if success)
            total = len(results)
            self.logger.info(f"Index refresh complete: {successful}/{total} documents refreshed")

            return results

        except Exception as e:
            self.logger.error(f"Error refreshing index: {str(e)}")
            raise RAGChatException(f"Failed to refresh index: {str(e)}")


def get_indexing_pipeline() -> IndexingPipeline:
    """
    Get a singleton instance of the indexing pipeline.

    Returns:
        IndexingPipeline instance
    """
    return IndexingPipeline()


def run_full_indexing(docs_directory: str = "my-website/docs"):
    """
    Run a full indexing process on the specified directory.

    Args:
        docs_directory: Directory containing Markdown files to index
    """
    pipeline = get_indexing_pipeline()

    try:
        results = pipeline.index_from_docs_directory(docs_directory)

        successful = sum(1 for success in results.values() if success)
        total = len(results)

        print(f"Indexing complete: {successful}/{total} documents processed")

        failed_docs = [path for path, success in results.items() if not success]
        if failed_docs:
            print(f"Failed to index {len(failed_docs)} documents:")
            for doc in failed_docs:
                print(f"  - {doc}")
        else:
            print("All documents indexed successfully!")

    except Exception as e:
        print(f"Error during indexing: {str(e)}")


if __name__ == "__main__":
    import sys

    docs_dir = sys.argv[1] if len(sys.argv) > 1 else "my-website/docs"
    print(f"Starting indexing process for: {docs_dir}")

    run_full_indexing(docs_dir)