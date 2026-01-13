import hashlib
import uuid
from typing import List, Dict, Any, Optional
from src.models.document_chunk import DocumentChunk
from src.utils.token_counter import get_token_counter, truncate_text_to_token_limit
from src.utils.logging import get_logger

logger = get_logger(__name__)


class Chunker:
    """
    Implements deterministic document chunking with metadata tracking.
    Uses a strategy with overlapping windows to maintain context continuity.
    """

    def __init__(self,
                 chunk_size: int = 512,
                 overlap_size: int = 50,
                 max_chunk_size: int = 1024):
        """
        Initialize the chunker with specified parameters.

        Args:
            chunk_size: Target size for each chunk in tokens
            overlap_size: Number of tokens to overlap between chunks
            max_chunk_size: Maximum size for a chunk in tokens
        """
        self.chunk_size = chunk_size
        self.overlap_size = overlap_size
        self.max_chunk_size = max_chunk_size
        self.token_counter = get_token_counter()
        self.logger = get_logger(__name__)

    def chunk_document(self,
                      text: str,
                      source_metadata: Dict[str, Any],
                      chunk_id_prefix: Optional[str] = None) -> List[DocumentChunk]:
        """
        Chunk a document into smaller pieces with metadata tracking.

        Args:
            text: The text content to chunk
            source_metadata: Metadata about the source document
            chunk_id_prefix: Optional prefix for chunk IDs

        Returns:
            List of DocumentChunk objects
        """
        if not text.strip():
            return []

        # Split text into sentences to maintain semantic boundaries
        sentences = self._split_into_sentences(text)

        if not sentences:
            return []

        chunks = []
        current_chunk = ""
        current_tokens = 0
        chunk_number = 0

        i = 0
        while i < len(sentences):
            sentence = sentences[i]
            sentence_tokens = self.token_counter.count_tokens(sentence)

            # If adding this sentence would exceed the chunk size
            if current_tokens + sentence_tokens > self.chunk_size and current_chunk:
                # Create a chunk with the current content
                chunk_id = self._generate_chunk_id(
                    text=current_chunk,
                    source_metadata=source_metadata,
                    chunk_number=chunk_number,
                    prefix=chunk_id_prefix
                )

                chunk = DocumentChunk(
                    chunk_id=chunk_id,
                    text_content=current_chunk.strip(),
                    source_metadata=self._update_metadata_for_chunk(
                        source_metadata,
                        chunk_number,
                        len(chunks) + 1
                    )
                )

                chunks.append(chunk)
                chunk_number += 1

                # Start a new chunk with overlap
                if self.overlap_size > 0:
                    # Find overlapping content from the previous chunk
                    overlap_text = self._get_overlap_content(current_chunk, self.overlap_size)
                    current_chunk = overlap_text + " " + sentence
                    current_tokens = self.token_counter.count_tokens(current_chunk)
                else:
                    current_chunk = sentence
                    current_tokens = sentence_tokens
            else:
                # Add sentence to current chunk
                if current_chunk:
                    current_chunk += " " + sentence
                else:
                    current_chunk = sentence
                current_tokens += sentence_tokens

            i += 1

        # Add the final chunk if there's remaining content
        if current_chunk.strip():
            chunk_id = self._generate_chunk_id(
                text=current_chunk,
                source_metadata=source_metadata,
                chunk_number=chunk_number,
                prefix=chunk_id_prefix
            )

            chunk = DocumentChunk(
                chunk_id=chunk_id,
                text_content=current_chunk.strip(),
                source_metadata=self._update_metadata_for_chunk(
                    source_metadata,
                    chunk_number,
                    len(chunks) + 1
                )
            )

            chunks.append(chunk)

        self.logger.info(f"Chunked document into {len(chunks)} chunks")
        return chunks

    def _split_into_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences using common sentence delimiters.

        Args:
            text: Text to split into sentences

        Returns:
            List of sentences
        """
        import re

        # Split text on sentence boundaries
        # This regex looks for sentence endings followed by whitespace and capital letter or end of string
        sentence_pattern = r'(?<=[.!?])\s+(?=[A-Z])|(?<=[.!?])$'
        sentences = re.split(sentence_pattern, text)

        # Clean up sentences and remove empty strings
        cleaned_sentences = [s.strip() for s in sentences if s.strip()]

        return cleaned_sentences

    def _get_overlap_content(self, chunk_text: str, overlap_tokens: int) -> str:
        """
        Get the last 'overlap_tokens' tokens from a chunk to use as overlap.

        Args:
            chunk_text: Text of the chunk
            overlap_tokens: Number of tokens for overlap

        Returns:
            Overlap text
        """
        # Get the token count of the full chunk
        full_tokens = self.token_counter.count_tokens(chunk_text)

        if full_tokens <= overlap_tokens:
            return chunk_text

        # Find a good split point (try to split at sentence or word boundary)
        # Start from the end and work backwards
        words = chunk_text.split()
        overlap_text = ""

        for i in range(len(words) - 1, -1, -1):
            test_text = " ".join(words[i:])
            if self.token_counter.count_tokens(test_text) <= overlap_tokens:
                overlap_text = test_text
                break

        # If we couldn't find a good split, just truncate
        if not overlap_text:
            overlap_text = truncate_text_to_token_limit(chunk_text, overlap_tokens)

        return overlap_text

    def _generate_chunk_id(self,
                          text: str,
                          source_metadata: Dict[str, Any],
                          chunk_number: int,
                          prefix: Optional[str] = None) -> str:
        """
        Generate a deterministic chunk ID based on content and metadata.

        Args:
            text: The chunk text
            source_metadata: Source metadata
            chunk_number: The chunk number in the document
            prefix: Optional prefix for the chunk ID

        Returns:
            Generated chunk ID
        """
        # Create a string that includes content and metadata for uniqueness
        content_for_hash = f"{text[:100]}|{chunk_number}|{source_metadata.get('file_path', '')}"

        # Generate a hash of the content for determinism
        content_hash = hashlib.md5(content_for_hash.encode()).hexdigest()[:8]

        # Create chunk ID with prefix if provided
        if prefix:
            return f"{prefix}_{chunk_number:04d}_{content_hash}"
        else:
            return f"chunk_{chunk_number:04d}_{content_hash}"

    def _update_metadata_for_chunk(self,
                                   original_metadata: Dict[str, Any],
                                   chunk_number: int,
                                   total_chunks: int) -> Dict[str, Any]:
        """
        Update metadata for a specific chunk.

        Args:
            original_metadata: Original document metadata
            chunk_number: Current chunk number
            total_chunks: Total number of chunks in document

        Returns:
            Updated metadata for this chunk
        """
        updated_metadata = original_metadata.copy()
        updated_metadata['chunk_number'] = chunk_number
        updated_metadata['total_chunks'] = total_chunks
        updated_metadata['chunk_id'] = chunk_number  # For reference

        return updated_metadata

    def chunk_documents_batch(self,
                             documents: List[Dict[str, Any]],
                             source_metadata_base: Optional[Dict[str, Any]] = None) -> List[DocumentChunk]:
        """
        Chunk multiple documents in batch.

        Args:
            documents: List of documents, each with 'content' and 'metadata' keys
            source_metadata_base: Base metadata to include in each chunk

        Returns:
            List of DocumentChunk objects from all documents
        """
        all_chunks = []

        for i, doc in enumerate(documents):
            content = doc.get('content', '')
            metadata = doc.get('metadata', {})

            # Merge base metadata with document-specific metadata
            if source_metadata_base:
                merged_metadata = {**source_metadata_base, **metadata}
            else:
                merged_metadata = metadata

            # Generate a prefix for chunk IDs based on document
            chunk_id_prefix = f"doc_{i:04d}"

            chunks = self.chunk_document(
                text=content,
                source_metadata=merged_metadata,
                chunk_id_prefix=chunk_id_prefix
            )

            all_chunks.extend(chunks)

        self.logger.info(f"Chunked {len(documents)} documents into {len(all_chunks)} total chunks")
        return all_chunks


def chunk_text_content(text: str,
                      source_metadata: Dict[str, Any],
                      chunk_size: int = 512,
                      overlap_size: int = 50) -> List[DocumentChunk]:
    """
    Convenience function to chunk text content.

    Args:
        text: The text content to chunk
        source_metadata: Metadata about the source document
        chunk_size: Target size for each chunk in tokens
        overlap_size: Number of tokens to overlap between chunks

    Returns:
        List of DocumentChunk objects
    """
    chunker = Chunker(chunk_size=chunk_size, overlap_size=overlap_size)
    return chunker.chunk_document(text, source_metadata)


if __name__ == "__main__":
    # Example usage
    chunker = Chunker(chunk_size=100, overlap_size=20)

    sample_text = """
    This is the first sentence of our sample document. It contains some information that we want to chunk.
    Here is the second sentence which continues the thought. We're testing the chunking algorithm.
    This is the third sentence. It's important that we maintain context between chunks.
    The fourth sentence continues the example. We want to make sure sentences aren't split inappropriately.
    Finally, this is the last sentence of our sample text. It concludes the example content.
    """

    metadata = {
        'file_path': 'test/sample.md',
        'title': 'Sample Document',
        'author': 'Test Author',
        'module': 'Test Module',
        'chapter': 'Test Chapter'
    }

    chunks = chunker.chunk_document(sample_text, metadata)

    print(f"Created {len(chunks)} chunks:")
    for i, chunk in enumerate(chunks):
        print(f"\nChunk {i+1} (ID: {chunk.chunk_id}):")
        print(f"  Content preview: {chunk.text_content[:100]}...")
        print(f"  Tokens: {chunk.created_at}")  # This will show the datetime