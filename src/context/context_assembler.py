from typing import List, Dict, Any, Optional, Tuple
from src.utils.token_counter import get_token_counter, ContextWindowManager
from src.utils.logging import get_logger
from src.models.chat_message import ChatMessage

logger = get_logger(__name__)


class ContextAssembler:
    """
    Assembles context from retrieved chunks and conversation history within token limits.
    """

    def __init__(self,
                 max_context_tokens: int = 3072,  # Suitable for Gemini models
                 max_response_tokens: int = 1024):
        """
        Initialize the context assembler.

        Args:
            max_context_tokens: Maximum tokens for the context window
            max_response_tokens: Estimated tokens for the model's response
        """
        self.max_context_tokens = max_context_tokens
        self.max_response_tokens = max_response_tokens
        self.available_context_tokens = max_context_tokens - max_response_tokens
        self.token_counter = get_token_counter()
        self.context_manager = ContextWindowManager(max_context_tokens=self.available_context_tokens)
        self.logger = get_logger(__name__)

    def assemble_context(self,
                        query: str,
                        retrieved_chunks: List[Dict[str, Any]],
                        conversation_history: Optional[List[ChatMessage]] = None,
                        system_prompt: Optional[str] = None) -> str:
        """
        Assemble a context window from query, retrieved chunks, and conversation history.

        Args:
            query: User's current query
            retrieved_chunks: List of retrieved document chunks
            conversation_history: Previous conversation history
            system_prompt: System prompt to include in context

        Returns:
            Assembled context string within token limits
        """
        try:
            # Convert conversation history to the format expected by ContextWindowManager
            history_for_context = []
            if conversation_history:
                for msg in conversation_history:
                    history_for_context.append({
                        'role': msg.role,
                        'content': msg.content
                    })

            # Extract text from retrieved chunks
            chunk_texts = [chunk['text_content'] for chunk in retrieved_chunks]

            # Use the ContextWindowManager to build the context
            context = self.context_manager.build_context_window(
                user_query=query,
                retrieved_chunks=chunk_texts,
                system_prompt=system_prompt,
                conversation_history=history_for_context
            )

            self.logger.debug(f"Assembled context with {self.token_counter.count_tokens(context)} tokens")

            return context

        except Exception as e:
            self.logger.error(f"Error assembling context: {str(e)}")
            raise

    def assemble_context_for_selected_text_only(self,
                                              query: str,
                                              selected_text: str,
                                              conversation_history: Optional[List[ChatMessage]] = None,
                                              system_prompt: Optional[str] = None) -> str:
        """
        Assemble context using only the user-selected text (for "Selected Text Only" mode).

        Args:
            query: User's current query
            selected_text: Text selected by the user
            conversation_history: Previous conversation history
            system_prompt: System prompt to include in context

        Returns:
            Assembled context string using only selected text
        """
        try:
            # Convert conversation history to the format expected by ContextWindowManager
            history_for_context = []
            if conversation_history:
                for msg in conversation_history:
                    history_for_context.append({
                        'role': msg.role,
                        'content': msg.content
                    })

            # For selected text mode, we only use the selected text as context
            context = self.context_manager.build_context_window(
                user_query=query,
                retrieved_chunks=[selected_text],  # Only the selected text
                system_prompt=system_prompt,
                conversation_history=history_for_context
            )

            self.logger.debug(f"Assembled selected-text context with {self.token_counter.count_tokens(context)} tokens")

            return context

        except Exception as e:
            self.logger.error(f"Error assembling selected-text context: {str(e)}")
            raise

    def validate_context_fits_limits(self, context: str) -> Tuple[bool, int]:
        """
        Validate that the context fits within token limits.

        Args:
            context: Context string to validate

        Returns:
            Tuple of (is_valid, token_count)
        """
        token_count = self.token_counter.count_tokens(context)
        is_valid = token_count <= self.available_context_tokens

        if not is_valid:
            self.logger.warning(f"Context exceeds token limit: {token_count}/{self.available_context_tokens}")

        return is_valid, token_count

    def get_remaining_tokens(self, current_context: str) -> int:
        """
        Get the number of remaining tokens available in the context window.

        Args:
            current_context: Current context string

        Returns:
            Number of remaining tokens
        """
        current_tokens = self.token_counter.count_tokens(current_context)
        remaining = self.available_context_tokens - current_tokens
        return max(0, remaining)

    def truncate_chunks_to_fit(self,
                              chunks: List[Dict[str, Any]],
                              max_tokens: int) -> List[Dict[str, Any]]:
        """
        Truncate a list of chunks to fit within a token limit.

        Args:
            chunks: List of chunks to truncate
            max_tokens: Maximum tokens allowed

        Returns:
            Truncated list of chunks that fit within the limit
        """
        if not chunks:
            return []

        truncated_chunks = []
        current_tokens = 0

        for chunk in chunks:
            chunk_tokens = self.token_counter.count_tokens(chunk['text_content'])

            if current_tokens + chunk_tokens <= max_tokens:
                truncated_chunks.append(chunk)
                current_tokens += chunk_tokens
            else:
                # If this is the first chunk and it's too big, truncate it
                if len(truncated_chunks) == 0:
                    max_chunk_tokens = max_tokens
                    truncated_content = self.token_counter.truncate_text(
                        chunk['text_content'],
                        max_chunk_tokens
                    )

                    truncated_chunk = {
                        'chunk_id': chunk['chunk_id'],
                        'text_content': truncated_content,
                        'metadata': chunk['metadata'],
                        'score': chunk['score']
                    }

                    truncated_chunks.append(truncated_chunk)
                    current_tokens = max_chunk_tokens
                    break
                else:
                    # We can't add more chunks without exceeding the limit
                    break

        self.logger.debug(f"Truncated chunks from {len(chunks)} to {len(truncated_chunks)} to fit within {max_tokens} tokens")

        return truncated_chunks

    def format_chunks_for_context(self,
                                 chunks: List[Dict[str, Any]],
                                 max_total_tokens: Optional[int] = None) -> str:
        """
        Format chunks into a string suitable for context inclusion.

        Args:
            chunks: List of chunks to format
            max_total_tokens: Maximum total tokens for all chunks combined

        Returns:
            Formatted string containing the chunks
        """
        if not chunks:
            return ""

        # If max_total_tokens is specified, truncate chunks to fit
        if max_total_tokens is not None:
            chunks = self.truncate_chunks_to_fit(chunks, max_total_tokens)

        # Format chunks with clear separation
        formatted_chunks = []
        for i, chunk in enumerate(chunks):
            chunk_header = f"[Document Chunk {i+1}]"
            chunk_content = chunk['text_content']
            formatted_chunk = f"{chunk_header}\n{chunk_content}\n"

            formatted_chunks.append(formatted_chunk)

        formatted_context = "\n".join(formatted_chunks)
        return formatted_context

    def get_context_statistics(self, context: str) -> Dict[str, Any]:
        """
        Get statistics about a context string.

        Args:
            context: Context string to analyze

        Returns:
            Dictionary with context statistics
        """
        token_count = self.token_counter.count_tokens(context)
        char_count = len(context)
        word_count = len(context.split())

        return {
            'token_count': token_count,
            'character_count': char_count,
            'word_count': word_count,
            'is_within_limit': token_count <= self.available_context_tokens,
            'utilization_percent': (token_count / self.available_context_tokens) * 100
        }


def get_context_assembler() -> ContextAssembler:
    """
    Get a singleton instance of the context assembler.

    Returns:
        ContextAssembler instance
    """
    return ContextAssembler()


if __name__ == "__main__":
    # Example usage
    assembler = get_context_assembler()

    # Example query and chunks
    query = "What is the main concept discussed in the introduction?"

    chunks = [
        {
            'chunk_id': 'chunk_001',
            'text_content': 'The introduction discusses fundamental concepts of artificial intelligence including machine learning algorithms and neural networks.',
            'metadata': {'source': 'intro.md', 'section': 'concepts'},
            'score': 0.85
        },
        {
            'chunk_id': 'chunk_002',
            'text_content': 'Machine learning is a subset of AI that enables systems to learn and improve from experience without being explicitly programmed.',
            'metadata': {'source': 'ml_basics.md', 'section': 'definition'},
            'score': 0.78
        }
    ]

    # Example conversation history
    from src.models.chat_message import ChatMessage
    history = [
        ChatMessage(
            message_id="msg_001",
            role="user",
            content="What is artificial intelligence?"
        ),
        ChatMessage(
            message_id="msg_002",
            role="assistant",
            content="Artificial intelligence refers to systems that can perform tasks that typically require human intelligence."
        )
    ]

    system_prompt = "Answer questions based only on the provided context. If the answer is not in the context, say 'I don't know based on the provided information.'"

    try:
        context = assembler.assemble_context(
            query=query,
            retrieved_chunks=chunks,
            conversation_history=history,
            system_prompt=system_prompt
        )

        print(f"Context assembled with {assembler.token_counter.count_tokens(context)} tokens")
        print(f"Context preview: {context[:200]}...")

        stats = assembler.get_context_statistics(context)
        print(f"Context statistics: {stats}")

    except Exception as e:
        print(f"Error: {e}")