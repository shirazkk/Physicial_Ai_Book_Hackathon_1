import math
from typing import List, Optional
from src.utils.config import get_settings


class TokenCounter:
    """
    Utility class for counting tokens in text content.
    Uses a simple approximation method since tiktoken is not available.
    """

    def __init__(self, model_name: Optional[str] = None):
        """
        Initialize the TokenCounter.

        Args:
            model_name: Name of the model to use for tokenization.
                      If None, will use a default approximation.
        """
        self.model_name = model_name

    def count_tokens(self, text: str) -> int:
        """
        Count the number of tokens in a text string using a simple approximation.
        On average, 1 token is approximately 4 characters or 0.75 words.

        Args:
            text: The text to count tokens for

        Returns:
            Number of tokens in the text (approximated)
        """
        if not text:
            return 0

        # Simple approximation: count words and divide by 0.75 (average words per token)
        # Or count characters and divide by 4 (average characters per token)
        # Using the character method as it's more consistent
        char_count = len(text)
        approx_tokens = math.ceil(char_count / 4)

        return approx_tokens

    def count_tokens_batch(self, texts: List[str]) -> List[int]:
        """
        Count tokens for a batch of texts.

        Args:
            texts: List of texts to count tokens for

        Returns:
            List of token counts for each text
        """
        return [self.count_tokens(text) for text in texts]

    def truncate_text(self, text: str, max_tokens: int) -> str:
        """
        Truncate text to fit within the specified token limit using approximation.

        Args:
            text: The text to truncate
            max_tokens: Maximum number of tokens allowed

        Returns:
            Truncated text that fits within the token limit
        """
        if not text or max_tokens <= 0:
            return ""

        # Using character-based truncation (4 chars per token approximation)
        max_chars = max_tokens * 4
        if len(text) <= max_chars:
            return text

        truncated_text = text[:max_chars]

        # Clean up any trailing incomplete word
        last_space = truncated_text.rfind(' ')
        if last_space > max_chars * 0.9:  # Only if it's reasonably close to the end
            truncated_text = truncated_text[:last_space]

        return truncated_text


# Global token counter instance
_token_counter: Optional[TokenCounter] = None


def get_token_counter() -> TokenCounter:
    """
    Get the global token counter instance.

    Returns:
        TokenCounter instance
    """
    global _token_counter
    if _token_counter is None:
        _token_counter = TokenCounter()
    return _token_counter


def count_tokens(text: str) -> int:
    """
    Count tokens in a text string using the global token counter.

    Args:
        text: The text to count tokens for

    Returns:
        Number of tokens in the text
    """
    return get_token_counter().count_tokens(text)


def truncate_text_to_token_limit(text: str, max_tokens: int) -> str:
    """
    Truncate text to fit within the specified token limit using the global token counter.

    Args:
        text: The text to truncate
        max_tokens: Maximum number of tokens allowed

    Returns:
        Truncated text that fits within the token limit
    """
    return get_token_counter().truncate_text(text, max_tokens)


class ContextWindowManager:
    """
    Manages context windows for LLM interactions, ensuring content fits within token limits.
    """

    def __init__(self, max_context_tokens: int = 3072):  # Default for Gemini models
        """
        Initialize the ContextWindowManager.

        Args:
            max_context_tokens: Maximum number of tokens allowed in context window
        """
        self.max_context_tokens = max_context_tokens
        self.token_counter = get_token_counter()

    def build_context_window(self,
                           user_query: str,
                           retrieved_chunks: List[str],
                           system_prompt: Optional[str] = None,
                           conversation_history: Optional[List[dict]] = None) -> str:
        """
        Build a context window that fits within the token limits.

        Args:
            user_query: The user's question/query
            retrieved_chunks: List of relevant text chunks retrieved from knowledge base
            system_prompt: Optional system prompt to include
            conversation_history: Optional conversation history to maintain context

        Returns:
            Context window as a single string that fits within token limits
        """
        # Start with system prompt if provided
        context_parts = []
        if system_prompt:
            context_parts.append(f"System: {system_prompt}")

        # Add conversation history if provided
        if conversation_history:
            history_text = self._format_conversation_history(conversation_history)
            context_parts.append(f"Previous conversation: {history_text}")

        # Add retrieved context chunks
        if retrieved_chunks:
            context_chunks = "\n\n".join([f"Context {i+1}: {chunk}" for i, chunk in enumerate(retrieved_chunks)])
            context_parts.append(f"Relevant context:\n{context_chunks}")

        # Add the current user query
        context_parts.append(f"Current question: {user_query}")

        # Combine all parts
        full_context = "\n\n".join(context_parts)

        # Check if the full context fits within token limits
        full_context_tokens = self.token_counter.count_tokens(full_context)

        if full_context_tokens <= self.max_context_tokens:
            return full_context

        # If the context is too long, we need to trim it
        # First, preserve system prompt and user query (if they're not too long)
        essential_parts = []
        if system_prompt:
            essential_parts.append(f"System: {system_prompt}")
        essential_parts.append(f"Current question: {user_query}")

        essential_context = "\n\n".join(essential_parts)
        essential_tokens = self.token_counter.count_tokens(essential_context)

        if essential_tokens > self.max_context_tokens:
            # If even the essential parts don't fit, return just the user query
            return self.token_counter.truncate_text(user_query, self.max_context_tokens)

        # Calculate remaining tokens for context chunks
        remaining_tokens = self.max_context_tokens - essential_tokens

        # Add conversation history if there's space
        history_context = ""
        if conversation_history:
            history_text = self._format_conversation_history(conversation_history)
            history_tokens = self.token_counter.count_tokens(history_text)

            if history_tokens <= remaining_tokens:
                history_context = f"Previous conversation: {history_text}"
                remaining_tokens -= history_tokens
            else:
                # Truncate history to fit
                truncated_history = self.token_counter.truncate_text(history_text, remaining_tokens)
                history_context = f"Previous conversation: {truncated_history}"
                remaining_tokens = 0

        # Add context chunks if there's space
        context_chunk_text = ""
        if retrieved_chunks and remaining_tokens > 0:
            # Add chunks in order until we run out of tokens
            chunk_texts = [f"Context {i+1}: {chunk}" for i, chunk in enumerate(retrieved_chunks)]

            for chunk_text in chunk_texts:
                chunk_tokens = self.token_counter.count_tokens(chunk_text)
                if chunk_tokens <= remaining_tokens:
                    if context_chunk_text:
                        context_chunk_text += "\n\n" + chunk_text
                    else:
                        context_chunk_text = chunk_text
                    remaining_tokens -= chunk_tokens
                else:
                    # Try to truncate this chunk to fit the remaining space
                    if remaining_tokens > 0:
                        truncated_chunk = self.token_counter.truncate_text(chunk_text, remaining_tokens)
                        if context_chunk_text:
                            context_chunk_text += "\n\n" + truncated_chunk
                        else:
                            context_chunk_text = truncated_chunk
                    break

        # Build the final context
        final_parts = []
        if system_prompt:
            final_parts.append(f"System: {system_prompt}")

        if history_context:
            final_parts.append(history_context)

        if context_chunk_text:
            final_parts.append(f"Relevant context:\n{context_chunk_text}")

        final_parts.append(f"Current question: {user_query}")

        return "\n\n".join(final_parts)

    def _format_conversation_history(self, history: List[dict]) -> str:
        """
        Format conversation history into a string.

        Args:
            history: List of conversation turns with 'role' and 'content' keys

        Returns:
            Formatted conversation history as a string
        """
        formatted_turns = []
        for turn in history:
            role = turn.get('role', 'unknown')
            content = turn.get('content', '')
            formatted_turns.append(f"{role.capitalize()}: {content}")

        return "\n".join(formatted_turns)


def get_context_window_manager() -> ContextWindowManager:
    """
    Get the global context window manager instance.

    Returns:
        ContextWindowManager instance
    """
    # For now, we'll create a new instance each time
    # In a more complex app, you might want to maintain a global instance
    settings = get_settings()
    # Using a reasonable default for Gemini models
    return ContextWindowManager(max_context_tokens=3072)