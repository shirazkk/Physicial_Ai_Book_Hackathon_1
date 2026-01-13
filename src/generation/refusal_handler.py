from typing import Dict, Any, Optional
from src.utils.exceptions import GenerationError
from src.utils.logging import get_logger
from src.generation.system_prompts import SystemPrompts

logger = get_logger(__name__)


class RefusalHandler:
    """
    Handles graceful refusal for out-of-corpus queries.
    """

    def __init__(self):
        """
        Initialize the refusal handler.
        """
        self.default_refusal_message = SystemPrompts.get_refusal_template()
        self.selected_text_refusal_message = SystemPrompts.get_selected_text_refusal_template()
        self.logger = get_logger(__name__)

    def handle_out_of_corpus_query(self,
                                  query: str,
                                  context_available: bool = False,
                                  selected_text_mode: bool = False) -> str:
        """
        Handle a query that cannot be answered from the corpus.

        Args:
            query: The original query that couldn't be answered
            context_available: Whether relevant context was available
            selected_text_mode: Whether in "Selected Text Only" mode

        Returns:
            Appropriate refusal response
        """
        try:
            if selected_text_mode:
                refusal_message = self.selected_text_refusal_message
                self.logger.info(f"Handling out-of-corpus query in selected text mode: '{query[:50]}...'")
            else:
                refusal_message = self.default_refusal_message
                self.logger.info(f"Handling out-of-corpus query: '{query[:50]}...'")

            # Log the refused query for potential analysis
            self.logger.debug(f"Refused query: {query}")

            return refusal_message

        except Exception as e:
            self.logger.error(f"Error handling out-of-corpus query: {str(e)}")
            # Return a safe default refusal message
            return self.default_refusal_message

    def is_refusal_response(self, response: str) -> bool:
        """
        Check if a response is a refusal response.

        Args:
            response: Response text to check

        Returns:
            True if the response is a refusal, False otherwise
        """
        normalized_response = response.strip().lower()

        # Check for variations of the refusal messages
        refusal_indicators = [
            self.default_refusal_message.lower(),
            self.selected_text_refusal_message.lower(),
            "i don't know based on the provided information",
            "i don't know based on the selected text",
            "i cannot answer",
            "i'm unable to answer",
            "the information is not provided",
            "not mentioned in the context",
            "not available in the provided context"
        ]

        for indicator in refusal_indicators:
            if indicator in normalized_response:
                return True

        return False

    def generate_context_aware_refusal(self,
                                     query: str,
                                     retrieved_chunks: Optional[list] = None,
                                     min_similarity_threshold: float = 0.3,
                                     selected_text_mode: bool = False) -> str:
        """
        Generate a refusal response based on the quality of retrieved context.

        Args:
            query: The original query
            retrieved_chunks: List of retrieved chunks with scores
            min_similarity_threshold: Minimum similarity score to consider chunks relevant
            selected_text_mode: Whether in "Selected Text Only" mode

        Returns:
            Appropriate refusal response
        """
        try:
            if retrieved_chunks is None:
                retrieved_chunks = []

            # Check if any chunks meet the similarity threshold
            relevant_chunks = [
                chunk for chunk in retrieved_chunks
                if chunk.get('score', 0) >= min_similarity_threshold
            ]

            if not relevant_chunks:
                # No relevant context found
                self.logger.info(f"No relevant context found for query: '{query[:50]}...'")
                return self.handle_out_of_corpus_query(
                    query,
                    context_available=False,
                    selected_text_mode=selected_text_mode
                )
            else:
                # We have relevant context but still need to refuse
                # This could happen if the content is in the context but doesn't answer the query
                self.logger.info(f"Have relevant context but cannot answer query: '{query[:50]}...'")
                return self.handle_out_of_corpus_query(
                    query,
                    context_available=True,
                    selected_text_mode=selected_text_mode
                )

        except Exception as e:
            self.logger.error(f"Error generating context-aware refusal: {str(e)}")
            return self.default_refusal_message

    def format_refusal_with_alternatives(self,
                                       query: str,
                                       alternative_suggestions: Optional[list] = None,
                                       selected_text_mode: bool = False) -> str:
        """
        Format a refusal response with alternative suggestions.

        Args:
            query: The original query
            alternative_suggestions: List of alternative suggestions
            selected_text_mode: Whether in "Selected Text Only" mode

        Returns:
            Refusal response with alternatives
        """
        try:
            base_refusal = self.handle_out_of_corpus_query(
                query,
                selected_text_mode=selected_text_mode
            )

            if not alternative_suggestions:
                return base_refusal

            # Add alternative suggestions to the refusal
            alternatives_text = "\n\nYou might find these related topics helpful:"
            for i, suggestion in enumerate(alternative_suggestions[:3]):  # Limit to 3 suggestions
                alternatives_text += f"\n{i+1}. {suggestion}"

            return base_refusal + alternatives_text

        except Exception as e:
            self.logger.error(f"Error formatting refusal with alternatives: {str(e)}")
            return self.handle_out_of_corpus_query(query, selected_text_mode=selected_text_mode)

    def get_refusal_metrics(self, response: str) -> Dict[str, Any]:
        """
        Get metrics about a refusal response.

        Args:
            response: Response to analyze

        Returns:
            Dictionary with refusal metrics
        """
        try:
            is_refusal = self.is_refusal_response(response)
            refusal_type = self._classify_refusal_type(response)

            metrics = {
                'is_refusal': is_refusal,
                'refusal_type': refusal_type,
                'response_length': len(response),
                'contains_refusal_template': any(
                    template.lower() in response.lower()
                    for template in [
                        self.default_refusal_message.lower(),
                        self.selected_text_refusal_message.lower()
                    ]
                )
            }

            return metrics

        except Exception as e:
            self.logger.error(f"Error getting refusal metrics: {str(e)}")
            return {
                'is_refusal': self.is_refusal_response(response),
                'response_length': len(response),
                'error': str(e)
            }

    def _classify_refusal_type(self, response: str) -> str:
        """
        Classify the type of refusal response.

        Args:
            response: Response to classify

        Returns:
            Refusal type classification
        """
        normalized_response = response.strip().lower()

        if self.selected_text_refusal_message.lower() in normalized_response:
            return "selected_text_only_refusal"
        elif self.default_refusal_message.lower() in normalized_response:
            return "general_corpus_refusal"
        elif "i don't know" in normalized_response:
            return "unkown_refusal"
        elif "cannot answer" in normalized_response:
            return "cannot_answer_refusal"
        else:
            return "other_refusal"


def get_refusal_handler() -> RefusalHandler:
    """
    Get a singleton instance of the refusal handler.

    Returns:
        RefusalHandler instance
    """
    return RefusalHandler()


def handle_refusal(query: str, selected_text_mode: bool = False) -> str:
    """
    Convenience function to handle a refusal.

    Args:
        query: The query that couldn't be answered
        selected_text_mode: Whether in "Selected Text Only" mode

    Returns:
        Appropriate refusal response
    """
    handler = get_refusal_handler()
    return handler.handle_out_of_corpus_query(query, selected_text_mode=selected_text_mode)


if __name__ == "__main__":
    # Example usage
    handler = get_refusal_handler()

    # Test a regular out-of-corpus query
    query = "What is the weather like today?"
    refusal = handler.handle_out_of_corpus_query(query)
    print(f"Refusal for query '{query}': {refusal}")

    # Test selected text mode refusal
    selected_text_refusal = handler.handle_out_of_corpus_query(query, selected_text_mode=True)
    print(f"Selected text refusal: {selected_text_refusal}")

    # Test refusal detection
    test_responses = [
        "I don't know based on the provided information.",
        "The answer is in chapter 3 of the book.",
        "I don't know based on the selected text."
    ]

    for resp in test_responses:
        is_refusal = handler.is_refusal_response(resp)
        metrics = handler.get_refusal_metrics(resp)
        print(f"Response: '{resp}' | Is refusal: {is_refusal} | Metrics: {metrics}")