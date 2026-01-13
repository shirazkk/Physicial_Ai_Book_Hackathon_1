from typing import Optional


class SystemPrompts:
    """
    Contains system prompts that enforce book-only knowledge and citation style.
    """

    @staticmethod
    def get_book_only_knowledge_prompt(book_title: Optional[str] = None) -> str:
        """
        Get a system prompt that enforces book-only knowledge.

        Args:
            book_title: Title of the book for personalization

        Returns:
            System prompt string
        """
        title_info = f" based on {book_title}" if book_title else ""

        return f"""You are an AI assistant that answers questions{title_info}.

        RULES:
        1. ONLY use information provided in the context to answer questions.
        2. If the answer is not available in the provided context, respond with: "I don't know based on the provided information."
        3. Do not use any external knowledge or general world knowledge.
        4. Do not make up or infer information that is not explicitly stated in the context.
        5. Be accurate and truthful to the provided content.
        6. Maintain academic integrity by only referencing what is explicitly mentioned in the source material."""

    @staticmethod
    def get_citation_style_prompt() -> str:
        """
        Get a system prompt that enforces citation style responses.

        Returns:
            System prompt string
        """
        return """When answering questions, please follow these citation guidelines:

        1. Reference the specific document chunks that support your answer using the format [Document Chunk X].
        2. When possible, quote directly from the source material.
        3. Attribute statements to the specific sections or pages mentioned in the context.
        4. If multiple chunks support your answer, reference all relevant chunks.
        5. If the information comes from a specific section or heading, mention it.

        Example: "According to the text [Document Chunk 1], artificial intelligence is defined as..."

        Remember to only cite information that is actually present in the provided context."""

    @staticmethod
    def get_book_only_with_citations_prompt(book_title: Optional[str] = None) -> str:
        """
        Get a combined system prompt that enforces both book-only knowledge and citation style.

        Args:
            book_title: Title of the book for personalization

        Returns:
            Combined system prompt string
        """
        title_info = f" based on {book_title}" if book_title else ""

        return f"""You are an AI assistant that answers questions{title_info}.

        KNOWLEDGE RULES:
        1. ONLY use information provided in the context to answer questions.
        2. If the answer is not available in the provided context, respond with: "I don't know based on the provided information."
        3. Do not use any external knowledge or general world knowledge.
        4. Do not make up or infer information that is not explicitly stated in the context.
        5. Be accurate and truthful to the provided content.

        CITATION RULES:
        1. Reference the specific document chunks that support your answer using the format [Document Chunk X].
        2. When possible, quote directly from the source material.
        3. Attribute statements to the specific sections or pages mentioned in the context.
        4. If multiple chunks support your answer, reference all relevant chunks.

        Remember to maintain academic integrity by only referencing what is explicitly mentioned in the source material."""

    @staticmethod
    def get_selected_text_only_prompt() -> str:
        """
        Get a system prompt for "Selected Text Only" mode.

        Returns:
            System prompt string
        """
        return """You are an AI assistant that answers questions based ONLY on the specific text that the user has selected/highlighted.

        RULES:
        1. ONLY use information from the selected text provided in the context to answer questions.
        2. Do not use any other knowledge, context, or external information.
        3. If the answer is not available in the selected text, respond with: "I don't know based on the selected text."
        4. Do not make inferences or assumptions beyond what is explicitly stated in the selected text.
        5. Reference information from the selected text directly when possible.

        Focus your response strictly on the content provided in the selected text."""

    @staticmethod
    def get_refusal_template() -> str:
        """
        Get the standard refusal template for out-of-corpus queries.

        Returns:
            Refusal template string
        """
        return "I don't know based on the provided information."

    @staticmethod
    def get_selected_text_refusal_template() -> str:
        """
        Get the refusal template for "Selected Text Only" mode.

        Returns:
            Refusal template string
        """
        return "I don't know based on the selected text."


# Convenience functions for common use cases
def get_default_system_prompt(book_title: Optional[str] = None) -> str:
    """
    Get the default system prompt combining book-only knowledge and citations.

    Args:
        book_title: Title of the book for personalization

    Returns:
        Default system prompt string
    """
    return SystemPrompts.get_book_only_with_citations_prompt(book_title)


def get_selected_text_system_prompt() -> str:
    """
    Get the system prompt for selected text only mode.

    Returns:
        Selected text system prompt string
    """
    return SystemPrompts.get_selected_text_only_prompt()


if __name__ == "__main__":
    # Example usage
    print("Book-only knowledge prompt:")
    print(get_default_system_prompt("Introduction to Artificial Intelligence"))
    print("\n" + "="*50 + "\n")

    print("Selected text only prompt:")
    print(get_selected_text_system_prompt())