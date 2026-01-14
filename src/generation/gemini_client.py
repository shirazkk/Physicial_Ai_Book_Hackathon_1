from typing import List, Dict, Any, Optional
import google.generativeai as genai
from src.utils.config import get_settings
from src.utils.exceptions import GenerationError
from src.utils.logging import get_logger

logger = get_logger(__name__)


class GeminiChatClient:
    """
    Client for generating responses using the Google Gemini chat API.
    """

    def __init__(self, api_key: Optional[str] = None, model_name: Optional[str] = None):
        """
        Initialize the Gemini chat client.

        Args:
            api_key: Gemini API key. If None, will be loaded from config.
            model_name: Name of the chat model to use. If None, will be loaded from config.
        """
        settings = get_settings()

        self.api_key = api_key or settings.gemini_api_key
        self.model_name = model_name or settings.gemini_chat_model

        # Configure the API key
        genai.configure(api_key=self.api_key)

        # Initialize the generative model
        self.model = genai.GenerativeModel(self.model_name)
        self.logger = get_logger(__name__)

    def generate_response(self,
                        prompt: str,
                        context: Optional[str] = None,
                        temperature: float = 0.7,
                        max_output_tokens: int = 1024) -> str:
        """
        Generate a response to a prompt with optional context.

        Args:
            prompt: User's question or prompt
            context: Optional context to provide to the model
            temperature: Controls randomness in the response (0.0 to 1.0)
            max_output_tokens: Maximum number of tokens in the response

        Returns:
            Generated response text
        """
        try:
            if not prompt.strip():
                raise GenerationError("Prompt cannot be empty")

            # Build the full prompt with context if provided
            full_prompt = prompt
            if context:
                full_prompt = f"{context}\n\nQuestion: {prompt}"

            # Generate content using the model
            response = self.model.generate_content(
                full_prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=temperature,
                    max_output_tokens=max_output_tokens,
                )
            )

            # Extract the text from the response
            if response.candidates and len(response.candidates) > 0:
                candidate = response.candidates[0]
                if hasattr(candidate, 'content') and candidate.content and hasattr(candidate.content, 'parts') and candidate.content.parts:
                    generated_text = candidate.content.parts[0].text
                    self.logger.debug(f"Generated response with {len(generated_text)} characters")
                    return generated_text
                else:
                    raise GenerationError("No content parts found in the response candidate")
            else:
                raise GenerationError("No candidates returned from Gemini model")

        except Exception as e:
            self.logger.error(f"Error generating response: {str(e)}")
            raise GenerationError(f"Failed to generate response: {str(e)}")

    def generate_response_with_citations(self,
                                      prompt: str,
                                      context: str,
                                      temperature: float = 0.7,
                                      max_output_tokens: int = 1024) -> Dict[str, Any]:
        """
        Generate a response with citation information based on the provided context.

        Args:
            prompt: User's question or prompt
            context: Context containing document chunks with source information
            temperature: Controls randomness in the response (0.0 to 1.0)
            max_output_tokens: Maximum number of tokens in the response

        Returns:
            Dictionary containing the response and citation information
        """
        try:
            # Create a prompt that encourages citing sources
            citation_prompt = f"""
            {context}

            Based strictly on the information provided above, answer the following question:
            {prompt}

            If the answer is not available in the provided context, respond with:
            "I don't know based on the provided information."

            When providing an answer, please cite the relevant document chunks by referring to them as
            "[Document Chunk X]" where X corresponds to the chunk number in the context.
            """

            response_text = self.generate_response(
                prompt=citation_prompt,
                context=None,  # Context is already in the prompt
                temperature=temperature,
                max_output_tokens=max_output_tokens
            )

            # Extract citation information (basic approach)
            citations = self._extract_citations(response_text, context)

            result = {
                'response': response_text,
                'citations': citations,
                'has_citations': len(citations) > 0
            }

            self.logger.debug(f"Generated response with {len(citations)} citations")
            return result

        except Exception as e:
            self.logger.error(f"Error generating response with citations: {str(e)}")
            raise GenerationError(f"Failed to generate response with citations: {str(e)}")

    def _extract_citations(self, response: str, context: str) -> List[Dict[str, Any]]:
        """
        Extract citation information from the response and context.

        Args:
            response: Generated response text
            context: Original context with document chunks

        Returns:
            List of citation information
        """
        citations = []

        # Look for patterns like "[Document Chunk X]" in the response
        import re
        citation_pattern = r'\[Document Chunk (\d+)\]'
        matches = re.findall(citation_pattern, response)

        for match in matches:
            chunk_num = int(match)
            # This is a simplified approach - in a real implementation,
            # you'd extract the actual source information for each chunk
            citation = {
                'chunk_number': chunk_num,
                'text_excerpt': f"Referenced document chunk {chunk_num}"
            }
            citations.append(citation)

        return citations

    def chat_with_history(self,
                         user_input: str,
                         conversation_history: Optional[List[Dict[str, str]]] = None,
                         context: Optional[str] = None,
                         temperature: float = 0.7) -> str:
        """
        Generate a response considering conversation history.

        Args:
            user_input: Current user input
            conversation_history: List of previous messages in the format:
                                [{'role': 'user'|'model', 'content': 'message'}]
            context: Optional context to provide to the model
            temperature: Controls randomness in the response (0.0 to 1.0)

        Returns:
            Generated response text
        """
        try:
            # Prepare the conversation history
            history = conversation_history or []

            # Build the full prompt with context and history
            full_prompt = ""
            if context:
                full_prompt += f"{context}\n\n"

            # Add conversation history
            for msg in history:
                role = "User" if msg['role'] == 'user' else "Assistant"
                full_prompt += f"{role}: {msg['content']}\n"

            # Add current user input
            full_prompt += f"User: {user_input}\nAssistant:"

            # Generate the response
            response = self.model.generate_content(
                full_prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=temperature,
                    max_output_tokens=1024,
                )
            )

            if response.candidates and len(response.candidates) > 0:
                candidate = response.candidates[0]
                if hasattr(candidate, 'content') and candidate.content and hasattr(candidate.content, 'parts') and candidate.content.parts:
                    generated_text = candidate.content.parts[0].text
                    self.logger.debug(f"Generated chat response with {len(generated_text)} characters")
                    return generated_text
                else:
                    raise GenerationError("No content parts found in the response candidate")
            else:
                raise GenerationError("No candidates returned from Gemini model")

        except Exception as e:
            self.logger.error(f"Error in chat with history: {str(e)}")
            raise GenerationError(f"Failed to generate chat response: {str(e)}")


def get_gemini_chat_client() -> GeminiChatClient:
    """
    Get a singleton instance of the Gemini chat client.

    Returns:
        GeminiChatClient instance
    """
    settings = get_settings()
    return GeminiChatClient(
        api_key=settings.gemini_api_key,
        model_name=settings.gemini_chat_model
    )


# Convenience function for generating a response
def generate_response(prompt: str, context: Optional[str] = None) -> str:
    """
    Generate a response using the default Gemini chat client.

    Args:
        prompt: User's question or prompt
        context: Optional context to provide to the model

    Returns:
        Generated response text
    """
    client = get_gemini_chat_client()
    return client.generate_response(prompt, context)


if __name__ == "__main__":
    # Example usage
    import os

    try:
        client = get_gemini_chat_client()

        # Example context and question
        context = """
        [Document Chunk 1]
        Artificial intelligence (AI) is intelligence demonstrated by machines, in contrast to the natural intelligence displayed by humans and animals. Leading AI textbooks define the field as the study of "intelligent agents".

        [Document Chunk 2]
        Machine learning is a method of data analysis that automates analytical model building. It is a branch of artificial intelligence based on the idea that systems can learn from data.
        """

        question = "What is artificial intelligence?"

        print("Generating response...")
        result = client.generate_response_with_citations(question, context)

        print(f"Response: {result['response']}")
        print(f"Citations: {result['citations']}")

    except Exception as e:
        print(f"Error: {e}")
        print("Make sure your GEMINI_API_KEY is set in the environment")