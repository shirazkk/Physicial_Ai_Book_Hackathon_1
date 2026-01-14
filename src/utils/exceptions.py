from typing import Optional
from fastapi import HTTPException, status


class RAGChatException(Exception):
    """
    Base exception class for RAG Chatbot application.
    All custom exceptions should inherit from this class.
    """
    def __init__(self, message: str, error_code: Optional[str] = None):
        self.message = message
        self.error_code = error_code
        super().__init__(self.message)


class DocumentProcessingError(RAGChatException):
    """
    Raised when there's an error during document processing.
    """
    pass


class EmbeddingGenerationError(RAGChatException):
    """
    Raised when there's an error generating embeddings.
    """
    pass


class VectorStorageError(RAGChatException):
    """
    Raised when there's an error with vector storage operations.
    """
    pass


class RetrievalError(RAGChatException):
    """
    Raised when there's an error during content retrieval.
    """
    pass


class GenerationError(RAGChatException):
    """
    Raised when there's an error during response generation.
    """
    pass


class ChatHistoryError(RAGChatException):
    """
    Raised when there's an error with chat history operations.
    """
    pass


class ConfigurationError(RAGChatException):
    """
    Raised when there's an error with application configuration.
    """
    pass


class ValidationError(RAGChatException):
    """
    Raised when input validation fails.
    """
    pass


# HTTP Exception Utilities
def create_not_found_exception(detail: str) -> HTTPException:
    """
    Create a 404 HTTP exception.

    Args:
        detail: Description of the error

    Returns:
        HTTPException with 404 status code
    """
    return HTTPException(
        status_code=status.HTTP_404_NOT_FOUND,
        detail=detail
    )


def create_bad_request_exception(detail: str) -> HTTPException:
    """
    Create a 400 HTTP exception.

    Args:
        detail: Description of the error

    Returns:
        HTTPException with 400 status code
    """
    return HTTPException(
        status_code=status.HTTP_400_BAD_REQUEST,
        detail=detail
    )


def create_internal_error_exception(detail: str = "Internal server error") -> HTTPException:
    """
    Create a 500 HTTP exception.

    Args:
        detail: Description of the error (optional)

    Returns:
        HTTPException with 500 status code
    """
    return HTTPException(
        status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
        detail=detail
    )


def create_rate_limit_exception(detail: str = "Rate limit exceeded") -> HTTPException:
    """
    Create a 429 HTTP exception for rate limiting.

    Args:
        detail: Description of the error (optional)

    Returns:
        HTTPException with 429 status code
    """
    return HTTPException(
        status_code=status.HTTP_429_TOO_MANY_REQUESTS,
        detail=detail
    )