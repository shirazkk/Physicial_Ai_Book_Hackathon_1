import re
from typing import Optional, Union
from pydantic import validator, BaseModel
from src.utils.logging import get_logger

logger = get_logger(__name__)


class InputValidator:
    """
    Utility class for validating and sanitizing user inputs.
    """

    @staticmethod
    def validate_session_id(session_id: str) -> bool:
        """
        Validate session ID format.

        Args:
            session_id: Session ID to validate

        Returns:
            True if valid, False otherwise
        """
        if not session_id:
            return False

        # Allow UUID format or alphanumeric with hyphens/underscores
        pattern = r'^[a-zA-Z0-9_-]{8,64}$'
        is_valid = bool(re.match(pattern, session_id))

        if not is_valid:
            logger.warning(f"Invalid session ID format: {session_id}")

        return is_valid

    @staticmethod
    def validate_user_id(user_id: str) -> bool:
        """
        Validate user ID format.

        Args:
            user_id: User ID to validate

        Returns:
            True if valid, False otherwise
        """
        if not user_id:
            return True  # User ID can be optional

        # Allow UUID format or alphanumeric with hyphens/underscores
        pattern = r'^[a-zA-Z0-9_-]{1,64}$'
        is_valid = bool(re.match(pattern, user_id))

        if not is_valid:
            logger.warning(f"Invalid user ID format: {user_id}")

        return is_valid

    @staticmethod
    def validate_question(question: str) -> tuple[bool, str]:
        """
        Validate question text.

        Args:
            question: Question text to validate

        Returns:
            Tuple of (is_valid, sanitized_question)
        """
        if not question or not question.strip():
            return False, ""

        # Remove excessive whitespace
        sanitized = " ".join(question.split())

        # Check for minimum length
        if len(sanitized) < 3:
            logger.warning("Question is too short")
            return False, sanitized

        # Check for maximum length
        if len(sanitized) > 1000:
            logger.warning("Question exceeds maximum length")
            return False, sanitized[:1000]

        # Sanitize potential harmful content
        sanitized = InputValidator.sanitize_text(sanitized)

        return True, sanitized

    @staticmethod
    def validate_selected_text(selected_text: str) -> tuple[bool, str]:
        """
        Validate selected text.

        Args:
            selected_text: Selected text to validate

        Returns:
            Tuple of (is_valid, sanitized_text)
        """
        if not selected_text or not selected_text.strip():
            return False, ""

        # Remove excessive whitespace
        sanitized = " ".join(selected_text.split())

        # Check for maximum length
        if len(sanitized) > 5000:  # Reasonable limit for selected text
            logger.warning("Selected text exceeds maximum length")
            return False, sanitized[:5000]

        # Sanitize potential harmful content
        sanitized = InputValidator.sanitize_text(sanitized)

        return True, sanitized

    @staticmethod
    def sanitize_text(text: str) -> str:
        """
        Sanitize text input to remove potentially harmful content.

        Args:
            text: Text to sanitize

        Returns:
            Sanitized text
        """
        if not text:
            return text

        # Remove potential script tags
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', text, flags=re.IGNORECASE | re.DOTALL)

        # Remove potential javascript: urls
        sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)

        # Remove potential data: urls
        sanitized = re.sub(r'data:', '', sanitized, flags=re.IGNORECASE)

        # Remove potential vbscript: urls
        sanitized = re.sub(r'vbscript:', '', sanitized, flags=re.IGNORECASE)

        # Remove potential href="javascript:" patterns
        sanitized = re.sub(r'href\s*=\s*["\'][^"\']*javascript:[^"\']*["\']', 'href="#"', sanitized, flags=re.IGNORECASE)

        # Escape HTML brackets to prevent injection
        sanitized = sanitized.replace('<', '&lt;').replace('>', '&gt;')

        return sanitized

    @staticmethod
    def validate_file_path(file_path: str) -> bool:
        """
        Validate file path to prevent directory traversal.

        Args:
            file_path: File path to validate

        Returns:
            True if valid, False otherwise
        """
        if not file_path:
            return False

        # Check for directory traversal attempts
        if '../' in file_path or '..\\' in file_path:
            logger.warning(f"Directory traversal attempt detected: {file_path}")
            return False

        # Additional validation can be added based on requirements
        return True

    @staticmethod
    def validate_metadata(metadata: dict) -> bool:
        """
        Validate metadata dictionary.

        Args:
            metadata: Metadata to validate

        Returns:
            True if valid, False otherwise
        """
        if not metadata:
            return True  # Metadata can be optional

        # Check that metadata is a dictionary
        if not isinstance(metadata, dict):
            logger.warning("Metadata is not a dictionary")
            return False

        # Check for maximum size
        if len(str(metadata)) > 10000:  # 10KB limit
            logger.warning("Metadata exceeds size limit")
            return False

        # Validate keys and values
        for key, value in metadata.items():
            if not isinstance(key, str) or len(str(key)) > 100:
                logger.warning(f"Invalid metadata key: {key}")
                return False

            if len(str(value)) > 1000:
                logger.warning(f"Metadata value too long for key: {key}")
                return False

        return True


# Convenience functions
def validate_session_id(session_id: str) -> bool:
    """Validate session ID."""
    return InputValidator.validate_session_id(session_id)


def validate_user_id(user_id: str) -> bool:
    """Validate user ID."""
    return InputValidator.validate_user_id(user_id)


def validate_question(question: str) -> tuple[bool, str]:
    """Validate question text."""
    return InputValidator.validate_question(question)


def validate_selected_text(selected_text: str) -> tuple[bool, str]:
    """Validate selected text."""
    return InputValidator.validate_selected_text(selected_text)


def sanitize_text(text: str) -> str:
    """Sanitize text input."""
    return InputValidator.sanitize_text(text)


# Pydantic validators
def validate_session_id_pydantic(v: str) -> str:
    """Pydantic validator for session ID."""
    if not InputValidator.validate_session_id(v):
        raise ValueError('Invalid session ID format')
    return v


def validate_question_pydantic(v: str) -> str:
    """Pydantic validator for question."""
    is_valid, sanitized = InputValidator.validate_question(v)
    if not is_valid:
        raise ValueError('Invalid question format')
    return sanitized


if __name__ == "__main__":
    # Example usage
    validator = InputValidator()

    # Test question validation
    test_question = "What is artificial intelligence?"
    is_valid, sanitized = validator.validate_question(test_question)
    print(f"Question '{test_question}' is valid: {is_valid}, sanitized: '{sanitized}'")

    # Test malicious input
    malicious_input = '<script>alert("xss")</script> Hello World'
    is_valid, sanitized = validator.validate_question(malicious_input)
    print(f"Malicious input '{malicious_input}' is valid: {is_valid}, sanitized: '{sanitized}'")