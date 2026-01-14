import re
from pathlib import Path
from typing import Dict, Any, List, Optional
from bs4 import BeautifulSoup
import markdown
from src.utils.logging import get_logger

logger = get_logger(__name__)


class ContentExtractor:
    """
    Extracts and normalizes text content from Markdown files.
    Preserves semantic meaning while removing formatting.
    """

    def __init__(self):
        self.logger = get_logger(__name__)

    def extract_content(self, file_path: str) -> str:
        """
        Extract text content from a Markdown file.

        Args:
            file_path: Path to the Markdown file

        Returns:
            Extracted text content
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                content = file.read()

            # Parse markdown to HTML first
            html_content = markdown.markdown(content)

            # Use BeautifulSoup to extract plain text
            soup = BeautifulSoup(html_content, 'html.parser')

            # Extract text and normalize whitespace
            text_content = soup.get_text()
            normalized_content = self._normalize_text(text_content)

            return normalized_content
        except Exception as e:
            self.logger.error(f"Error extracting content from {file_path}: {str(e)}")
            raise

    def extract_content_with_metadata(self, file_path: str) -> Dict[str, Any]:
        """
        Extract text content from a Markdown file with metadata.

        Args:
            file_path: Path to the Markdown file

        Returns:
            Dictionary containing content and metadata
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                raw_content = file.read()

            # Extract metadata from frontmatter if present
            metadata, content_without_frontmatter = self._extract_frontmatter(raw_content)

            # Parse markdown to HTML first
            html_content = markdown.markdown(content_without_frontmatter)

            # Use BeautifulSoup to extract plain text
            soup = BeautifulSoup(html_content, 'html.parser')

            # Extract text and normalize whitespace
            text_content = soup.get_text()
            normalized_content = self._normalize_text(text_content)

            # Extract structural information
            structural_info = self._extract_structural_info(content_without_frontmatter)

            result = {
                'content': normalized_content,
                'raw_content': raw_content,
                'metadata': metadata,
                'structural_info': structural_info,
                'file_path': file_path,
                'file_name': Path(file_path).name
            }

            return result
        except Exception as e:
            self.logger.error(f"Error extracting content with metadata from {file_path}: {str(e)}")
            raise

    def _extract_frontmatter(self, content: str) -> tuple[Dict[str, Any], str]:
        """
        Extract frontmatter metadata from Markdown content if present.

        Args:
            content: Raw Markdown content

        Returns:
            Tuple of (metadata dict, content without frontmatter)
        """
        # Check for YAML frontmatter
        if content.startswith('---'):
            parts = content.split('---', 2)
            if len(parts) >= 3:
                import yaml
                try:
                    metadata = yaml.safe_load(parts[1])
                    if metadata is None:
                        metadata = {}
                    return metadata, parts[2].strip()
                except yaml.YAMLError:
                    # If YAML parsing fails, return empty metadata
                    self.logger.warning("Failed to parse YAML frontmatter, returning empty metadata")
                    return {}, content

        return {}, content

    def _extract_structural_info(self, content: str) -> Dict[str, Any]:
        """
        Extract structural information from Markdown content.

        Args:
            content: Markdown content without frontmatter

        Returns:
            Dictionary containing structural information
        """
        # Extract headings
        headings = []
        heading_pattern = r'^(#{1,6})\s+(.+)$'
        lines = content.split('\n')

        for line in lines:
            match = re.match(heading_pattern, line.strip())
            if match:
                level = len(match.group(1))
                title = match.group(2).strip()
                headings.append({
                    'level': level,
                    'title': title
                })

        # Extract code blocks
        code_blocks = re.findall(r'```.*?\n(.*?)```', content, re.DOTALL)

        return {
            'headings': headings,
            'code_blocks_count': len(code_blocks),
            'word_count': len(content.split()),
            'character_count': len(content)
        }

    def _normalize_text(self, text: str) -> str:
        """
        Normalize text by cleaning up whitespace and special characters.

        Args:
            text: Raw text content

        Returns:
            Normalized text
        """
        # Replace multiple newlines with double newline
        text = re.sub(r'\n\s*\n', '\n\n', text)

        # Replace multiple spaces with single space
        text = re.sub(r' +', ' ', text)

        # Remove leading/trailing whitespace from each line
        lines = [line.strip() for line in text.split('\n')]

        # Join lines back together
        normalized = '\n'.join(lines)

        # Remove excessive whitespace at the beginning and end
        normalized = normalized.strip()

        return normalized

    def extract_content_batch(self, file_paths: List[str]) -> List[Dict[str, Any]]:
        """
        Extract content from multiple Markdown files.

        Args:
            file_paths: List of file paths to process

        Returns:
            List of dictionaries containing content and metadata for each file
        """
        results = []
        for file_path in file_paths:
            try:
                result = self.extract_content_with_metadata(file_path)
                results.append(result)
                self.logger.info(f"Successfully processed {file_path}")
            except Exception as e:
                self.logger.error(f"Failed to process {file_path}: {str(e)}")
                # Add error information to results
                results.append({
                    'file_path': file_path,
                    'error': str(e),
                    'content': '',
                    'metadata': {},
                    'structural_info': {}
                })

        return results


def extract_markdown_content(file_path: str) -> str:
    """
    Convenience function to extract content from a Markdown file.

    Args:
        file_path: Path to the Markdown file

    Returns:
        Extracted text content
    """
    extractor = ContentExtractor()
    return extractor.extract_content(file_path)


def extract_markdown_content_with_metadata(file_path: str) -> Dict[str, Any]:
    """
    Convenience function to extract content with metadata from a Markdown file.

    Args:
        file_path: Path to the Markdown file

    Returns:
        Dictionary containing content and metadata
    """
    extractor = ContentExtractor()
    return extractor.extract_content_with_metadata(file_path)


if __name__ == "__main__":
    # Example usage
    extractor = ContentExtractor()
    import sys

    if len(sys.argv) > 1:
        file_path = sys.argv[1]
        content = extractor.extract_content(file_path)
        print(f"Content from {file_path}:")
        print(content[:500] + "..." if len(content) > 500 else content)
    else:
        print("Usage: python content_extractor.py <markdown_file_path>")