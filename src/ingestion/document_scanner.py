import os
from pathlib import Path
from typing import List, Optional
from src.utils.logging import get_logger

logger = get_logger(__name__)


class DocumentScanner:
    """
    Scans the my-website/docs directory to discover all Markdown files.
    """

    def __init__(self, docs_directory: Optional[str] = None):
        """
        Initialize the document scanner.

        Args:
            docs_directory: Path to the docs directory. If None, defaults to 'my-website/docs'
        """
        self.docs_directory = docs_directory or "my-website/docs"
        self.logger = get_logger(__name__)

    def scan_documents(self) -> List[str]:
        """
        Scan the docs directory to find all Markdown files.

        Returns:
            List of file paths to Markdown files found in the directory
        """
        markdown_files = []
        docs_path = Path(self.docs_directory)

        if not docs_path.exists():
            self.logger.warning(f"Docs directory does not exist: {self.docs_directory}")
            return []

        # Walk through all subdirectories
        for root, dirs, files in os.walk(docs_path):
            for file in files:
                # Check if the file is a Markdown file
                if file.lower().endswith(('.md', '.mdx')):
                    full_path = os.path.join(root, file)
                    markdown_files.append(full_path)

        self.logger.info(f"Found {len(markdown_files)} Markdown files in {self.docs_directory}")
        return markdown_files

    def scan_documents_with_metadata(self) -> List[dict]:
        """
        Scan the docs directory to find all Markdown files with metadata.

        Returns:
            List of dictionaries containing file paths and metadata
        """
        markdown_files = []
        docs_path = Path(self.docs_directory)

        if not docs_path.exists():
            self.logger.warning(f"Docs directory does not exist: {self.docs_directory}")
            return []

        # Walk through all subdirectories
        for root, dirs, files in os.walk(docs_path):
            for file in files:
                # Check if the file is a Markdown file
                if file.lower().endswith(('.md', '.mdx')):
                    full_path = os.path.join(root, file)

                    # Get file stats
                    stat = os.stat(full_path)

                    # Create relative path from docs directory
                    relative_path = os.path.relpath(full_path, self.docs_directory)

                    file_info = {
                        'file_path': full_path,
                        'relative_path': relative_path,
                        'file_name': file,
                        'size': stat.st_size,
                        'modified': stat.st_mtime,
                        'directory': os.path.dirname(relative_path)
                    }

                    markdown_files.append(file_info)

        self.logger.info(f"Found {len(markdown_files)} Markdown files with metadata in {self.docs_directory}")
        return markdown_files


def get_all_markdown_files(docs_directory: Optional[str] = "my-website/docs") -> List[str]:
    """
    Convenience function to get all Markdown files from the docs directory.

    Args:
        docs_directory: Path to the docs directory

    Returns:
        List of file paths to Markdown files
    """
    scanner = DocumentScanner(docs_directory)
    return scanner.scan_documents()


if __name__ == "__main__":
    # Example usage
    scanner = DocumentScanner()
    files = scanner.scan_documents()

    print(f"Found {len(files)} Markdown files:")
    for file in files:
        print(f"  - {file}")