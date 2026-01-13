from dataclasses import dataclass
from typing import Optional, Dict, Any
from datetime import datetime


@dataclass
class DocumentChunk:
    """
    Represents a piece of book content that has been processed and embedded.

    This entity contains the chunk_id, source metadata (module, chapter, file path, heading),
    and the actual text content as specified in the requirements.
    """
    chunk_id: str
    text_content: str
    source_metadata: Dict[str, Any]
    embedding: Optional[list] = None
    created_at: Optional[datetime] = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now()

    @property
    def module(self) -> Optional[str]:
        """Get the module from source metadata."""
        return self.source_metadata.get('module')

    @property
    def chapter(self) -> Optional[str]:
        """Get the chapter from source metadata."""
        return self.source_metadata.get('chapter')

    @property
    def file_path(self) -> Optional[str]:
        """Get the file path from source metadata."""
        return self.source_metadata.get('file_path')

    @property
    def heading(self) -> Optional[str]:
        """Get the heading from source metadata."""
        return self.source_metadata.get('heading')

    def to_dict(self) -> Dict[str, Any]:
        """Convert the document chunk to a dictionary representation."""
        return {
            'chunk_id': self.chunk_id,
            'text_content': self.text_content,
            'source_metadata': self.source_metadata,
            'embedding': self.embedding,
            'created_at': self.created_at.isoformat() if self.created_at else None
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'DocumentChunk':
        """Create a DocumentChunk from a dictionary representation."""
        return cls(
            chunk_id=data['chunk_id'],
            text_content=data['text_content'],
            source_metadata=data['source_metadata'],
            embedding=data.get('embedding'),
            created_at=datetime.fromisoformat(data['created_at']) if data.get('created_at') else None
        )