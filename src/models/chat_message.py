from dataclasses import dataclass
from typing import Optional, Dict, Any, List
from datetime import datetime


@dataclass
class ChatMessage:
    """
    Represents a single exchange in a conversation, containing timestamp,
    user input, AI response, and source context.
    """
    message_id: str
    role: str  # 'user' or 'assistant'
    content: str
    timestamp: Optional[datetime] = None
    source_context: Optional[List[Dict[str, Any]]] = None  # Context used to generate response
    metadata: Optional[Dict[str, Any]] = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now()
        if self.source_context is None:
            self.source_context = []
        if self.metadata is None:
            self.metadata = {}

    def to_dict(self) -> Dict[str, Any]:
        """Convert the chat message to a dictionary representation."""
        return {
            'message_id': self.message_id,
            'role': self.role,
            'content': self.content,
            'timestamp': self.timestamp.isoformat() if self.timestamp else None,
            'source_context': self.source_context,
            'metadata': self.metadata
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ChatMessage':
        """Create a ChatMessage from a dictionary representation."""
        return cls(
            message_id=data['message_id'],
            role=data['role'],
            content=data['content'],
            timestamp=datetime.fromisoformat(data['timestamp']) if data.get('timestamp') else None,
            source_context=data.get('source_context', []),
            metadata=data.get('metadata', {})
        )