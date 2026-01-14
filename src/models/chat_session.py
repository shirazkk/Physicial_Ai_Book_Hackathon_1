from __future__ import annotations  # Enable forward references
from dataclasses import dataclass
from typing import Optional, Dict, Any, List
from datetime import datetime
from .chat_message import ChatMessage  # Import here to avoid circular dependency


@dataclass
class ChatSession:
    """
    Represents a conversation between user and chatbot, containing session_id, user_id, and metadata.
    """
    session_id: str
    user_id: Optional[str] = None
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None
    metadata: Optional[Dict[str, Any]] = None
    messages: Optional[List[ChatMessage]] = None

    def __post_init__(self):
        if self.created_at is None:
            self.created_at = datetime.now()
        if self.updated_at is None:
            self.updated_at = datetime.now()
        if self.messages is None:
            self.messages = []
        if self.metadata is None:
            self.metadata = {}

    def add_message(self, message: ChatMessage) -> None:
        """Add a message to the session."""
        self.messages.append(message)
        self.updated_at = datetime.now()

    def get_messages(self) -> List[ChatMessage]:
        """Get all messages in the session."""
        return self.messages

    def to_dict(self) -> Dict[str, Any]:
        """Convert the chat session to a dictionary representation."""
        return {
            'session_id': self.session_id,
            'user_id': self.user_id,
            'created_at': self.created_at.isoformat() if self.created_at else None,
            'updated_at': self.updated_at.isoformat() if self.updated_at else None,
            'metadata': self.metadata,
            'messages': [msg.to_dict() for msg in self.messages]
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'ChatSession':
        """Create a ChatSession from a dictionary representation."""
        return cls(
            session_id=data['session_id'],
            user_id=data.get('user_id'),
            created_at=datetime.fromisoformat(data['created_at']) if data.get('created_at') else None,
            updated_at=datetime.fromisoformat(data['updated_at']) if data.get('updated_at') else None,
            metadata=data.get('metadata', {}),
            messages=[ChatMessage.from_dict(msg_data) for msg_data in data.get('messages', [])]
        )