from typing import Optional, List
from datetime import datetime
from uuid import uuid4
from sqlalchemy.orm import Session
from sqlalchemy import and_

from src.database.models import ChatSession as DBChatSession
from src.database.models import ChatMessage as DBChatMessage
from src.models.chat_session import ChatSession as DomainChatSession
from src.models.chat_message import ChatMessage as DomainChatMessage
from src.utils.logging import get_logger

logger = get_logger(__name__)


class SessionManager:
    """
    Manages the lifecycle of chat sessions including creation, retrieval, and updates.
    """

    def __init__(self, db_session: Session):
        """
        Initialize the session manager.

        Args:
            db_session: SQLAlchemy database session
        """
        self.db = db_session
        self.logger = get_logger(__name__)

    def create_session(self, user_id: Optional[str] = None, metadata: Optional[dict] = None) -> str:
        """
        Create a new chat session.

        Args:
            user_id: Optional user identifier
            metadata: Optional session metadata

        Returns:
            Session ID of the created session
        """
        session_id = str(uuid4())

        db_session = DBChatSession(
            session_id=session_id,
            user_id=user_id,
            metadata_json=str(metadata) if metadata else None
        )

        self.db.add(db_session)
        self.db.commit()

        self.logger.info(f"Created new session: {session_id}")
        return session_id

    def get_session(self, session_id: str) -> Optional[DomainChatSession]:
        """
        Retrieve a chat session by its ID.

        Args:
            session_id: Session ID to retrieve

        Returns:
            ChatSession object or None if not found
        """
        db_session = self.db.query(DBChatSession).filter(
            DBChatSession.session_id == session_id
        ).first()

        if not db_session:
            return None

        # Convert database model to domain model
        messages = self.get_messages(session_id)

        domain_session = DomainChatSession(
            session_id=db_session.session_id,
            user_id=db_session.user_id,
            created_at=db_session.created_at,
            updated_at=db_session.updated_at,
            metadata=eval(db_session.metadata_json) if db_session.metadata_json else {},
            messages=messages
        )

        self.logger.debug(f"Retrieved session: {session_id}")
        return domain_session

    def update_session_metadata(self, session_id: str, metadata: dict) -> bool:
        """
        Update the metadata for a session.

        Args:
            session_id: Session ID to update
            metadata: New metadata dictionary

        Returns:
            True if successful, False otherwise
        """
        db_session = self.db.query(DBChatSession).filter(
            DBChatSession.session_id == session_id
        ).first()

        if not db_session:
            self.logger.warning(f"Attempted to update non-existent session: {session_id}")
            return False

        db_session.metadata_json = str(metadata)
        db_session.updated_at = datetime.utcnow()

        self.db.commit()
        self.logger.info(f"Updated metadata for session: {session_id}")
        return True

    def delete_session(self, session_id: str) -> bool:
        """
        Delete a session (soft delete by marking inactive).

        Args:
            session_id: Session ID to delete

        Returns:
            True if successful, False otherwise
        """
        # In a real implementation, we might want soft deletion
        # For now, we'll do a hard delete
        result = self.db.query(DBChatSession).filter(
            DBChatSession.session_id == session_id
        ).delete()

        self.db.commit()

        if result > 0:
            self.logger.info(f"Deleted session: {session_id}")
            return True
        else:
            self.logger.warning(f"Attempted to delete non-existent session: {session_id}")
            return False

    def get_user_sessions(self, user_id: str) -> List[DomainChatSession]:
        """
        Get all sessions for a specific user.

        Args:
            user_id: User ID to get sessions for

        Returns:
            List of user's sessions
        """
        db_sessions = self.db.query(DBChatSession).filter(
            DBChatSession.user_id == user_id
        ).all()

        domain_sessions = []
        for db_session in db_sessions:
            messages = self.get_messages(db_session.session_id)

            domain_session = DomainChatSession(
                session_id=db_session.session_id,
                user_id=db_session.user_id,
                created_at=db_session.created_at,
                updated_at=db_session.updated_at,
                metadata=eval(db_session.metadata_json) if db_session.metadata_json else {},
                messages=messages
            )
            domain_sessions.append(domain_session)

        self.logger.debug(f"Retrieved {len(domain_sessions)} sessions for user: {user_id}")
        return domain_sessions

    def get_messages(self, session_id: str) -> List[DomainChatMessage]:
        """
        Get all messages for a specific session.

        Args:
            session_id: Session ID to get messages for

        Returns:
            List of messages in the session
        """
        db_messages = self.db.query(DBChatMessage).filter(
            DBChatMessage.session_id == session_id,
            DBChatMessage.is_active == True
        ).order_by(DBChatMessage.created_at.asc()).all()

        domain_messages = []
        for db_msg in db_messages:
            domain_msg = DomainChatMessage(
                message_id=db_msg.message_id,
                role=db_msg.role,
                content=db_msg.content,
                timestamp=db_msg.created_at,
                source_context=eval(db_msg.source_context) if db_msg.source_context else [],
                metadata={}  # Could add more metadata if needed
            )
            domain_messages.append(domain_msg)

        return domain_messages

    def session_exists(self, session_id: str) -> bool:
        """
        Check if a session exists.

        Args:
            session_id: Session ID to check

        Returns:
            True if session exists, False otherwise
        """
        count = self.db.query(DBChatSession).filter(
            DBChatSession.session_id == session_id
        ).count()

        return count > 0


def get_session_manager(db_session: Session) -> SessionManager:
    """
    Get a session manager instance.

    Args:
        db_session: SQLAlchemy database session

    Returns:
        SessionManager instance
    """
    return SessionManager(db_session)