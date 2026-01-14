from sqlalchemy import Column, Integer, String, DateTime, Text, ForeignKey, Boolean
from sqlalchemy.orm import declarative_base, relationship
from sqlalchemy.sql import func
from typing import List
import uuid
from datetime import datetime

Base = declarative_base()


class ChatSession(Base):
    """
    SQLAlchemy model for chat sessions.
    """
    __tablename__ = "chat_sessions"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    session_id = Column(String, unique=True, nullable=False, index=True)
    user_id = Column(String, nullable=True, index=True)  # Can be anonymous
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now(), server_default=func.now())
    metadata_json = Column(Text, nullable=True)  # JSON metadata for the session

    # Relationship to messages
    messages = relationship("ChatMessage", back_populates="session", cascade="all, delete-orphan")


class ChatMessage(Base):
    """
    SQLAlchemy model for chat messages.
    """
    __tablename__ = "chat_messages"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    message_id = Column(String, unique=True, nullable=False, index=True)
    session_id = Column(String, ForeignKey("chat_sessions.session_id"), nullable=False)
    role = Column(String(20), nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    source_context = Column(Text, nullable=True)  # JSON of source context used
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    is_active = Column(Boolean, default=True, nullable=False)  # Soft delete flag

    # Relationship to session
    session = relationship("ChatSession", back_populates="messages")


# Alternative models that mirror the dataclasses if needed
class DocumentChunk(Base):
    """
    SQLAlchemy model for document chunks.
    """
    __tablename__ = "document_chunks"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    chunk_id = Column(String, unique=True, nullable=False, index=True)
    text_content = Column(Text, nullable=False)
    source_metadata = Column(Text, nullable=False)  # JSON serialized metadata
    embedding = Column(Text, nullable=True)  # JSON serialized embedding vector
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), onupdate=func.now())


def create_tables(engine):
    """
    Create all tables in the database.

    Args:
        engine: SQLAlchemy engine instance
    """
    Base.metadata.create_all(bind=engine)


if __name__ == "__main__":
    from sqlalchemy import create_engine
    from src.utils.config import get_settings

    settings = get_settings()

    # Create an engine and generate tables
    engine = create_engine(settings.neon_database_url)
    create_tables(engine)

    print("Tables created successfully!")