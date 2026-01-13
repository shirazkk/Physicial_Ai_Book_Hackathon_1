from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import QueuePool
from contextlib import contextmanager
import logging

from src.utils.config import get_settings
from src.utils.logging import get_logger

logger = get_logger(__name__)


class DatabaseConnection:
    """
    Manages database connection and session management for Neon Postgres.
    """

    def __init__(self):
        settings = get_settings()

        # Create the database URL
        self.database_url = settings.neon_database_url

        # Configure engine with connection pooling
        self.engine = create_engine(
            self.database_url,
            poolclass=QueuePool,
            pool_size=settings.neon_pool_min,
            max_overflow=settings.neon_pool_max - settings.neon_pool_min,
            pool_pre_ping=True,  # Verify connections before use
            pool_recycle=3600,    # Recycle connections after 1 hour
            echo=False            # Set to True for SQL logging
        )

        # Create session factory
        self.SessionLocal = sessionmaker(
            autocommit=False,
            autoflush=False,
            bind=self.engine
        )

        self.logger = get_logger(__name__)

    def get_db_session(self):
        """
        Get a database session using context manager.
        """
        db = self.SessionLocal()
        try:
            yield db
        finally:
            db.close()

    @contextmanager
    def get_session(self):
        """
        Context manager for database sessions.
        """
        session = self.SessionLocal()
        try:
            yield session
            session.commit()
        except Exception as e:
            session.rollback()
            self.logger.error(f"Database transaction error: {str(e)}")
            raise
        finally:
            session.close()

    def test_connection(self) -> bool:
        """
        Test the database connection.

        Returns:
            True if connection is successful, False otherwise
        """
        try:
            with self.get_session() as session:
                # Execute a simple query to test connection
                session.execute("SELECT 1")
                return True
        except Exception as e:
            self.logger.error(f"Database connection test failed: {str(e)}")
            return False

    def get_engine(self):
        """
        Get the database engine instance.

        Returns:
            SQLAlchemy engine
        """
        return self.engine

    def dispose_engine(self):
        """
        Dispose of the database engine (useful for cleanup).
        """
        self.engine.dispose()


# Global database connection instance
_db_connection: DatabaseConnection = None


def get_db_connection() -> DatabaseConnection:
    """
    Get the global database connection instance.

    Returns:
        DatabaseConnection instance
    """
    global _db_connection
    if _db_connection is None:
        _db_connection = DatabaseConnection()
    return _db_connection


def get_db_session():
    """
    Dependency function for FastAPI to get database session.
    """
    db_conn = get_db_connection()
    return db_conn.get_db_session()


def initialize_database():
    """
    Initialize the database by creating tables.
    """
    from .models import create_tables
    db_conn = get_db_connection()
    create_tables(db_conn.get_engine())
    logger.info("Database initialized successfully")


if __name__ == "__main__":
    # Test database connection
    db_conn = get_db_connection()

    if db_conn.test_connection():
        print("Database connection successful!")
    else:
        print("Database connection failed!")