from typing import Optional
from pydantic_settings import BaseSettings
from pydantic import Field


class Settings(BaseSettings):
    """
    Application settings loaded from environment variables.
    """
    # Gemini API settings
    gemini_api_key: str = Field(..., validation_alias='GEMINI_API_KEY')
    gemini_chat_model: str = Field(default='gemini-flash-latest', validation_alias='GEMINI_CHAT_MODEL')
    gemini_embedding_model: str = Field(default='embedding-001', validation_alias='GEMINI_EMBEDDING_MODEL')

    # Qdrant settings
    qdrant_api_key: str = Field(..., validation_alias='QDRANT_API_KEY')
    qdrant_url: str = Field(..., validation_alias='QDRANT_URL')
    qdrant_collection_name: str = Field(default='book_chunks', validation_alias='QDRANT_COLLECTION_NAME')
    qdrant_vector_name: str = Field(default='bookvector', validation_alias='QDRANT_VECTOR_NAME')
    qdrant_vector_size: int = Field(default=512, validation_alias='QDRANT_VECTOR_SIZE')
    qdrant_distance_metric: str = Field(default='Cosine', validation_alias='QDRANT_DISTANCE_METRIC')

    # Neon Postgres settings
    neon_database_url: str = Field(..., validation_alias='NEON_DATABASE_URL')
    neon_pool_min: int = Field(default=1, validation_alias='NEON_POOL_MIN')
    neon_pool_max: int = Field(default=10, validation_alias='NEON_POOL_MAX')

    # API settings
    fastapi_host: str = Field(default='0.0.0.0', validation_alias='FASTAPI_HOST')
    fastapi_port: int = Field(default=8000, validation_alias='FASTAPI_PORT')
    fastapi_debug: bool = Field(default=False, validation_alias='FASTAPI_DEBUG')

    # CORS settings
    cors_origins: list[str] = Field(
        default=['http://localhost:3000', 'http://localhost:3001', 'https://shirazkk.github.io/Physicial_Ai_Book_Hackathon_1'],
        validation_alias='CORS_ORIGINS'
    )

    # Frontend settings
    chatbot_api_url: str = Field(default='http://localhost:8000', validation_alias='CHATBOT_API_URL')
    chatbot_widget_enabled: bool = Field(default=True, validation_alias='CHATBOT_WIDGET_ENABLED')

    # Security settings
    jwt_secret_key: str = Field(..., validation_alias='JWT_SECRET_KEY')
    jwt_expiration_hours: int = Field(default=24, validation_alias='JWT_EXPIRATION_HOURS')
    session_secret: str = Field(..., validation_alias='SESSION_SECRET')
    rate_limit_requests: int = Field(default=100, validation_alias='RATE_LIMIT_REQUESTS')
    rate_limit_window: int = Field(default=3600, validation_alias='RATE_LIMIT_WINDOW')

    # Application settings
    environment: str = Field(default='development', validation_alias='ENVIRONMENT')
    log_level: str = Field(default='INFO', validation_alias='LOG_LEVEL')
    allowed_origins: str = Field(default='http://localhost:3000, https://shirazkk.github.io/Physicial_Ai_Book_Hackathon_1, http://localhost:8080', validation_alias='ALLOWED_ORIGINS')

    # RAG parameters
    chunk_size: int = Field(default=600, validation_alias='CHUNK_SIZE')
    chunk_overlap: int = Field(default=100, validation_alias='CHUNK_OVERLAP')
    top_k_results: int = Field(default=8, validation_alias='TOP_K_RESULTS')
    relevance_threshold: float = Field(default=0.7, validation_alias='RELEVANCE_THRESHOLD')
    max_context_length: int = Field(default=4000, validation_alias='MAX_CONTEXT_LENGTH')

    # Model aliases
    embedding_model: str = Field(default='models/text-embedding-004', validation_alias='EMBEDDING_MODEL')
    chat_model: str = Field(default='gemini-flash-latest', validation_alias='CHAT_MODEL')

    class Config:
        env_file = ".env"
        case_sensitive = True


# Global settings instance
_settings: Optional[Settings] = None


def get_settings() -> Settings:
    """
    Get application settings.
    This function can be used as a FastAPI dependency.
    """
    global _settings
    if _settings is None:
        _settings = Settings()
    return _settings