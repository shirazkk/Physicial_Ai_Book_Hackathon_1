from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.middleware.trustedhost import TrustedHostMiddleware
from contextlib import asynccontextmanager
import logging

from src.utils.config import get_settings
from src.utils.logging import setup_logging
from src.routers.chat_router import router as chat_router


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan manager for startup and shutdown events.
    """
    # Startup
    settings = get_settings()
    logger = setup_logging(
        log_level="INFO" if not settings.fastapi_debug else "DEBUG",
        json_format=True
    )

    logger.info("Application starting up")

    # Initialize services here if needed

    yield

    # Shutdown
    logger.info("Application shutting down")


def create_app() -> FastAPI:
    """
    Create and configure the FastAPI application.

    Returns:
        Configured FastAPI application instance
    """
    settings = get_settings()

    app = FastAPI(
        title="RAG Chatbot API",
        description="API for the Integrated RAG Chatbot for the Textbook",
        version="1.0.0",
        lifespan=lifespan,
        debug=settings.fastapi_debug
    )

    # Add CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.cors_origins,
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
        # Additional security headers can be added here
    )

    # Add trusted host middleware
    app.add_middleware(
        TrustedHostMiddleware,
        allowed_hosts=["*"]  # In production, specify your actual hosts
    )

    # Include routers
    app.include_router(chat_router, tags=["chat"])

    @app.get("/health")
    async def health_check():
        """
        Health check endpoint to verify the application is running.
        """
        return {"status": "healthy", "service": "rag-chatbot-api"}

    @app.get("/")
    async def root():
        """
        Root endpoint for basic information about the API.
        """
        return {
            "message": "RAG Chatbot API for Textbook Q&A",
            "version": "1.0.0",
            "endpoints": [
                "/health",
                "/docs",
                "/redoc",
                "/api/v1/chat",
                "/api/v1/index"
            ]
        }

    return app


# Create the main application instance
app = create_app()


if __name__ == "__main__":
    import uvicorn
    settings = get_settings()
    uvicorn.run(
        "src.main:app",
        host=settings.fastapi_host,
        port=settings.fastapi_port,
        reload=settings.fastapi_debug,
        log_level="info" if not settings.fastapi_debug else "debug"
    )