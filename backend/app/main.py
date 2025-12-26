"""FastAPI application entry point."""

from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.exceptions import RequestValidationError
from fastapi.middleware.cors import CORSMiddleware
from pydantic import ValidationError

from app.api.deps import (
    APIError,
    api_error_handler,
    request_validation_error_handler,
    validation_error_handler,
    generic_error_handler,
)
from app.api.routes import api_router
from app.core.config import get_settings
from app.core.logging import setup_logging, get_logger
from app.db.qdrant import ensure_collection_exists
from app.db.postgres import create_tables

# Setup logging
setup_logging()
logger = get_logger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifespan handler for startup/shutdown events."""
    # Startup
    logger.info("application_starting")

    # Initialize database connections
    try:
        ensure_collection_exists()
        logger.info("qdrant_initialized")
    except Exception as e:
        logger.error("qdrant_init_failed", error=str(e))

    try:
        await create_tables()
        logger.info("postgres_initialized")
    except Exception as e:
        logger.error("postgres_init_failed", error=str(e))

    logger.info("application_started")

    yield

    # Shutdown
    logger.info("application_shutdown")


def create_app() -> FastAPI:
    """Create and configure the FastAPI application."""
    settings = get_settings()

    app = FastAPI(
        title="Physical AI Textbook RAG API",
        description="""
Retrieval-Augmented Generation API for the Physical AI & Humanoid Robotics textbook.

**RAG Safety Rules (NON-NEGOTIABLE)**:
- All answers MUST be grounded in retrieved textbook content
- No external knowledge is permitted
- Refusal is ALWAYS preferred over speculation
        """,
        version="1.0.0",
        lifespan=lifespan,
    )

    # CORS middleware
    app.add_middleware(
        CORSMiddleware,
        allow_origins=settings.cors_origins_list,
        allow_credentials=True,
        allow_methods=["GET", "POST", "PATCH", "OPTIONS"],
        allow_headers=["*"],
    )

    # Exception handlers
    app.add_exception_handler(APIError, api_error_handler)
    app.add_exception_handler(RequestValidationError, request_validation_error_handler)
    app.add_exception_handler(ValidationError, validation_error_handler)
    app.add_exception_handler(Exception, generic_error_handler)

    # Include routers
    app.include_router(api_router)

    return app


# Create app instance
app = create_app()


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "app.main:app",
        host="0.0.0.0",
        port=8001,
        reload=True,
    )
