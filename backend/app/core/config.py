"""Application configuration using Pydantic Settings."""

from functools import lru_cache
from typing import List, Optional

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore",  # Ignore extra env vars like deprecated OPENAI_API_KEY
    )

    # Environment
    env: str = "development"
    log_level: str = "INFO"

    # Qdrant Cloud
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection: str = "textbook_chunks"

    # Neon Postgres
    database_url: str

    # Google Gemini (for embeddings, generation, and translation)
    google_api_key: str

    # RAG Configuration (Gemini text-embedding-004 uses 768 dimensions)
    embedding_model: str = "text-embedding-004"
    embedding_dimension: int = 768
    retrieval_threshold: float = 0.7
    retrieval_limit: int = 5

    # CORS - comma-separated list of allowed origins
    # PRODUCTION: MUST set CORS_ORIGINS to your frontend URL
    # Example: https://hasnaincodehub.github.io
    cors_origins: str = "http://localhost:3000,http://localhost:3001,http://127.0.0.1:3000,http://127.0.0.1:3001"

    # Authentication (JWT)
    # CRITICAL: MUST set JWT_SECRET in production (min 32 chars)
    # Generate with: python -c "import secrets; print(secrets.token_hex(32))"
    jwt_secret: str = ""  # Will use fallback in auth.py if not set (dev only)
    jwt_algorithm: str = "HS256"
    jwt_access_token_expire_minutes: int = 30

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    @property
    def async_database_url(self) -> str:
        """Convert postgres:// to postgresql+asyncpg:// for SQLAlchemy async."""
        url = self.database_url
        if url.startswith("postgres://"):
            url = url.replace("postgres://", "postgresql+asyncpg://", 1)
        elif url.startswith("postgresql://"):
            url = url.replace("postgresql://", "postgresql+asyncpg://", 1)
        return url


@lru_cache
def get_settings() -> Settings:
    """Get cached settings instance."""
    return Settings()
