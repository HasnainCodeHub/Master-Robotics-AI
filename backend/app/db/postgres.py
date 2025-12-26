"""Neon Postgres connection and session management."""

from typing import AsyncGenerator, Optional

from sqlalchemy import text
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine, async_sessionmaker
from sqlalchemy.orm import DeclarativeBase

from app.core.config import get_settings
from app.core.logging import get_logger

logger = get_logger(__name__)

# Global engine instance
_engine = None
_session_factory = None


class Base(DeclarativeBase):
    """SQLAlchemy declarative base for all models."""

    pass


def get_engine():
    """Get or create async SQLAlchemy engine."""
    global _engine
    if _engine is None:
        settings = get_settings()
        _engine = create_async_engine(
            settings.async_database_url,
            echo=settings.env == "development",
            pool_pre_ping=True,
            pool_size=5,
            max_overflow=10,
        )
        logger.info("postgres_engine_created")
    return _engine


def get_session_factory():
    """Get or create async session factory."""
    global _session_factory
    if _session_factory is None:
        engine = get_engine()
        _session_factory = async_sessionmaker(
            engine,
            class_=AsyncSession,
            expire_on_commit=False,
        )
        logger.info("postgres_session_factory_created")
    return _session_factory


async def get_db() -> AsyncGenerator[AsyncSession, None]:
    """Dependency for getting database sessions."""
    session_factory = get_session_factory()
    async with session_factory() as session:
        try:
            yield session
            await session.commit()
        except Exception:
            await session.rollback()
            raise
        finally:
            await session.close()


async def check_postgres_health() -> bool:
    """Check if Postgres is reachable and healthy."""
    try:
        engine = get_engine()
        async with engine.connect() as conn:
            await conn.execute(text("SELECT 1"))
        logger.debug("postgres_health_check_passed")
        return True
    except Exception as e:
        logger.error("postgres_health_check_failed", error=str(e))
        return False


async def create_tables() -> None:
    """Create all tables (for development/testing)."""
    engine = get_engine()
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)
    logger.info("postgres_tables_created")


async def drop_tables() -> None:
    """Drop all tables (for development/testing)."""
    engine = get_engine()
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.drop_all)
    logger.info("postgres_tables_dropped")
