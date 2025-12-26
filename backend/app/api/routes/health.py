"""Health check endpoint."""

from datetime import datetime, timezone

from fastapi import APIRouter

from app.db.qdrant import check_qdrant_health
from app.db.postgres import check_postgres_health
from app.schemas.health import HealthResponse
from app.core.logging import get_logger

logger = get_logger(__name__)

router = APIRouter()


@router.get("/health", response_model=HealthResponse)
async def health_check() -> HealthResponse:
    """
    Health check endpoint for Railway deployment and monitoring.

    Returns connection status for both Qdrant and Postgres.
    """
    qdrant_ok = await check_qdrant_health()
    postgres_ok = await check_postgres_health()

    status = "healthy" if (qdrant_ok and postgres_ok) else "unhealthy"
    error = None

    if not qdrant_ok and not postgres_ok:
        error = "Both Qdrant and Postgres connections failed"
    elif not qdrant_ok:
        error = "Qdrant connection failed"
    elif not postgres_ok:
        error = "Postgres connection failed"

    logger.info(
        "health_check",
        status=status,
        qdrant=qdrant_ok,
        postgres=postgres_ok,
    )

    return HealthResponse(
        status=status,
        qdrant_connected=qdrant_ok,
        postgres_connected=postgres_ok,
        timestamp=datetime.now(timezone.utc),
        error=error,
    )
