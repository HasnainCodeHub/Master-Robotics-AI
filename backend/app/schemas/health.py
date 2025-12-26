"""Health check response schema."""

from datetime import datetime
from typing import Optional

from pydantic import BaseModel


class HealthResponse(BaseModel):
    """Response for GET /health."""

    status: str  # "healthy" or "unhealthy"
    qdrant_connected: bool
    postgres_connected: bool
    version: str = "1.0.0"
    timestamp: datetime
    error: Optional[str] = None
