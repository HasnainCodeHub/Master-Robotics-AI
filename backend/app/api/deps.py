"""API dependencies and exception handlers."""

from typing import Any, Dict

from fastapi import HTTPException, Request
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse
from pydantic import ValidationError

from app.core.logging import get_logger

logger = get_logger(__name__)


class APIError(HTTPException):
    """Custom API error with structured response."""

    def __init__(
        self,
        status_code: int,
        error: str,
        message: str,
        details: Dict[str, Any] = None,
    ):
        self.error = error
        self.message = message
        self.details = details or {}
        super().__init__(status_code=status_code, detail=message)


async def api_error_handler(request: Request, exc: APIError) -> JSONResponse:
    """Handle APIError exceptions."""
    logger.warning(
        "api_error",
        error=exc.error,
        message=exc.message,
        details=exc.details,
        path=request.url.path,
    )
    return JSONResponse(
        status_code=exc.status_code,
        content={
            "error": exc.error,
            "message": exc.message,
            "details": exc.details,
        },
    )


async def request_validation_error_handler(
    request: Request, exc: RequestValidationError
) -> JSONResponse:
    """
    Handle FastAPI request validation errors (422).

    This catches validation errors from request body, query params, path params.
    Returns detailed field-level errors to help debugging.
    """
    # Extract readable error messages
    errors = []
    for error in exc.errors():
        loc = " -> ".join(str(l) for l in error.get("loc", []))
        msg = error.get("msg", "Invalid value")
        error_type = error.get("type", "unknown")
        errors.append({
            "field": loc,
            "message": msg,
            "type": error_type,
        })

    logger.warning(
        "request_validation_error",
        errors=errors,
        path=request.url.path,
        method=request.method,
    )

    return JSONResponse(
        status_code=422,
        content={
            "error": "validation_error",
            "message": "Request validation failed",
            "details": {
                "errors": errors,
            },
        },
    )


async def validation_error_handler(request: Request, exc: ValidationError) -> JSONResponse:
    """Handle Pydantic validation errors (from manual validation)."""
    logger.warning(
        "pydantic_validation_error",
        errors=exc.errors(),
        path=request.url.path,
    )
    return JSONResponse(
        status_code=400,
        content={
            "error": "validation_error",
            "message": "Invalid request body",
            "details": {"errors": exc.errors()},
        },
    )


async def generic_error_handler(request: Request, exc: Exception) -> JSONResponse:
    """Handle unexpected exceptions."""
    logger.error(
        "internal_error",
        error=str(exc),
        error_type=type(exc).__name__,
        path=request.url.path,
    )
    return JSONResponse(
        status_code=500,
        content={
            "error": "internal_error",
            "message": "An unexpected error occurred",
            "details": {},
        },
    )
