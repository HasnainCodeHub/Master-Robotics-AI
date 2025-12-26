"""
Authentication utilities for FastAPI backend.

This module provides JWT-based authentication with:
- Secure password hashing via passlib + bcrypt
- Standardized JWT tokens (HS256)
- Proper error handling and logging

SECURITY NOTES:
- All passwords are hashed using bcrypt (12 rounds)
- JWT tokens include: sub (user_id), email, exp, iat
- Token validation raises HTTPException on failure
"""

from datetime import datetime, timedelta, timezone
from typing import Optional
from uuid import UUID

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from jose import JWTError, jwt
from passlib.context import CryptContext
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.core.config import get_settings
from app.core.logging import get_logger
from app.db.postgres import get_db
from app.models.user import User

logger = get_logger(__name__)

# Security scheme for Bearer token
security = HTTPBearer(auto_error=False)

# Password hashing context using bcrypt
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

# JWT Configuration
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60  # 1 hour default
ACCESS_TOKEN_EXPIRE_MINUTES_REMEMBER = 60 * 24 * 7  # 7 days with "remember me"

# Minimum JWT secret length
MIN_SECRET_LENGTH = 32


def get_jwt_secret() -> str:
    """
    Get JWT secret from settings with validation.

    Returns:
        The JWT secret string.

    Raises:
        RuntimeError: If no valid JWT secret is configured.
    """
    settings = get_settings()

    # Use dedicated JWT_SECRET if set
    if settings.jwt_secret and len(settings.jwt_secret) >= MIN_SECRET_LENGTH:
        return settings.jwt_secret

    # Fallback for development ONLY
    if settings.env == "development":
        # Use a deterministic but unique fallback for development
        fallback = "dev-secret-key-do-not-use-in-production-" + settings.google_api_key[:16] if settings.google_api_key else "dev-fallback-secret-key-unsafe-12345678"
        logger.warning(
            "jwt_secret_fallback",
            message="Using fallback JWT secret. Set JWT_SECRET env var in production!"
        )
        return fallback[:64]  # Ensure reasonable length

    # Production requires proper secret
    logger.error("jwt_secret_missing", message="JWT_SECRET must be set in production")
    raise RuntimeError("JWT_SECRET environment variable is required in production")


def hash_password(password: str) -> str:
    """
    Hash a password using bcrypt via passlib.

    Args:
        password: Plain text password.

    Returns:
        Bcrypt hash string.
    """
    return pwd_context.hash(password)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """
    Verify a password against its hash.

    Args:
        plain_password: Plain text password to verify.
        hashed_password: Bcrypt hash to verify against.

    Returns:
        True if password matches, False otherwise.
    """
    try:
        return pwd_context.verify(plain_password, hashed_password)
    except Exception as e:
        logger.warning("password_verify_failed", error=str(e))
        return False


def create_access_token(
    user_id: UUID,
    email: str,
    remember_me: bool = False,
) -> tuple[str, int]:
    """
    Create a JWT access token.

    Args:
        user_id: The user's UUID.
        email: The user's email address.
        remember_me: If True, token expires in 7 days instead of 1 hour.

    Returns:
        Tuple of (token, expires_in_seconds).
    """
    expire_minutes = (
        ACCESS_TOKEN_EXPIRE_MINUTES_REMEMBER
        if remember_me
        else ACCESS_TOKEN_EXPIRE_MINUTES
    )
    expires_delta = timedelta(minutes=expire_minutes)
    expire = datetime.now(timezone.utc) + expires_delta
    issued_at = datetime.now(timezone.utc)

    # Standard JWT payload
    to_encode = {
        "sub": str(user_id),      # Subject (user ID)
        "email": email,            # User email for convenience
        "exp": expire,             # Expiration time
        "iat": issued_at,          # Issued at time
    }

    try:
        token = jwt.encode(to_encode, get_jwt_secret(), algorithm=ALGORITHM)
        expires_in = int(expires_delta.total_seconds())
        return token, expires_in
    except Exception as e:
        logger.error("jwt_encode_failed", error=str(e))
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create authentication token",
        )


def decode_access_token(token: str) -> Optional[dict]:
    """
    Decode and validate a JWT access token.

    Args:
        token: The JWT token string.

    Returns:
        Dict with user_id (UUID) and email if valid, None otherwise.
    """
    try:
        payload = jwt.decode(token, get_jwt_secret(), algorithms=[ALGORITHM])

        user_id_str: str = payload.get("sub")
        email: str = payload.get("email")

        if user_id_str is None:
            logger.debug("jwt_missing_sub", message="Token missing 'sub' claim")
            return None

        return {
            "user_id": UUID(user_id_str),
            "email": email,
        }
    except JWTError as e:
        logger.debug("jwt_decode_failed", error=str(e))
        return None
    except ValueError as e:
        logger.debug("jwt_invalid_uuid", error=str(e))
        return None
    except Exception as e:
        logger.warning("jwt_unexpected_error", error=str(e))
        return None


async def get_current_user(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
    db: AsyncSession = Depends(get_db),
) -> Optional[User]:
    """
    Get the current authenticated user from the JWT token.

    This is a soft dependency - returns None if not authenticated.
    Use get_current_user_required for endpoints that require auth.

    Args:
        credentials: Bearer token from Authorization header.
        db: Database session.

    Returns:
        User if authenticated, None otherwise.
    """
    if credentials is None:
        return None

    token = credentials.credentials
    token_data = decode_access_token(token)

    if token_data is None:
        return None

    try:
        # Fetch user from database
        result = await db.execute(
            select(User).where(User.id == token_data["user_id"])
        )
        user = result.scalar_one_or_none()
        return user
    except Exception as e:
        logger.error("get_current_user_failed", error=str(e))
        return None


async def get_current_user_required(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
    db: AsyncSession = Depends(get_db),
) -> User:
    """
    Get the current authenticated user, raising 401 if not authenticated.

    Use this for endpoints that require authentication.

    Args:
        credentials: Bearer token from Authorization header.
        db: Database session.

    Returns:
        The authenticated User.

    Raises:
        HTTPException: 401 if not authenticated or token invalid.
    """
    if credentials is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Not authenticated",
            headers={"WWW-Authenticate": "Bearer"},
        )

    token = credentials.credentials
    token_data = decode_access_token(token)

    if token_data is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid or expired token",
            headers={"WWW-Authenticate": "Bearer"},
        )

    try:
        # Fetch user from database
        result = await db.execute(
            select(User).where(User.id == token_data["user_id"])
        )
        user = result.scalar_one_or_none()

        if user is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found",
                headers={"WWW-Authenticate": "Bearer"},
            )

        return user
    except HTTPException:
        raise
    except Exception as e:
        logger.error("get_current_user_required_failed", error=str(e))
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Authentication error",
        )
