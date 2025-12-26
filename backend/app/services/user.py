"""User service for authentication and profile management."""

from typing import Optional
from uuid import UUID

from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.orm import selectinload

from app.core.auth import hash_password, verify_password, create_access_token
from app.core.logging import get_logger
from app.models.user import User, UserProfile, ExperienceLevel, HardwareAccess
from app.schemas.user import (
    SignupRequest,
    SigninRequest,
    AuthResponse,
    UserResponse,
    UserProfileResponse,
    UserProfileUpdate,
)

logger = get_logger(__name__)


async def get_user_by_email(db: AsyncSession, email: str) -> Optional[User]:
    """Get a user by email address."""
    result = await db.execute(
        select(User)
        .options(selectinload(User.profile))
        .where(User.email == email)
    )
    return result.scalar_one_or_none()


async def get_user_by_id(db: AsyncSession, user_id: UUID) -> Optional[User]:
    """Get a user by ID with profile loaded."""
    result = await db.execute(
        select(User)
        .options(selectinload(User.profile))
        .where(User.id == user_id)
    )
    return result.scalar_one_or_none()


async def create_user(db: AsyncSession, request: SignupRequest) -> AuthResponse:
    """
    Create a new user with profile.

    Args:
        db: Database session.
        request: Signup request with credentials and profile.

    Returns:
        AuthResponse with user, profile, and token.

    Raises:
        ValueError: If email already exists.
    """
    # Check if email exists
    existing = await get_user_by_email(db, request.email)
    if existing:
        raise ValueError("Email already registered")

    # Create user
    user = User(
        email=request.email,
        password_hash=hash_password(request.password),
    )
    db.add(user)
    await db.flush()  # Get user.id

    # Create profile
    profile = UserProfile(
        user_id=user.id,
        software_level=ExperienceLevel(request.profile.software_level.value),
        robotics_level=ExperienceLevel(request.profile.robotics_level.value),
        hardware_access=HardwareAccess(request.profile.hardware_access.value),
    )
    db.add(profile)
    await db.commit()

    # Refresh to get relationships
    await db.refresh(user)
    await db.refresh(profile)

    # Generate token (now includes email in JWT payload)
    access_token, expires_in = create_access_token(user.id, user.email)

    logger.info("user_created", user_id=str(user.id), email=user.email)

    return AuthResponse(
        user=UserResponse(id=user.id, email=user.email),
        profile=UserProfileResponse(
            software_level=profile.software_level.value,
            robotics_level=profile.robotics_level.value,
            hardware_access=profile.hardware_access.value,
            personalization_enabled=profile.personalization_enabled,
        ),
        access_token=access_token,
        expires_in=expires_in,
    )


async def authenticate_user(
    db: AsyncSession,
    request: SigninRequest,
) -> Optional[AuthResponse]:
    """
    Authenticate a user and return token.

    Args:
        db: Database session.
        request: Signin request with credentials.

    Returns:
        AuthResponse if successful, None if invalid credentials.
    """
    user = await get_user_by_email(db, request.email)

    if user is None:
        logger.info("signin_failed", reason="user_not_found", email=request.email)
        return None

    if not verify_password(request.password, user.password_hash):
        logger.info("signin_failed", reason="invalid_password", email=request.email)
        return None

    # Generate token (now includes email in JWT payload)
    access_token, expires_in = create_access_token(
        user.id,
        user.email,
        remember_me=request.remember_me,
    )

    logger.info("user_signed_in", user_id=str(user.id), email=user.email)

    return AuthResponse(
        user=UserResponse(id=user.id, email=user.email),
        profile=UserProfileResponse(
            software_level=user.profile.software_level.value,
            robotics_level=user.profile.robotics_level.value,
            hardware_access=user.profile.hardware_access.value,
            personalization_enabled=user.profile.personalization_enabled,
        ),
        access_token=access_token,
        expires_in=expires_in,
    )


async def update_user_profile(
    db: AsyncSession,
    user: User,
    updates: UserProfileUpdate,
) -> UserProfileResponse:
    """
    Update user profile.

    Args:
        db: Database session.
        user: The user to update.
        updates: Profile updates.

    Returns:
        Updated profile.
    """
    # Ensure profile is loaded
    if user.profile is None:
        user = await get_user_by_id(db, user.id)

    profile = user.profile

    # Apply updates
    if updates.software_level is not None:
        profile.software_level = ExperienceLevel(updates.software_level.value)
    if updates.robotics_level is not None:
        profile.robotics_level = ExperienceLevel(updates.robotics_level.value)
    if updates.hardware_access is not None:
        profile.hardware_access = HardwareAccess(updates.hardware_access.value)
    if updates.personalization_enabled is not None:
        profile.personalization_enabled = updates.personalization_enabled

    await db.commit()
    await db.refresh(profile)

    logger.info("profile_updated", user_id=str(user.id))

    return UserProfileResponse(
        software_level=profile.software_level.value,
        robotics_level=profile.robotics_level.value,
        hardware_access=profile.hardware_access.value,
        personalization_enabled=profile.personalization_enabled,
    )
