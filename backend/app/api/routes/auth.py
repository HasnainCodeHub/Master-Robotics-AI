"""
Authentication routes for signup, signin, and profile management.

SECURITY NOTES:
- All endpoints use proper exception handling
- No stack traces are exposed to clients
- All errors return JSON responses
"""

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession

from app.core.auth import get_current_user_required
from app.core.logging import get_logger
from app.db.postgres import get_db
from app.models.user import User
from app.schemas.user import (
    SignupRequest,
    SigninRequest,
    AuthResponse,
    SignoutResponse,
    MeResponse,
    UserResponse,
    UserProfileResponse,
    UserProfileUpdate,
)
from app.services.user import create_user, authenticate_user, update_user_profile, get_user_by_id

logger = get_logger(__name__)

router = APIRouter()


@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
async def signup(
    request: SignupRequest,
    db: AsyncSession = Depends(get_db),
) -> AuthResponse:
    """
    Create a new user account with profile.

    - **email**: User's email address (must be unique)
    - **password**: Password (min 8 chars, must have letters and numbers)
    - **profile**: User background information for personalization

    Returns:
        AuthResponse with user, profile, and access token.

    Raises:
        400: Email already registered or invalid data.
        500: Internal server error.
    """
    try:
        response = await create_user(db, request)
        logger.info("signup_success", email=request.email)
        return response
    except ValueError as e:
        # Known validation errors (e.g., email exists)
        logger.info("signup_failed", email=request.email, reason=str(e))
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e),
        )
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        # Unexpected errors - log but don't expose details
        logger.error("signup_error", email=request.email, error=str(e))
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create account. Please try again.",
        )


@router.post("/signin", response_model=AuthResponse)
async def signin(
    request: SigninRequest,
    db: AsyncSession = Depends(get_db),
) -> AuthResponse:
    """
    Sign in with email and password.

    - **email**: User's email address
    - **password**: User's password
    - **remember_me**: If true, token expires in 7 days instead of 1 hour

    Returns:
        AuthResponse with user, profile, and access token.

    Raises:
        401: Invalid email or password.
        500: Internal server error.
    """
    try:
        response = await authenticate_user(db, request)

        if response is None:
            logger.info("signin_failed", email=request.email, reason="invalid_credentials")
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password",
            )

        logger.info("signin_success", email=request.email)
        return response
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        # Unexpected errors - log but don't expose details
        logger.error("signin_error", email=request.email, error=str(e))
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Authentication failed. Please try again.",
        )


@router.post("/signout", response_model=SignoutResponse)
async def signout(
    user: User = Depends(get_current_user_required),
) -> SignoutResponse:
    """
    Sign out the current user.

    Note: This is a stateless JWT implementation, so signout just returns success.
    The client should clear the token from storage.

    Returns:
        SignoutResponse with success message.
    """
    try:
        logger.info("user_signed_out", user_id=str(user.id))
        return SignoutResponse()
    except Exception as e:
        logger.error("signout_error", user_id=str(user.id) if user else "unknown", error=str(e))
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Signout failed. Please try again.",
        )


@router.get("/me", response_model=MeResponse)
async def get_me(
    user: User = Depends(get_current_user_required),
    db: AsyncSession = Depends(get_db),
) -> MeResponse:
    """
    Get the current authenticated user's information and profile.

    Returns:
        MeResponse with user and profile data.

    Raises:
        401: Not authenticated or invalid token.
        500: Internal server error.
    """
    try:
        # Ensure profile is loaded
        user_with_profile = await get_user_by_id(db, user.id)

        if user_with_profile is None or user_with_profile.profile is None:
            logger.error("get_me_no_profile", user_id=str(user.id))
            raise HTTPException(
                status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
                detail="User profile not found",
            )

        return MeResponse(
            user=UserResponse(id=user_with_profile.id, email=user_with_profile.email),
            profile=UserProfileResponse(
                software_level=user_with_profile.profile.software_level.value,
                robotics_level=user_with_profile.profile.robotics_level.value,
                hardware_access=user_with_profile.profile.hardware_access.value,
                personalization_enabled=user_with_profile.profile.personalization_enabled,
            ),
        )
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error("get_me_error", user_id=str(user.id) if user else "unknown", error=str(e))
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve user information",
        )


@router.patch("/profile", response_model=UserProfileResponse)
async def update_profile(
    updates: UserProfileUpdate,
    user: User = Depends(get_current_user_required),
    db: AsyncSession = Depends(get_db),
) -> UserProfileResponse:
    """
    Update the current user's profile.

    All fields are optional - only provided fields will be updated.

    Returns:
        Updated UserProfileResponse.

    Raises:
        401: Not authenticated or invalid token.
        500: Internal server error.
    """
    try:
        result = await update_user_profile(db, user, updates)
        logger.info("profile_updated", user_id=str(user.id))
        return result
    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error("profile_update_error", user_id=str(user.id) if user else "unknown", error=str(e))
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to update profile",
        )
