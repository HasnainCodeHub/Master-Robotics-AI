"""User and authentication schemas."""

from enum import Enum
from typing import Optional
from uuid import UUID

from pydantic import BaseModel, EmailStr, Field, field_validator


class ExperienceLevel(str, Enum):
    """Experience level for software and robotics background."""

    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class HardwareAccess(str, Enum):
    """Type of hardware access available to the user."""

    SIMULATION_ONLY = "simulation_only"
    JETSON_DEVICE = "jetson_device"
    PHYSICAL_ROBOT = "physical_robot"


# Profile Schemas

class UserProfileCreate(BaseModel):
    """Profile data collected at signup."""

    software_level: ExperienceLevel
    robotics_level: ExperienceLevel
    hardware_access: HardwareAccess = HardwareAccess.SIMULATION_ONLY


class UserProfileResponse(BaseModel):
    """Profile data returned to client."""

    software_level: ExperienceLevel
    robotics_level: ExperienceLevel
    hardware_access: HardwareAccess
    personalization_enabled: bool

    class Config:
        from_attributes = True


class UserProfileUpdate(BaseModel):
    """Profile update request."""

    software_level: Optional[ExperienceLevel] = None
    robotics_level: Optional[ExperienceLevel] = None
    hardware_access: Optional[HardwareAccess] = None
    personalization_enabled: Optional[bool] = None


# Auth Schemas

class SignupRequest(BaseModel):
    """Signup request with credentials and profile."""

    email: EmailStr
    password: str = Field(..., min_length=8)
    profile: UserProfileCreate

    @field_validator("password")
    @classmethod
    def validate_password(cls, v: str) -> str:
        """Validate password has letters and numbers."""
        if not any(c.isalpha() for c in v):
            raise ValueError("Password must contain at least one letter")
        if not any(c.isdigit() for c in v):
            raise ValueError("Password must contain at least one number")
        return v


class SigninRequest(BaseModel):
    """Signin request."""

    email: EmailStr
    password: str
    remember_me: bool = False


class UserResponse(BaseModel):
    """User data returned to client."""

    id: UUID
    email: str

    class Config:
        from_attributes = True


class AuthResponse(BaseModel):
    """Authentication response with token."""

    user: UserResponse
    profile: UserProfileResponse
    access_token: str
    token_type: str = "bearer"
    expires_in: int  # seconds


class SignoutRequest(BaseModel):
    """Signout request."""

    pass  # Token from header


class SignoutResponse(BaseModel):
    """Signout response."""

    message: str = "Signed out successfully"


class MeResponse(BaseModel):
    """Current user response."""

    user: UserResponse
    profile: UserProfileResponse
