"""User and UserProfile models for authentication and personalization."""

import enum
from datetime import datetime
from uuid import uuid4

from sqlalchemy import (
    Column,
    DateTime,
    Enum,
    ForeignKey,
    String,
    Boolean,
    func,
)
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship

from app.db.postgres import Base


class ExperienceLevel(str, enum.Enum):
    """Experience level for software and robotics background."""

    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class HardwareAccess(str, enum.Enum):
    """Type of hardware access available to the user."""

    SIMULATION_ONLY = "simulation_only"
    JETSON_DEVICE = "jetson_device"
    PHYSICAL_ROBOT = "physical_robot"


class User(Base):
    """Registered user account."""

    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    created_at = Column(DateTime(timezone=True), nullable=False, server_default=func.now())
    updated_at = Column(DateTime(timezone=True), nullable=False, server_default=func.now(), onupdate=func.now())

    # Relationships
    profile = relationship("UserProfile", back_populates="user", uselist=False, cascade="all, delete-orphan")
    sessions = relationship("ChatSession", back_populates="user", cascade="all, delete-orphan")

    def __repr__(self) -> str:
        return f"<User(id={self.id}, email={self.email})>"


class UserProfile(Base):
    """User profile for personalization settings."""

    __tablename__ = "user_profiles"

    user_id = Column(
        UUID(as_uuid=True),
        ForeignKey("users.id", ondelete="CASCADE"),
        primary_key=True,
    )
    software_level = Column(Enum(ExperienceLevel), nullable=False)
    robotics_level = Column(Enum(ExperienceLevel), nullable=False)
    hardware_access = Column(
        Enum(HardwareAccess),
        nullable=False,
        default=HardwareAccess.SIMULATION_ONLY,
    )
    personalization_enabled = Column(Boolean, nullable=False, default=True)
    created_at = Column(DateTime(timezone=True), nullable=False, server_default=func.now())
    updated_at = Column(DateTime(timezone=True), nullable=False, server_default=func.now(), onupdate=func.now())

    # Relationships
    user = relationship("User", back_populates="profile")

    def __repr__(self) -> str:
        return f"<UserProfile(user_id={self.user_id}, software={self.software_level}, robotics={self.robotics_level})>"
