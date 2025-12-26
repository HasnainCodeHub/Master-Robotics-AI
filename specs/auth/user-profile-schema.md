# User Profile Schema Specification

**Feature**: User Profile for Personalization
**Phase**: 5 - Authentication & Personalization (Bonus)
**Owner**: identity-personalization agent
**Created**: 2024-12-24
**Updated**: 2024-12-25

---

## Overview

Defines the minimal user profile fields collected during signup to enable presentation-level personalization. No sensitive personal data is collected.

### Design Principles

1. **Data Minimization**: Only collect fields that directly improve the learning experience
2. **No PII**: No name, address, phone, demographics, or personally identifying data
3. **Learning-Relevant Only**: Fields inform presentation style, not content substance
4. **RAG Isolation**: Profile data NEVER influences retrieval or answer generation

---

## Critical Boundary: Profile vs RAG

| System | Uses Profile? | Behavior |
|--------|--------------|----------|
| Frontend UI | Yes | Adjusts hint visibility, tooltip behavior, example highlighting |
| Textbook Renderer | Yes | CSS-based visibility toggles for beginner/advanced sections |
| RAG Retrieval | NO | Same chunks retrieved for all users |
| Answer Generation | NO | Same prompts and answers for all users |
| Refusal Logic | NO | Same thresholds and messages for all users |

**This boundary is non-negotiable.** See `specs/auth/personalization-rules.md` for enforcement details.

## Profile Fields

### Software Background (Required)

**Field Name**: `software_level`
**Type**: Enum
**Options**:
- `beginner` - New to programming, learning Python basics
- `intermediate` - Comfortable with Python, some project experience
- `advanced` - Professional developer, experienced with multiple languages

**Purpose**: Adjusts code explanation depth and terminology complexity.

**Personalization Impact**:
- Beginner: More code comments, step-by-step explanations
- Intermediate: Standard explanations, assumes basic patterns known
- Advanced: Concise code, optional advanced patterns available

---

### Robotics Background (Required)

**Field Name**: `robotics_level`
**Type**: Enum
**Options**:
- `beginner` - New to robotics, no prior experience
- `intermediate` - Basic robotics concepts, some ROS exposure
- `advanced` - Hands-on experience with robots, ROS proficient

**Purpose**: Adjusts robotics concept explanations and terminology.

**Personalization Impact**:
- Beginner: Detailed robotics concept introductions, analogies
- Intermediate: Standard explanations, assumes basic terminology
- Advanced: Concise explanations, focus on advanced patterns

---

### Hardware Access (Optional)

**Field Name**: `hardware_access`
**Type**: Enum
**Options**:
- `simulation_only` - No physical hardware, using simulators only
- `jetson_device` - Has access to NVIDIA Jetson device
- `physical_robot` - Has access to a physical robot

**Default**: `simulation_only`

**Purpose**: Tailors examples to available hardware context.

**Personalization Impact**:
- Simulation Only: Focus on Gazebo/Isaac Sim examples
- Jetson Device: Include Jetson-specific deployment notes
- Physical Robot: Include hardware integration considerations

---

### Personalization Toggle (Required)

**Field Name**: `personalization_enabled`
**Type**: Boolean
**Default**: `true`

**Purpose**: Allows users to disable all personalization and receive default (intermediate) content presentation.

**When Enabled (`true`)**:
- UI adapts hints, tooltips, and examples based on user levels
- BeginnerHint/AdvancedDeepDive components respect user profile
- Recommended badges shown for relevant examples

**When Disabled (`false`)**:
- All users see identical intermediate-level presentation
- Same experience as anonymous (non-logged-in) users
- No personalized badges or adaptive components

**User Control**: Users can toggle this setting at any time via Profile Settings.

---

## User Personas (Examples)

### Persona 1: Software Developer New to Robotics

| Field | Value | Rationale |
|-------|-------|-----------|
| software_level | advanced | Years of Python/C++ experience |
| robotics_level | beginner | First robotics project |
| hardware_access | simulation_only | Learning before hardware purchase |

**Presentation**: Concise code explanations, expanded robotics terminology tooltips, Gazebo examples highlighted.

---

### Persona 2: Hobbyist with Jetson Nano

| Field | Value | Rationale |
|-------|-------|-----------|
| software_level | intermediate | Comfortable with Python basics |
| robotics_level | intermediate | Some Arduino/ROS experience |
| hardware_access | jetson_device | Owns a Jetson Nano |

**Presentation**: Standard explanations, Jetson deployment notes highlighted, balanced hints.

---

### Persona 3: Graduate Student Researcher

| Field | Value | Rationale |
|-------|-------|-----------|
| software_level | advanced | Publishes research code |
| robotics_level | advanced | PhD in robotics |
| hardware_access | physical_robot | Lab with research robots |

**Presentation**: Minimal hints (collapsed), hardware integration examples highlighted, concise tooltips.

---

### Persona 4: Complete Beginner

| Field | Value | Rationale |
|-------|-------|-----------|
| software_level | beginner | Learning first programming language |
| robotics_level | beginner | No prior exposure |
| hardware_access | simulation_only | Starting with simulators |

**Presentation**: Expanded hints by default, detailed code comments visible, robotics analogies shown, all tooltips auto-display.

---

## Database Schema

### UserProfile Table

```sql
CREATE TYPE experience_level AS ENUM ('beginner', 'intermediate', 'advanced');
CREATE TYPE hardware_access_type AS ENUM ('simulation_only', 'jetson_device', 'physical_robot');

CREATE TABLE user_profiles (
    user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
    software_level experience_level NOT NULL,
    robotics_level experience_level NOT NULL,
    hardware_access hardware_access_type NOT NULL DEFAULT 'simulation_only',
    personalization_enabled BOOLEAN NOT NULL DEFAULT true,
    created_at TIMESTAMP NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP NOT NULL DEFAULT NOW()
);

-- Index for quick lookups
CREATE INDEX idx_profile_user ON user_profiles(user_id);
```

### SQLAlchemy Model

```python
from enum import Enum as PyEnum
from sqlalchemy import Column, ForeignKey, Boolean, DateTime, Enum
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.orm import relationship
from app.db.base import Base

class ExperienceLevel(str, PyEnum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class HardwareAccess(str, PyEnum):
    SIMULATION_ONLY = "simulation_only"
    JETSON_DEVICE = "jetson_device"
    PHYSICAL_ROBOT = "physical_robot"

class UserProfile(Base):
    __tablename__ = "user_profiles"

    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="CASCADE"), primary_key=True)
    software_level = Column(Enum(ExperienceLevel), nullable=False)
    robotics_level = Column(Enum(ExperienceLevel), nullable=False)
    hardware_access = Column(Enum(HardwareAccess), nullable=False, default=HardwareAccess.SIMULATION_ONLY)
    personalization_enabled = Column(Boolean, nullable=False, default=True)
    created_at = Column(DateTime, nullable=False, server_default=func.now())
    updated_at = Column(DateTime, nullable=False, server_default=func.now(), onupdate=func.now())

    # Relationship
    user = relationship("User", back_populates="profile")
```

## Pydantic Schemas

```python
from enum import Enum
from pydantic import BaseModel
from uuid import UUID

class ExperienceLevel(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class HardwareAccess(str, Enum):
    SIMULATION_ONLY = "simulation_only"
    JETSON_DEVICE = "jetson_device"
    PHYSICAL_ROBOT = "physical_robot"

class UserProfileCreate(BaseModel):
    software_level: ExperienceLevel
    robotics_level: ExperienceLevel
    hardware_access: HardwareAccess = HardwareAccess.SIMULATION_ONLY

class UserProfileResponse(BaseModel):
    user_id: UUID
    software_level: ExperienceLevel
    robotics_level: ExperienceLevel
    hardware_access: HardwareAccess
    personalization_enabled: bool

    class Config:
        from_attributes = True

class UserProfileUpdate(BaseModel):
    software_level: ExperienceLevel | None = None
    robotics_level: ExperienceLevel | None = None
    hardware_access: HardwareAccess | None = None
    personalization_enabled: bool | None = None
```

## Validation Rules

### Required Fields at Signup
1. `software_level` must be provided
2. `robotics_level` must be provided
3. `hardware_access` defaults to `simulation_only` if not provided

### Update Rules
1. Any field can be updated independently
2. `personalization_enabled` can be toggled at any time
3. Changes take effect immediately

### Default Behavior
When `personalization_enabled = false`:
- Show default (intermediate) content for all users
- Do not apply any level-based adjustments
- User sees same content as anonymous users

## Privacy Considerations

### Data Minimization
- Only collect fields necessary for personalization
- No name, location, or demographic data
- No tracking of learning progress (not required)

### Data Retention
- Profile deleted when user account is deleted (CASCADE)
- No separate retention policy needed

### Data Access
- Profile only accessible to authenticated user
- Profile not shared with third parties
- Profile not used for marketing

## API Endpoints for Profile

### GET /api/auth/profile
Get current user's profile.

### PATCH /api/auth/profile
Update profile fields.

```json
Request:
{
  "software_level": "advanced",
  "personalization_enabled": true
}

Response (200):
{
  "user_id": "uuid",
  "software_level": "advanced",
  "robotics_level": "beginner",
  "hardware_access": "simulation_only",
  "personalization_enabled": true
}
```

## Constraints

1. **Minimal Data**: Only collect what's needed for personalization
2. **No PII**: No personal identifying information beyond email
3. **User Control**: User can disable personalization at any time
4. **Defaults Safe**: Anonymous users get intermediate content

---

## Explicitly Excluded Fields

The following data is **NEVER** collected:

| Category | Examples | Reason |
|----------|----------|--------|
| Personal Identity | Name, photo, bio | Not needed for learning personalization |
| Contact Info | Phone, address | Privacy risk, not learning-relevant |
| Demographics | Age, gender, location | Could introduce bias, not needed |
| Employment | Company, job title | Could bias content inappropriately |
| Education | University, degree | Self-reported levels are sufficient |
| Learning History | Pages visited, time spent | Tracking deferred, privacy concern |
| Assessment Scores | Quiz results | Not implemented in MVP |

---

## Deterministic Personalization Guarantee

**Same Profile = Same Presentation**

Given identical profile values:
- Same hints visible/hidden
- Same tooltip behavior
- Same example highlighting
- Same CSS classes applied

There is **no probabilistic or ML-based adaptation**. Personalization is fully deterministic and reproducible.

---

## Safe Defaults Behavior

### When Profile is Missing or Incomplete

| Scenario | Behavior |
|----------|----------|
| Anonymous user (no account) | Intermediate presentation for all fields |
| Logged in, profile missing | Create profile with intermediate defaults |
| Profile field is null | Use intermediate for that field |
| personalization_enabled = false | Intermediate presentation for all fields |

**Principle**: Missing data should never block learning. Default to the middle ground.

---

## Acceptance Criteria

- [ ] All profile fields use enum types (no free-text)
- [ ] `software_level` and `robotics_level` required at signup
- [ ] `hardware_access` defaults to `simulation_only`
- [ ] `personalization_enabled` defaults to `true`
- [ ] Profile is deleted when user account is deleted (CASCADE)
- [ ] Profile data never passed to RAG service
- [ ] Same profile values produce identical presentation (deterministic)
- [ ] Anonymous users receive intermediate-level presentation
