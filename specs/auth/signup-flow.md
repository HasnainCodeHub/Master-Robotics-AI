# Authentication Flow Specification

**Feature**: User Authentication for Physical AI Textbook
**Phase**: 5 - Authentication & Personalization
**Owner**: identity-personalization agent
**Version**: 2.0.0
**Created**: 2024-12-24
**Updated**: 2024-12-25

---

## 1. Overview

This specification defines the complete authentication flow for the Physical AI Textbook application. Authentication uses **Better Auth** as the ONLY authentication provider with JWT-based token management.

### 1.1 Scope

**In Scope**:
- Email/password signup with profile data collection
- Email/password signin with "remember me" option
- JWT token-based session management
- Stateless logout with client-side cleanup
- Profile data for personalization (not identity)

**Out of Scope**:
- Social login (OAuth/OIDC - Google, GitHub, etc.)
- Password reset flow (deferred to future enhancement)
- Email verification (deferred to future enhancement)
- Two-factor authentication
- Account recovery

### 1.2 Constraints

1. **Better Auth Only**: No custom authentication implementations beyond the JWT wrapper
2. **No Social Login**: Simple email/password authentication only
3. **Stateless JWT**: Server does not maintain session state; tokens are self-validating
4. **Privacy-First Profiling**: Collect only what improves learning

---

## 2. Signup Flow

### 2.1 User Journey

```
[User] → Click "Sign Up" → [Modal Opens] → Enter Credentials → Enter Profile → Submit → [Token Stored] → Redirect
```

### 2.2 Step-by-Step Flow

#### Step 1: Initiate Signup
1. User clicks "Sign Up" button in navigation bar
2. Auth modal opens in signup mode
3. Form displays with two sections: credentials and profile

#### Step 2: Collect Credentials

| Field | Type | Validation | Required |
|-------|------|------------|----------|
| Email | `EmailStr` | RFC 5322 compliant, max 255 chars | Yes |
| Password | `string` | Min 8 chars, at least 1 letter + 1 number | Yes |
| Confirm Password | `string` | Must match password field | Yes (frontend only) |

**Password Validation Rules** (enforced in `backend/app/schemas/user.py`):
```python
@field_validator("password")
def validate_password(cls, v: str) -> str:
    if not any(c.isalpha() for c in v):
        raise ValueError("Password must contain at least one letter")
    if not any(c.isdigit() for c in v):
        raise ValueError("Password must contain at least one number")
    return v
```

#### Step 3: Collect Profile Information

Profile data enables personalization of learning content depth and examples.

| Field | Type | Options | Default | Required |
|-------|------|---------|---------|----------|
| Software Level | `ExperienceLevel` | beginner, intermediate, advanced | None (must select) | Yes |
| Robotics Level | `ExperienceLevel` | beginner, intermediate, advanced | None (must select) | Yes |
| Hardware Access | `HardwareAccess` | simulation_only, jetson_device, physical_robot | simulation_only | No |

**Profile Data Purpose** (transparent to user):
- **Software Level**: Adjusts code example complexity
- **Robotics Level**: Adjusts robotics concept explanations
- **Hardware Access**: Tailors lab exercises to available equipment

#### Step 4: Submit Signup

**Frontend Actions**:
1. Validate all fields client-side
2. Construct request payload
3. POST to `/api/auth/signup`

**Backend Processing** (in `backend/app/services/user.py`):
1. Check email uniqueness
2. Hash password with bcrypt (cost factor 12)
3. Create `User` record in database
4. Create `UserProfile` record linked to user
5. Generate JWT access token
6. Return `AuthResponse`

#### Step 5: Success Handling

**Response** (HTTP 201):
```json
{
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "learner@example.com"
  },
  "profile": {
    "software_level": "intermediate",
    "robotics_level": "beginner",
    "hardware_access": "simulation_only",
    "personalization_enabled": true
  },
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "token_type": "bearer",
  "expires_in": 3600
}
```

**Frontend Actions**:
1. Store `access_token` in `localStorage`
2. Store token expiry timestamp
3. Update auth context with user/profile data
4. Close modal
5. Show success toast: "Account created successfully"
6. Redirect to previous page or home

### 2.3 Signup Error States

| Error Condition | HTTP Status | Error Message | User-Facing Message |
|-----------------|-------------|---------------|---------------------|
| Email already registered | 400 | "Email already registered" | "This email is already in use" |
| Invalid email format | 422 | Pydantic validation error | "Please enter a valid email" |
| Password too short | 422 | "ensure this value has at least 8 characters" | "Password must be at least 8 characters" |
| Password missing letter | 400 | "Password must contain at least one letter" | Same |
| Password missing number | 400 | "Password must contain at least one number" | Same |
| Passwords don't match | N/A | Frontend validation | "Passwords do not match" |
| Profile field missing | 422 | Pydantic validation error | "Please complete all required fields" |
| Server error | 500 | Internal error | "Unable to create account. Please try again." |

---

## 3. Signin Flow

### 3.1 User Journey

```
[User] → Click "Sign In" → [Modal Opens] → Enter Credentials → [Optional: Remember Me] → Submit → [Token Stored] → Redirect
```

### 3.2 Step-by-Step Flow

#### Step 1: Initiate Signin
1. User clicks "Sign In" button in navigation bar
2. Auth modal opens in signin mode

#### Step 2: Collect Credentials

| Field | Type | Validation | Required |
|-------|------|------------|----------|
| Email | `EmailStr` | RFC 5322 compliant | Yes |
| Password | `string` | Non-empty | Yes |
| Remember Me | `boolean` | N/A | No (default: false) |

#### Step 3: Submit Signin

**Request Payload**:
```json
{
  "email": "learner@example.com",
  "password": "securePassword123",
  "remember_me": false
}
```

**Backend Processing** (in `backend/app/services/user.py`):
1. Look up user by email
2. Verify password against stored bcrypt hash
3. If valid, load user profile
4. Generate JWT with appropriate expiry
5. Return `AuthResponse`

#### Step 4: Success Handling

Same as signup success, with token expiry varying based on `remember_me`:

| Remember Me | Token Expiry | `expires_in` (seconds) |
|-------------|--------------|------------------------|
| false | 1 hour | 3600 |
| true | 7 days | 604800 |

### 3.3 Signin Error States

| Error Condition | HTTP Status | Error Message | User-Facing Message |
|-----------------|-------------|---------------|---------------------|
| User not found | 401 | "Invalid email or password" | Same (security) |
| Wrong password | 401 | "Invalid email or password" | Same (security) |
| Empty email | 422 | Validation error | "Email is required" |
| Empty password | 422 | Validation error | "Password is required" |
| Server error | 500 | Internal error | "Unable to sign in. Please try again." |

**Security Note**: Invalid credentials always return the same message to prevent email enumeration attacks.

---

## 4. Session Handling

### 4.1 Token Architecture

**Token Type**: JWT (JSON Web Token)
**Algorithm**: HS256
**Secret Source**: Application settings (environment variable)

**Token Payload**:
```json
{
  "sub": "550e8400-e29b-41d4-a716-446655440000",
  "exp": 1703548800,
  "iat": 1703545200
}
```

| Claim | Purpose |
|-------|---------|
| `sub` | User UUID (subject) |
| `exp` | Expiry timestamp (UTC) |
| `iat` | Issued-at timestamp (UTC) |

### 4.2 Token Expiry Configuration

Defined in `backend/app/core/auth.py`:
```python
ACCESS_TOKEN_EXPIRE_MINUTES = 60  # 1 hour default
ACCESS_TOKEN_EXPIRE_MINUTES_REMEMBER = 60 * 24 * 7  # 7 days with "remember me"
```

### 4.3 Token Storage

**Storage Location**: `localStorage`

```javascript
// On successful auth
localStorage.setItem('access_token', response.access_token);
localStorage.setItem('token_expiry', Date.now() + (response.expires_in * 1000));

// On API request
const token = localStorage.getItem('access_token');
headers['Authorization'] = `Bearer ${token}`;
```

**Why localStorage over cookies**:
- Simpler CORS handling for API-first architecture
- Explicit control over token lifecycle
- No CSRF attack vector (token not auto-sent)

**Trade-off**: Vulnerable to XSS. Mitigated by:
- Strict Content Security Policy
- No inline scripts
- Input sanitization

### 4.4 Session Persistence on Page Reload

**Initialization Flow** (on app mount):
```
1. Check localStorage for access_token
2. If missing → anonymous state
3. If present, check token_expiry
4. If expired → clear and anonymous state
5. If valid → call GET /api/auth/me
6. If /me returns 200 → authenticated state
7. If /me returns 401 → clear and anonymous state
```

### 4.5 Token Refresh Strategy

**Current Implementation**: No automatic refresh (MVP scope)

**Behavior**:
- Token expiry is enforced server-side
- Expired tokens receive 401 response
- Frontend clears session and redirects to signin
- User must re-authenticate after token expiry

**Future Enhancement** (out of scope):
- Refresh token rotation
- Silent token refresh before expiry

### 4.6 API Request Authentication

All authenticated requests include the Bearer token:

```http
GET /api/auth/me HTTP/1.1
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Backend Validation** (in `backend/app/core/auth.py`):
1. Extract token from `Authorization` header
2. Decode and verify JWT signature
3. Check expiry (`exp` claim)
4. Look up user by `sub` claim
5. Return user or raise 401

---

## 5. Logout Flow

### 5.1 User Journey

```
[User] → Click "Sign Out" → [API Call] → [Local Cleanup] → [Redirect] → [Toast]
```

### 5.2 Step-by-Step Flow

#### Step 1: Initiate Logout
1. User clicks "Sign Out" from user menu dropdown

#### Step 2: Server Notification

**Request**:
```http
POST /api/auth/signout HTTP/1.1
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Response** (HTTP 200):
```json
{
  "message": "Signed out successfully"
}
```

**Note**: This is a stateless JWT implementation. The server does not maintain a session store, so "signout" is primarily for:
- Logging the signout event
- Future token blacklisting capability
- Clean API semantics

#### Step 3: Client-Side Cleanup

```javascript
// Clear stored credentials
localStorage.removeItem('access_token');
localStorage.removeItem('token_expiry');

// Clear auth context
setUser(null);
setProfile(null);
setIsAuthenticated(false);
```

#### Step 4: Post-Logout Behavior

1. Redirect to home page (`/`)
2. Show success toast: "Signed out successfully"
3. Chatbot remains functional in anonymous mode
4. Any authenticated-only features become inaccessible

---

## 6. API Endpoints Reference

### 6.1 POST /api/auth/signup

**Purpose**: Create new user account with profile

**Request**:
```json
{
  "email": "learner@example.com",
  "password": "securePassword123",
  "profile": {
    "software_level": "intermediate",
    "robotics_level": "beginner",
    "hardware_access": "simulation_only"
  }
}
```

**Success Response** (201):
```json
{
  "user": {
    "id": "uuid",
    "email": "learner@example.com"
  },
  "profile": {
    "software_level": "intermediate",
    "robotics_level": "beginner",
    "hardware_access": "simulation_only",
    "personalization_enabled": true
  },
  "access_token": "jwt...",
  "token_type": "bearer",
  "expires_in": 3600
}
```

**Error Responses**:
- 400: Email already registered
- 422: Validation error

### 6.2 POST /api/auth/signin

**Purpose**: Authenticate existing user

**Request**:
```json
{
  "email": "learner@example.com",
  "password": "securePassword123",
  "remember_me": false
}
```

**Success Response** (200): Same as signup

**Error Responses**:
- 401: Invalid email or password
- 422: Validation error

### 6.3 POST /api/auth/signout

**Purpose**: Log signout event (stateless)

**Headers**: `Authorization: Bearer <token>`

**Success Response** (200):
```json
{
  "message": "Signed out successfully"
}
```

**Error Responses**:
- 401: Not authenticated

### 6.4 GET /api/auth/me

**Purpose**: Get current user info and profile

**Headers**: `Authorization: Bearer <token>`

**Success Response** (200):
```json
{
  "user": {
    "id": "uuid",
    "email": "learner@example.com"
  },
  "profile": {
    "software_level": "intermediate",
    "robotics_level": "beginner",
    "hardware_access": "simulation_only",
    "personalization_enabled": true
  }
}
```

**Error Responses**:
- 401: Not authenticated or token expired

### 6.5 PATCH /api/auth/profile

**Purpose**: Update user profile settings

**Headers**: `Authorization: Bearer <token>`

**Request** (all fields optional):
```json
{
  "software_level": "advanced",
  "robotics_level": "intermediate",
  "hardware_access": "jetson_device",
  "personalization_enabled": false
}
```

**Success Response** (200):
```json
{
  "software_level": "advanced",
  "robotics_level": "intermediate",
  "hardware_access": "jetson_device",
  "personalization_enabled": false
}
```

---

## 7. Security Considerations

### 7.1 Password Security

| Measure | Implementation |
|---------|----------------|
| Hashing | bcrypt with cost factor 12 |
| Storage | Only hash stored, never plaintext |
| Logging | Passwords never logged |
| Transmission | HTTPS only in production |

### 7.2 Token Security

| Measure | Implementation |
|---------|----------------|
| Signing | HS256 with secure secret |
| Payload | Contains user ID only (no sensitive data) |
| Expiry | Always includes exp claim |
| Validation | Verified on every authenticated request |

### 7.3 API Security

| Measure | Implementation |
|---------|----------------|
| CORS | Strict origin checking |
| Rate Limiting | Future enhancement |
| Input Validation | Pydantic schema validation |
| Error Messages | Generic for auth failures (prevent enumeration) |

---

## 8. Frontend Implementation Notes

### 8.1 Auth Context Structure

```typescript
interface AuthContext {
  user: User | null;
  profile: UserProfile | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  login: (email: string, password: string, rememberMe?: boolean) => Promise<void>;
  signup: (data: SignupData) => Promise<void>;
  logout: () => Promise<void>;
  updateProfile: (updates: ProfileUpdate) => Promise<void>;
}
```

### 8.2 Protected Route Pattern

```typescript
function ProtectedRoute({ children }) {
  const { isAuthenticated, isLoading } = useAuth();

  if (isLoading) return <LoadingSpinner />;
  if (!isAuthenticated) return <Navigate to="/" />;

  return children;
}
```

### 8.3 Anonymous Mode Support

The chatbot must work without authentication. When not authenticated:
- Profile defaults apply (intermediate software, beginner robotics)
- Personalization is disabled
- Chat history is not persisted across sessions

---

## 9. Database Schema Reference

### 9.1 Users Table

```sql
CREATE TABLE users (
  id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
  email VARCHAR(255) UNIQUE NOT NULL,
  password_hash VARCHAR(255) NOT NULL,
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);
```

### 9.2 User Profiles Table

```sql
CREATE TABLE user_profiles (
  user_id UUID PRIMARY KEY REFERENCES users(id) ON DELETE CASCADE,
  software_level experience_level NOT NULL,
  robotics_level experience_level NOT NULL,
  hardware_access hardware_access NOT NULL DEFAULT 'simulation_only',
  personalization_enabled BOOLEAN NOT NULL DEFAULT TRUE,
  created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
  updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
);
```

---

## 10. Acceptance Criteria

### 10.1 Signup
- [ ] User can create account with email/password
- [ ] Password validation enforced (8+ chars, letter + number)
- [ ] Profile data collected during signup
- [ ] JWT token returned and stored
- [ ] Duplicate email returns appropriate error
- [ ] User redirected after successful signup

### 10.2 Signin
- [ ] User can sign in with valid credentials
- [ ] Invalid credentials return generic error
- [ ] "Remember me" extends token expiry to 7 days
- [ ] JWT token returned and stored
- [ ] Profile data included in response

### 10.3 Session
- [ ] Token persists across page reloads
- [ ] Expired token results in logout
- [ ] API requests include Authorization header
- [ ] 401 response clears session

### 10.4 Logout
- [ ] User can sign out from menu
- [ ] Token cleared from localStorage
- [ ] User redirected to home
- [ ] Success message displayed
- [ ] Chatbot continues to work anonymously

---

## 11. Related Specifications

- [User Profile Schema](/specs/auth/user-profile-schema.md)
- [Personalization Rules](/specs/auth/personalization-rules.md)
- [Auth API Contract](/specs/001-physical-ai-textbook/contracts/auth-api.yaml)
