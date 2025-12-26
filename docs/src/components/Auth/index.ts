/**
 * Auth Components Index
 *
 * Exports all authentication-related components and hooks.
 */

// Context and hooks
export { AuthProvider, useAuth } from './AuthContext';

// Components
export { AuthModal } from './AuthModal';
export { UserMenu } from './UserMenu';
export { ProfileSettings } from './ProfileSettings';
export { ConfirmDialog } from './ConfirmDialog';
export { Toast } from './Toast';

// Access Guards
export { BookAccessGuard, getAndClearRedirectUrl } from './guards';

// API utilities
export { getStoredToken } from './api';

// Types (re-export for convenience)
export type {
  User,
  UserProfile,
  AuthState,
  ExperienceLevel,
  HardwareAccess,
  SignupRequest,
  SigninRequest,
  ProfileUpdateRequest,
  AuthResponse,
} from './types';
