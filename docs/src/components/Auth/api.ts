/**
 * Authentication API client.
 *
 * Phase 5.5: Real backend authentication.
 * Connects to FastAPI backend with Neon Postgres.
 *
 * BROWSER-SAFE: No process.env usage. Uses getApiBaseUrl() utility.
 */

import type {
  SignupRequest,
  SigninRequest,
  AuthResponse,
  UserProfile,
  ProfileUpdateRequest,
  User,
} from './types';
import { getApiBaseUrl } from '../../utils/config';

/**
 * Get API base URL lazily (not at module import time).
 * This prevents SSR crashes and allows runtime configuration.
 */
function getBaseUrl(): string {
  return getApiBaseUrl();
}

// Storage key for JWT token
const TOKEN_KEY = 'auth_token';

/**
 * Get stored access token.
 */
export function getStoredToken(): string | null {
  if (typeof window === 'undefined') return null;
  return localStorage.getItem(TOKEN_KEY);
}

/**
 * Store access token.
 */
function storeToken(token: string): void {
  if (typeof window === 'undefined') return;
  localStorage.setItem(TOKEN_KEY, token);
}

/**
 * Clear stored token.
 */
function clearToken(): void {
  if (typeof window === 'undefined') return;
  localStorage.removeItem(TOKEN_KEY);
}

/**
 * Make authenticated API request.
 */
async function authFetch(
  path: string,
  options: RequestInit = {},
): Promise<Response> {
  const token = getStoredToken();
  const headers: HeadersInit = {
    'Content-Type': 'application/json',
    ...options.headers,
  };

  if (token) {
    (headers as Record<string, string>)['Authorization'] = `Bearer ${token}`;
  }

  const response = await fetch(`${getBaseUrl()}${path}`, {
    ...options,
    headers,
    credentials: 'include', // Include cookies for CORS
  });

  return response;
}

/**
 * Parse error response from backend.
 * Handles both simple errors and detailed validation errors.
 */
async function parseError(response: Response): Promise<string> {
  try {
    const data = await response.json();

    // Handle validation errors with detailed field info
    if (data.error === 'validation_error' && data.details?.errors) {
      const errors = data.details.errors as Array<{field: string; message: string}>;
      if (errors.length > 0) {
        // Return first error message (most relevant)
        return errors[0].message;
      }
    }

    return data.detail || data.message || `Request failed with status ${response.status}`;
  } catch {
    return `Request failed with status ${response.status}`;
  }
}

/**
 * Sign up a new user.
 * Note: Does NOT auto-login. User must sign in after signup.
 */
export async function signup(request: SignupRequest): Promise<void> {
  const response = await authFetch('/api/auth/signup', {
    method: 'POST',
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const error = await parseError(response);
    throw new Error(error);
  }

  // Don't store token - user must sign in manually after signup
}

/**
 * Sign in an existing user.
 */
export async function signin(request: SigninRequest): Promise<AuthResponse> {
  const response = await authFetch('/api/auth/signin', {
    method: 'POST',
    body: JSON.stringify(request),
  });

  if (!response.ok) {
    const error = await parseError(response);
    throw new Error(error);
  }

  const data: AuthResponse = await response.json();
  storeToken(data.access_token);
  return data;
}

/**
 * Sign out the current user.
 */
export async function signout(): Promise<void> {
  const token = getStoredToken();
  if (token) {
    try {
      await authFetch('/api/auth/signout', {
        method: 'POST',
      });
    } catch {
      // Ignore errors, just clear token
    }
  }
  clearToken();
}

/**
 * Get current user info.
 */
export async function getMe(): Promise<{ user: User; profile: UserProfile } | null> {
  const token = getStoredToken();
  if (!token) return null;

  try {
    const response = await authFetch('/api/auth/me');

    if (!response.ok) {
      clearToken();
      return null;
    }

    return response.json();
  } catch {
    clearToken();
    return null;
  }
}

/**
 * Update user profile.
 */
export async function updateProfile(
  updates: ProfileUpdateRequest,
): Promise<UserProfile> {
  const response = await authFetch('/api/auth/profile', {
    method: 'PATCH',
    body: JSON.stringify(updates),
  });

  if (!response.ok) {
    const error = await parseError(response);
    throw new Error(error);
  }

  return response.json();
}
