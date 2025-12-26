/**
 * AuthContext - Global authentication state management.
 *
 * Provides authentication state and methods to the entire application.
 * Automatically restores session on page load by calling getMe().
 *
 * SSR-SAFE: Uses isClient guard to prevent hydration mismatches.
 * Browser-only APIs (localStorage) are only accessed after client mount.
 */

import React, {
  createContext,
  useContext,
  useState,
  useEffect,
  useCallback,
  useMemo,
  ReactNode,
} from 'react';
import type {
  User,
  UserProfile,
  AuthState,
  SignupRequest,
  SigninRequest,
  ProfileUpdateRequest,
  ExperienceLevel,
  HardwareAccess,
} from './types';
import {
  signup as apiSignup,
  signin as apiSignin,
  signout as apiSignout,
  getMe,
  updateProfile as apiUpdateProfile,
  getStoredToken,
} from './api';

/**
 * Auth context value interface.
 */
interface AuthContextValue extends AuthState {
  login: (email: string, password: string, rememberMe: boolean) => Promise<void>;
  signup: (
    email: string,
    password: string,
    profile: {
      software_level: ExperienceLevel;
      robotics_level: ExperienceLevel;
      hardware_access: HardwareAccess;
    }
  ) => Promise<void>;
  logout: () => Promise<void>;
  updateProfile: (updates: ProfileUpdateRequest) => Promise<void>;
  clearError: () => void;
}

/**
 * Default auth state for SSR.
 * isLoading is FALSE during SSR to prevent blocking render.
 * It becomes TRUE only on client when we start checking auth.
 */
const defaultAuthState: AuthState = {
  user: null,
  profile: null,
  isAuthenticated: false,
  isLoading: false, // FALSE for SSR - prevents render blocking
  error: null,
};

/**
 * Auth context.
 */
const AuthContext = createContext<AuthContextValue | undefined>(undefined);

/**
 * Props for AuthProvider.
 */
interface AuthProviderProps {
  children: ReactNode;
}

/**
 * AuthProvider component.
 * Wraps the application and provides auth state to all children.
 *
 * SSR-SAFE: Uses isClient state to delay browser API access.
 */
export function AuthProvider({ children }: AuthProviderProps): JSX.Element {
  // SSR-safe: Track if we're on the client
  const [isClient, setIsClient] = useState(false);

  const [user, setUser] = useState<User | null>(null);
  const [profile, setProfile] = useState<UserProfile | null>(null);
  // Start as false for SSR, will become true when client starts auth check
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const isAuthenticated = user !== null;

  /**
   * Mark as client-side mounted.
   * This triggers auth restoration.
   */
  useEffect(() => {
    setIsClient(true);
  }, []);

  /**
   * Restore session on client mount.
   * Only runs after isClient is true (browser environment).
   */
  useEffect(() => {
    // Guard: Only run on client
    if (!isClient) {
      return;
    }

    let isMounted = true;

    async function restoreSession() {
      // Set loading true now that we're checking auth
      setIsLoading(true);

      const token = getStoredToken();
      if (!token) {
        if (isMounted) {
          setIsLoading(false);
        }
        return;
      }

      try {
        const data = await getMe();
        if (isMounted && data) {
          setUser(data.user);
          setProfile(data.profile);
        }
      } catch (err) {
        // Session invalid, clear state
        if (isMounted) {
          setUser(null);
          setProfile(null);
        }
      } finally {
        if (isMounted) {
          setIsLoading(false);
        }
      }
    }

    restoreSession();

    return () => {
      isMounted = false;
    };
  }, [isClient]);

  /**
   * Login handler.
   */
  const login = useCallback(
    async (email: string, password: string, rememberMe: boolean) => {
      setIsLoading(true);
      setError(null);

      try {
        const request: SigninRequest = {
          email,
          password,
          remember_me: rememberMe,
        };
        const response = await apiSignin(request);
        setUser(response.user);
        setProfile(response.profile);
      } catch (err) {
        const message = err instanceof Error ? err.message : 'Login failed';
        setError(message);
        throw err;
      } finally {
        setIsLoading(false);
      }
    },
    []
  );

  /**
   * Signup handler.
   * Note: Does NOT auto-login. User must sign in after signup.
   */
  const signup = useCallback(
    async (
      email: string,
      password: string,
      profileData: {
        software_level: ExperienceLevel;
        robotics_level: ExperienceLevel;
        hardware_access: HardwareAccess;
      }
    ) => {
      setIsLoading(true);
      setError(null);

      try {
        const request: SignupRequest = {
          email,
          password,
          profile: profileData,
        };
        await apiSignup(request);
        // Don't set user/profile - user must sign in manually
      } catch (err) {
        const message = err instanceof Error ? err.message : 'Signup failed';
        setError(message);
        throw err;
      } finally {
        setIsLoading(false);
      }
    },
    []
  );

  /**
   * Logout handler.
   */
  const logout = useCallback(async () => {
    setIsLoading(true);
    try {
      await apiSignout();
    } finally {
      setUser(null);
      setProfile(null);
      setError(null);
      setIsLoading(false);
    }
  }, []);

  /**
   * Update profile handler.
   */
  const updateProfile = useCallback(async (updates: ProfileUpdateRequest) => {
    setError(null);

    try {
      const updatedProfile = await apiUpdateProfile(updates);
      setProfile(updatedProfile);
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Profile update failed';
      setError(message);
      throw err;
    }
  }, []);

  /**
   * Clear error.
   */
  const clearError = useCallback(() => {
    setError(null);
  }, []);

  /**
   * Memoized context value.
   */
  const value = useMemo<AuthContextValue>(
    () => ({
      user,
      profile,
      isAuthenticated,
      isLoading,
      error,
      login,
      signup,
      logout,
      updateProfile,
      clearError,
    }),
    [user, profile, isAuthenticated, isLoading, error, login, signup, logout, updateProfile, clearError]
  );

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

/**
 * useAuth hook.
 * Access auth state and methods from any component.
 */
export function useAuth(): AuthContextValue {
  const context = useContext(AuthContext);
  if (context === undefined) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}

export default AuthContext;
