/**
 * Type definitions for authentication and user profile.
 */

export type ExperienceLevel = 'beginner' | 'intermediate' | 'advanced';
export type HardwareAccess = 'simulation_only' | 'jetson_device' | 'physical_robot';

export interface UserProfile {
  user_id: string;
  software_level: ExperienceLevel;
  robotics_level: ExperienceLevel;
  hardware_access: HardwareAccess;
  personalization_enabled: boolean;
  created_at: string;
  updated_at: string;
}

export interface User {
  id: string;
  email: string;
  created_at: string;
}

export interface AuthState {
  user: User | null;
  profile: UserProfile | null;
  isAuthenticated: boolean;
  isLoading: boolean;
  error: string | null;
}

export interface SignupRequest {
  email: string;
  password: string;
  profile: {
    software_level: ExperienceLevel;
    robotics_level: ExperienceLevel;
    hardware_access: HardwareAccess;
  };
}

export interface SigninRequest {
  email: string;
  password: string;
  remember_me: boolean;
}

export interface AuthResponse {
  user: User;
  profile: UserProfile;
  access_token: string;
  token_type: string;
}

export interface ProfileUpdateRequest {
  software_level?: ExperienceLevel;
  robotics_level?: ExperienceLevel;
  hardware_access?: HardwareAccess;
  personalization_enabled?: boolean;
}
