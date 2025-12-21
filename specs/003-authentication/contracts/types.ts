/**
 * TypeScript types for Authentication API
 * Generated from OpenAPI spec: auth-api.yaml
 * Date: 2025-12-21
 */

export type SkillLevel = 'beginner' | 'intermediate' | 'advanced';

export interface RegisterRequest {
  email: string;
  password: string;
  name: string;
  skill_level: SkillLevel;
  learning_goals?: string;
  prior_experience?: string;
}

export interface LoginRequest {
  email: string;
  password: string;
  remember_me?: boolean;
}

export interface ProfileUpdateRequest {
  name?: string;
  skill_level?: SkillLevel;
  learning_goals?: string;
  prior_experience?: string;
}

export interface ForgotPasswordRequest {
  email: string;
}

export interface ResetPasswordRequest {
  token: string;
  new_password: string;
}

export interface LearningProfile {
  id: string;
  user_id: string;
  name: string;
  skill_level: SkillLevel;
  learning_goals: string | null;
  prior_experience: string | null;
  created_at: string;
  updated_at: string;
}

export interface UserProfile {
  id: string;
  email: string;
  is_active: boolean;
  created_at: string;
  last_login_at: string | null;
  learning_profile: LearningProfile;
}

export interface AuthResponse {
  access_token: string;
  token_type: 'bearer';
  user: UserProfile;
}

export interface ApiError {
  detail: string;
  error_code?: string;
}

export interface ApiResponse<T> {
  data?: T;
  error?: ApiError;
}
