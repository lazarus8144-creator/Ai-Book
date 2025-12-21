/**
 * TypeScript types for Personalization API
 *
 * Generated from: personalization-api.yaml
 * Feature: 004-personalization
 * Date: 2025-12-21
 *
 * Usage:
 *   import {
 *     PersonalizationRequest,
 *     PersonalizedContentResponse,
 *     PersonalizationPreferences
 *   } from '@/services/personalizationApi';
 */

// ============================================================================
// Enums
// ============================================================================

export enum VerbosityLevel {
  CONCISE = 'concise',
  MODERATE = 'moderate',
  DETAILED = 'detailed',
}

export enum ExamplePreference {
  THEORETICAL = 'theoretical',
  PRACTICAL = 'practical',
  MIXED = 'mixed',
}

export enum CodeCommentDepth {
  MINIMAL = 'minimal',
  STANDARD = 'standard',
  EXTENSIVE = 'extensive',
}

export enum ViewType {
  ORIGINAL = 'original',
  PERSONALIZED = 'personalized',
}

// ============================================================================
// Request Types
// ============================================================================

export interface PersonalizationRequest {
  chapter_id: string;
  force_regenerate?: boolean;
}

export interface PersonalizationPreferencesUpdate {
  verbosity_level?: VerbosityLevel;
  example_preference?: ExamplePreference;
  code_comment_depth?: CodeCommentDepth;
}

// ============================================================================
// Response Types
// ============================================================================

export interface PersonalizedContentResponse {
  id: string; // UUID
  user_id: string; // UUID
  chapter_id: string;
  personalized_markdown: string;
  original_content_hash: string;
  created_at: string; // ISO 8601 datetime
  last_accessed_at: string; // ISO 8601 datetime
  is_invalid: boolean;
  from_cache: boolean;
  generation_time_ms: number | null;
}

export interface ChapterContentResponse {
  chapter_id: string;
  content: string; // Markdown
  view_type: ViewType;
  last_updated: string; // ISO 8601 datetime
}

export interface PersonalizationPreferencesResponse {
  user_id: string; // UUID
  verbosity_level: VerbosityLevel;
  example_preference: ExamplePreference;
  code_comment_depth: CodeCommentDepth;
  created_at: string; // ISO 8601 datetime
  updated_at: string; // ISO 8601 datetime
}

export interface RecommendationItem {
  chapter_id: string;
  title: string;
  reason: string;
}

export interface ChapterRecommendationResponse {
  current_chapter_id: string;
  recommendations: RecommendationItem[]; // Max 3 items
  generated_at: string; // ISO 8601 datetime
  expires_at: string; // ISO 8601 datetime
}

export interface ErrorResponse {
  error: string;
  detail?: string;
  status_code: number;
}

// ============================================================================
// API Client Types
// ============================================================================

export interface PersonalizationApiConfig {
  baseUrl: string;
  getAuthToken: () => Promise<string | null>;
}

export interface PersonalizationApiClient {
  /**
   * Generate personalized chapter content
   * POST /api/v1/personalization/personalize
   */
  personalizeChapter(
    request: PersonalizationRequest
  ): Promise<PersonalizedContentResponse>;

  /**
   * Get chapter content (original or personalized view)
   * GET /api/v1/personalization/chapter/{chapter_id}?view={view}
   */
  getChapterContent(
    chapterId: string,
    view?: ViewType
  ): Promise<ChapterContentResponse>;

  /**
   * Get user personalization preferences
   * GET /api/v1/personalization/preferences
   */
  getPreferences(): Promise<PersonalizationPreferencesResponse>;

  /**
   * Update personalization preferences
   * PUT /api/v1/personalization/preferences
   */
  updatePreferences(
    update: PersonalizationPreferencesUpdate
  ): Promise<PersonalizationPreferencesResponse>;

  /**
   * Get personalized chapter recommendations
   * GET /api/v1/personalization/recommendations/{chapter_id}
   */
  getRecommendations(
    chapterId: string
  ): Promise<ChapterRecommendationResponse>;
}

// ============================================================================
// Frontend State Types
// ============================================================================

export interface PersonalizationState {
  /** Current view mode for chapter */
  viewType: ViewType;

  /** Whether personalization is in progress */
  isPersonalizing: boolean;

  /** Personalized content cache (keyed by chapter_id) */
  personalizedContent: Record<string, PersonalizedContentResponse>;

  /** User preferences */
  preferences: PersonalizationPreferencesResponse | null;

  /** Current recommendations */
  recommendations: ChapterRecommendationResponse | null;

  /** Error message if personalization fails */
  error: string | null;

  /** Progress message during personalization (e.g., "Personalizing section 2 of 4...") */
  progressMessage: string | null;
}

export interface PersonalizationActions {
  /** Trigger personalization for current chapter */
  personalizeChapter: (chapterId: string, forceRegenerate?: boolean) => Promise<void>;

  /** Toggle between original and personalized view */
  toggleView: () => void;

  /** Update user preferences (invalidates cache) */
  updatePreferences: (update: PersonalizationPreferencesUpdate) => Promise<void>;

  /** Load recommendations for current chapter */
  loadRecommendations: (chapterId: string) => Promise<void>;

  /** Clear error state */
  clearError: () => void;
}

// ============================================================================
// React Hook Types
// ============================================================================

export interface UsePersonalizationResult {
  state: PersonalizationState;
  actions: PersonalizationActions;
}

export interface UsePreferencesResult {
  preferences: PersonalizationPreferencesResponse | null;
  isLoading: boolean;
  error: string | null;
  updatePreferences: (update: PersonalizationPreferencesUpdate) => Promise<void>;
  resetPreferences: () => Promise<void>;
}

// ============================================================================
// Component Props
// ============================================================================

export interface PersonalizeButtonProps {
  chapterId: string;
  onPersonalizationStart?: () => void;
  onPersonalizationComplete?: (content: PersonalizedContentResponse) => void;
  onError?: (error: string) => void;
  className?: string;
}

export interface ContentToggleProps {
  viewType: ViewType;
  onToggle: (newView: ViewType) => void;
  disabled?: boolean;
  className?: string;
}

export interface PersonalizationSettingsProps {
  preferences: PersonalizationPreferencesResponse | null;
  onUpdate: (update: PersonalizationPreferencesUpdate) => Promise<void>;
  onClose: () => void;
}

export interface ChapterRecommendationsProps {
  chapterId: string;
  recommendations: RecommendationItem[];
  onChapterClick: (chapterId: string) => void;
  className?: string;
}

// ============================================================================
// Utility Types
// ============================================================================

export type SkillLevel = 'beginner' | 'intermediate' | 'advanced';

export interface UserLearningProfile {
  skill_level: SkillLevel;
  learning_goals: string;
  prior_experience: string;
}

export interface ChapterMetadata {
  chapter_id: string;
  title: string;
  module: number;
  difficulty: number; // 1-3 (beginner, intermediate, advanced)
  keywords: string[];
  prerequisites: string[]; // chapter_ids
  word_count: number;
}

// ============================================================================
// Constants
// ============================================================================

export const DEFAULT_PREFERENCES: PersonalizationPreferencesUpdate = {
  verbosity_level: VerbosityLevel.MODERATE,
  example_preference: ExamplePreference.MIXED,
  code_comment_depth: CodeCommentDepth.STANDARD,
};

export const PERSONALIZATION_CACHE_DURATION_MS = 24 * 60 * 60 * 1000; // 24 hours
export const RECOMMENDATION_CACHE_DURATION_MS = 7 * 24 * 60 * 60 * 1000; // 7 days
export const MAX_PERSONALIZATION_TIME_MS = 30 * 1000; // 30 seconds timeout
