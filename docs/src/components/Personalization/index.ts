/**
 * Personalization Components - Barrel Exports
 *
 * PRESENTATION ONLY personalization for the Physical AI Textbook.
 * These components control UI display based on user profile,
 * without affecting RAG outputs or content meaning.
 *
 * @see specs/auth/personalization-rules.md for full specification
 */

// Context and hook
export {
  PersonalizationProvider,
  usePersonalization,
  default as PersonalizationContext,
} from './PersonalizationContext';

// Display components
export { BeginnerHint, default as BeginnerHintComponent } from './BeginnerHint';
export { AdvancedDeepDive, default as AdvancedDeepDiveComponent } from './AdvancedDeepDive';
export { TermTooltip, default as TermTooltipComponent } from './TermTooltip';
export { HardwareRecommendation, default as HardwareRecommendationComponent } from './HardwareRecommendation';

// Control components
export { PersonalizationToggle, default as PersonalizationToggleComponent } from './PersonalizationToggle';

// Types
export type { HardwareType } from './HardwareRecommendation';

// CSS (import in Root or global styles)
import './personalization.css';
