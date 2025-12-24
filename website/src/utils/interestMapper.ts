/**
 * Interest Mapper Utility (T008)
 * Maps user interests to book chapters and provides filtering logic
 */

import type { ChapterMetadata, Interest } from '../types/personalization';
import { AVAILABLE_INTERESTS } from '../config/interests.config';

/**
 * Filter chapters by user interests
 * Returns chapters that match ANY of the user's selected interests
 *
 * @param chapters - All available chapters
 * @param userInterests - User's selected interest IDs
 * @returns Filtered chapters matching user interests
 */
export function filterChaptersByInterests(
  chapters: ChapterMetadata[],
  userInterests: string[]
): ChapterMetadata[] {
  // If no interests selected, return empty array
  if (!userInterests || userInterests.length === 0) {
    return [];
  }

  // Filter chapters where ANY chapter interest matches ANY user interest
  return chapters.filter((chapter) => {
    if (!chapter.interests || chapter.interests.length === 0) {
      return false;
    }

    return chapter.interests.some((chapterInterest) =>
      userInterests.includes(chapterInterest)
    );
  });
}

/**
 * Get all available interests
 * Returns the list of interests defined in the configuration
 */
export function getAvailableInterests(): Interest[] {
  return AVAILABLE_INTERESTS;
}

/**
 * Validate interest IDs
 * Checks if all provided interest IDs exist in the available interests
 *
 * @param interestIds - Interest IDs to validate
 * @returns true if all IDs are valid, false otherwise
 */
export function validateInterestIds(interestIds: string[]): boolean {
  if (!interestIds || interestIds.length === 0) {
    return true; // Empty array is valid
  }

  const validIds = AVAILABLE_INTERESTS.map((interest) => interest.id);

  return interestIds.every((id) => validIds.includes(id));
}

/**
 * Sort chapters by book order
 * Preserves the original chapter sequence from the book
 *
 * @param chapters - Chapters to sort
 * @returns Sorted chapters
 */
export function sortChaptersByBookOrder(
  chapters: ChapterMetadata[]
): ChapterMetadata[] {
  return [...chapters].sort((a, b) => {
    // Sort by module first, then by order
    if (a.module && b.module && a.module !== b.module) {
      return a.module.localeCompare(b.module);
    }

    if (a.order !== undefined && b.order !== undefined) {
      return a.order - b.order;
    }

    // Fallback to URL comparison
    return a.url.localeCompare(b.url);
  });
}

/**
 * Get interest labels from IDs
 * Converts interest IDs to human-readable labels
 *
 * @param interestIds - Interest IDs to convert
 * @returns Array of interest labels
 */
export function getInterestLabels(interestIds: string[]): string[] {
  return interestIds
    .map((id) => AVAILABLE_INTERESTS.find((interest) => interest.id === id))
    .filter((interest): interest is Interest => interest !== undefined)
    .map((interest) => interest.label);
}

/**
 * Backend-to-Frontend Interest ID Mapping
 * Maps numeric backend interest IDs to string frontend interest IDs
 */
const BACKEND_TO_FRONTEND_INTEREST_MAP: Record<number, string> = {
  1: 'physical-ai',
  2: 'ros2',
  3: 'kinematics',
  4: 'dynamics-control',
  5: 'sensors',
  6: 'humanoid-design',
  7: 'simulation',
  8: 'machine-learning',
};

/**
 * Map backend interest IDs to frontend interest strings
 *
 * @param backendIds - Numeric backend interest IDs
 * @returns Array of frontend interest string IDs
 */
export function mapBackendToFrontendInterests(backendIds: number[]): string[] {
  return backendIds
    .map((id) => BACKEND_TO_FRONTEND_INTEREST_MAP[id])
    .filter((id): id is string => id !== undefined);
}

/**
 * Frontend-to-Backend Interest ID Mapping
 * Maps string frontend interest IDs to numeric backend interest IDs
 */
const FRONTEND_TO_BACKEND_INTEREST_MAP: Record<string, number> = {
  'physical-ai': 1,
  'ros2': 2,
  'kinematics': 3,
  'dynamics-control': 4,
  'sensors': 5,
  'humanoid-design': 6,
  'simulation': 7,
  'machine-learning': 8,
};

/**
 * Map frontend interest strings to backend interest IDs
 *
 * @param frontendIds - String frontend interest IDs
 * @returns Array of numeric backend interest IDs
 */
export function mapFrontendToBackendInterests(frontendIds: string[]): number[] {
  return frontendIds
    .map((id) => FRONTEND_TO_BACKEND_INTEREST_MAP[id])
    .filter((id): id is number => id !== undefined);
}
