/**
 * User utilities for managing user identity
 * Provides temporary user ID management until better-auth is fully integrated
 */

const USER_ID_KEY = 'user_id';
const USER_INTERESTS_KEY = 'user_has_interests';

/**
 * Get the current user ID from localStorage
 * Returns null if user is not signed in
 */
export function getCurrentUserId(): string | null {
  if (typeof window === 'undefined') return null;

  try {
    const userId = localStorage.getItem(USER_ID_KEY);
    return userId;
  } catch (error) {
    console.warn('[User] Failed to get user ID from localStorage:', error);
    return null;
  }
}

/**
 * Set the current user ID in localStorage
 */
export function setCurrentUserId(userId: string): void {
  if (typeof window === 'undefined') return;

  try {
    localStorage.setItem(USER_ID_KEY, userId);
  } catch (error) {
    console.warn('[User] Failed to set user ID in localStorage:', error);
  }
}

/**
 * Clear the current user ID (sign out)
 */
export function clearCurrentUserId(): void {
  if (typeof window === 'undefined') return;

  try {
    localStorage.removeItem(USER_ID_KEY);
    localStorage.removeItem(USER_INTERESTS_KEY);
  } catch (error) {
    console.warn('[User] Failed to clear user ID from localStorage:', error);
  }
}

/**
 * Check if user has personalization enabled (has interests)
 * Returns cached value from localStorage for performance
 */
export function getUserHasInterests(): boolean {
  if (typeof window === 'undefined') return false;

  try {
    const hasInterests = localStorage.getItem(USER_INTERESTS_KEY);
    return hasInterests === 'true';
  } catch (error) {
    console.warn('[User] Failed to get user interests flag:', error);
    return false;
  }
}

/**
 * Set whether user has interests (for caching)
 */
export function setUserHasInterests(hasInterests: boolean): void {
  if (typeof window === 'undefined') return;

  try {
    localStorage.setItem(USER_INTERESTS_KEY, hasInterests ? 'true' : 'false');
  } catch (error) {
    console.warn('[User] Failed to set user interests flag:', error);
  }
}

/**
 * Check if user is signed in
 */
export function isUserSignedIn(): boolean {
  return getCurrentUserId() !== null;
}
