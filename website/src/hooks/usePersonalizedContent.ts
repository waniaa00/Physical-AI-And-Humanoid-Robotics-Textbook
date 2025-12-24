/**
 * usePersonalizedContent Hook (T011, T037)
 * Orchestrates content filtering based on user interests
 * Automatically refreshes when interests change
 */

import { useState, useEffect, useMemo, useRef } from 'react';
import type { PersonalizedContentView } from '../types/personalization';
import { useContentMetadata } from './useContentMetadata';
import {
  filterChaptersByInterests,
  sortChaptersByBookOrder,
} from '../utils/interestMapper';

export interface UsePersonalizedContentResult {
  content: PersonalizedContentView | null;
  isLoading: boolean;
  error: Error | null;
  refetch: () => void;
  isRefreshing: boolean;
}

/**
 * Hook to get personalized content based on user interests
 *
 * @param userInterests - Array of user's selected interest IDs
 * @returns Personalized content view, loading state, error state, and refetch function
 */
export function usePersonalizedContent(
  userInterests: string[]
): UsePersonalizedContentResult {
  const { chapters, isLoading: metadataLoading, error: metadataError } = useContentMetadata();
  const [error, setError] = useState<Error | null>(null);
  const [isRefreshing, setIsRefreshing] = useState(false);

  // T037: Track previous interests to detect changes
  const prevInterestsRef = useRef<string[]>(userInterests);

  // Combine loading states
  const isLoading = metadataLoading;

  // Combine errors
  useEffect(() => {
    if (metadataError) {
      setError(metadataError);
    }
  }, [metadataError]);

  // T037: Detect interest changes and trigger refresh
  useEffect(() => {
    const prevInterests = prevInterestsRef.current;
    const currentInterests = userInterests;

    // Check if interests have actually changed
    const hasChanged =
      prevInterests.length !== currentInterests.length ||
      prevInterests.some((interest, index) => interest !== currentInterests[index]);

    if (hasChanged && prevInterests.length > 0) {
      // Interests changed - show refreshing state briefly
      console.log('[usePersonalizedContent] Interests changed, refreshing content');
      setIsRefreshing(true);

      // Clear refreshing state after brief delay (content recomputes via useMemo)
      const timer = setTimeout(() => {
        setIsRefreshing(false);
      }, 300);

      return () => clearTimeout(timer);
    }

    // Update ref for next comparison
    prevInterestsRef.current = currentInterests;
  }, [userInterests]);

  // Build personalized content view
  const content = useMemo<PersonalizedContentView | null>(() => {
    if (isLoading || error) {
      return null;
    }

    try {
      // Filter chapters by user interests
      const matchedChapters = filterChaptersByInterests(chapters, userInterests);

      // Sort by book order
      const sortedChapters = sortChaptersByBookOrder(matchedChapters);

      // Determine empty state
      const emptyState = userInterests.length === 0 || matchedChapters.length === 0;

      return {
        userInterests,
        matchedChapters: sortedChapters,
        emptyState,
        totalChapters: chapters.length,
        matchCount: matchedChapters.length,
      };
    } catch (err) {
      const errorMessage =
        err instanceof Error ? err : new Error('Failed to build personalized content');
      setError(errorMessage);
      console.error('[usePersonalizedContent] Error building content:', err);
      return null;
    }
  }, [chapters, userInterests, isLoading, error]);

  // Refetch function (triggers re-render and shows refresh state)
  const refetch = () => {
    setError(null);
    setIsRefreshing(true);

    // Content will be recomputed on next render due to useMemo dependencies
    setTimeout(() => {
      setIsRefreshing(false);
    }, 300);
  };

  return {
    content,
    isLoading,
    error,
    refetch,
    isRefreshing, // T037: Expose refreshing state
  };
}
