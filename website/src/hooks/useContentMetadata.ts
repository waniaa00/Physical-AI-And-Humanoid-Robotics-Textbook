/**
 * useContentMetadata Hook (T010)
 * Fetches and caches all chapter metadata
 */

import { useState, useEffect } from 'react';
import type { ChapterMetadata } from '../types/personalization';
import { extractAllChapterMetadata } from '../utils/metadataExtractor';

export interface UseContentMetadataResult {
  chapters: ChapterMetadata[];
  isLoading: boolean;
  error: Error | null;
}

/**
 * Hook to fetch and cache all chapter metadata
 * In production, this would integrate with Docusaurus APIs
 *
 * @returns Chapter metadata, loading state, and error state
 */
export function useContentMetadata(): UseContentMetadataResult {
  const [chapters, setChapters] = useState<ChapterMetadata[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<Error | null>(null);

  useEffect(() => {
    loadChapterMetadata();
  }, []);

  const loadChapterMetadata = async () => {
    try {
      setIsLoading(true);
      setError(null);

      // Simulate async loading (in production, this would fetch from Docusaurus API)
      // Add small delay to simulate real async operation
      await new Promise((resolve) => setTimeout(resolve, 100));

      const metadata = extractAllChapterMetadata();
      setChapters(metadata);
    } catch (err) {
      const errorMessage =
        err instanceof Error ? err : new Error('Failed to load chapter metadata');
      setError(errorMessage);
      console.error('[useContentMetadata] Error loading metadata:', err);
    } finally {
      setIsLoading(false);
    }
  };

  return {
    chapters,
    isLoading,
    error,
  };
}
