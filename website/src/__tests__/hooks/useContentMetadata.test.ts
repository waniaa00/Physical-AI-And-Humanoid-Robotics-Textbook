/**
 * Unit tests for useContentMetadata hook (T060)
 * Tests metadata loading, caching, and error handling
 */

import { renderHook, waitFor } from '@testing-library/react';
import { useContentMetadata } from '@site/src/hooks/useContentMetadata';
import * as metadataExtractor from '@site/src/utils/metadataExtractor';

// Mock the metadata extractor
jest.mock('@site/src/utils/metadataExtractor');

describe('useContentMetadata Hook (T060)', () => {
  const mockChapters = [
    {
      id: 'chapter1',
      title: 'Chapter 1: Introduction',
      url: '/docs/chapter1',
      interests: ['physical-ai', 'ros2'],
      module: 'module1',
      order: 1,
      description: 'Introduction chapter',
    },
    {
      id: 'chapter2',
      title: 'Chapter 2: ROS2',
      url: '/docs/chapter2',
      interests: ['ros2'],
      module: 'module1',
      order: 2,
      description: 'ROS2 fundamentals',
    },
  ];

  beforeEach(() => {
    jest.clearAllMocks();
    jest.spyOn(console, 'error').mockImplementation(() => {});
  });

  afterEach(() => {
    (console.error as jest.Mock).mockRestore();
  });

  describe('successful metadata loading', () => {
    it('should start with loading state', () => {
      (metadataExtractor.extractAllChapterMetadata as jest.Mock).mockReturnValue(mockChapters);

      const { result } = renderHook(() => useContentMetadata());

      expect(result.current.isLoading).toBe(true);
      expect(result.current.chapters).toEqual([]);
      expect(result.current.error).toBeNull();
    });

    it('should load chapter metadata successfully', async () => {
      (metadataExtractor.extractAllChapterMetadata as jest.Mock).mockReturnValue(mockChapters);

      const { result } = renderHook(() => useContentMetadata());

      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });

      expect(result.current.chapters).toEqual(mockChapters);
      expect(result.current.error).toBeNull();
    });

    it('should call extractAllChapterMetadata once', async () => {
      (metadataExtractor.extractAllChapterMetadata as jest.Mock).mockReturnValue(mockChapters);

      const { result } = renderHook(() => useContentMetadata());

      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });

      expect(metadataExtractor.extractAllChapterMetadata).toHaveBeenCalledTimes(1);
    });

    it('should return correct number of chapters', async () => {
      (metadataExtractor.extractAllChapterMetadata as jest.Mock).mockReturnValue(mockChapters);

      const { result } = renderHook(() => useContentMetadata());

      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });

      expect(result.current.chapters).toHaveLength(2);
    });
  });

  describe('error handling', () => {
    it('should handle extraction errors gracefully', async () => {
      const mockError = new Error('Failed to extract metadata');
      (metadataExtractor.extractAllChapterMetadata as jest.Mock).mockImplementation(() => {
        throw mockError;
      });

      const { result } = renderHook(() => useContentMetadata());

      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });

      expect(result.current.error).toEqual(mockError);
      expect(result.current.chapters).toEqual([]);
      expect(console.error).toHaveBeenCalled();
    });

    it('should convert non-Error objects to Error', async () => {
      (metadataExtractor.extractAllChapterMetadata as jest.Mock).mockImplementation(() => {
        throw 'String error';
      });

      const { result } = renderHook(() => useContentMetadata());

      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });

      expect(result.current.error).toBeInstanceOf(Error);
      expect(result.current.error?.message).toBe('Failed to load chapter metadata');
    });
  });

  describe('edge cases', () => {
    it('should handle empty chapter list', async () => {
      (metadataExtractor.extractAllChapterMetadata as jest.Mock).mockReturnValue([]);

      const { result } = renderHook(() => useContentMetadata());

      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });

      expect(result.current.chapters).toEqual([]);
      expect(result.current.error).toBeNull();
    });

    it('should not reload on re-render', async () => {
      (metadataExtractor.extractAllChapterMetadata as jest.Mock).mockReturnValue(mockChapters);

      const { result, rerender } = renderHook(() => useContentMetadata());

      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });

      // Force re-render
      rerender();

      // Should not call extractAllChapterMetadata again
      expect(metadataExtractor.extractAllChapterMetadata).toHaveBeenCalledTimes(1);
    });
  });

  describe('loading delay simulation', () => {
    it('should complete loading within reasonable time', async () => {
      (metadataExtractor.extractAllChapterMetadata as jest.Mock).mockReturnValue(mockChapters);

      const startTime = performance.now();
      const { result } = renderHook(() => useContentMetadata());

      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });

      const endTime = performance.now();
      const duration = endTime - startTime;

      // Should complete within 250ms (100ms delay + overhead)
      expect(duration).toBeLessThan(250);
    });
  });
});
