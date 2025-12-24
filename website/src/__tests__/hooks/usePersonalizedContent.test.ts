/**
 * Unit tests for usePersonalizedContent hook (T061)
 * Tests content filtering, interest change detection, and refresh mechanism
 */

import { renderHook, waitFor, act } from '@testing-library/react';
import { usePersonalizedContent } from '@site/src/hooks/usePersonalizedContent';
import * as useContentMetadataModule from '@site/src/hooks/useContentMetadata';

// Mock useContentMetadata hook
jest.mock('@site/src/hooks/useContentMetadata');

describe('usePersonalizedContent Hook (T061)', () => {
  const mockChapters = [
    {
      id: 'chapter1',
      title: 'Chapter 1: Introduction to Physical AI',
      url: '/docs/chapter1',
      interests: ['physical-ai', 'ros2'],
      module: 'module1',
      order: 1,
      description: 'Introduction',
    },
    {
      id: 'chapter2',
      title: 'Chapter 2: ROS2 Fundamentals',
      url: '/docs/chapter2',
      interests: ['ros2'],
      module: 'module1',
      order: 2,
      description: 'ROS2',
    },
    {
      id: 'chapter3',
      title: 'Chapter 3: Kinematics',
      url: '/docs/chapter3',
      interests: ['kinematics'],
      module: 'module1',
      order: 3,
      description: 'Kinematics',
    },
  ];

  const mockUseContentMetadata = useContentMetadataModule.useContentMetadata as jest.MockedFunction<
    typeof useContentMetadataModule.useContentMetadata
  >;

  beforeEach(() => {
    jest.clearAllMocks();
    jest.spyOn(console, 'log').mockImplementation(() => {});
    jest.spyOn(console, 'error').mockImplementation(() => {});

    // Default mock: successful metadata load
    mockUseContentMetadata.mockReturnValue({
      chapters: mockChapters,
      isLoading: false,
      error: null,
    });
  });

  afterEach(() => {
    (console.log as jest.Mock).mockRestore();
    (console.error as jest.Mock).mockRestore();
  });

  describe('content filtering', () => {
    it('should filter chapters by user interests', async () => {
      const { result } = renderHook(() => usePersonalizedContent(['physical-ai']));

      await waitFor(() => {
        expect(result.current.content).not.toBeNull();
      });

      expect(result.current.content?.matchedChapters).toHaveLength(1);
      expect(result.current.content?.matchedChapters[0].id).toBe('chapter1');
    });

    it('should filter chapters with multiple interests (OR logic)', async () => {
      const { result } = renderHook(() => usePersonalizedContent(['physical-ai', 'kinematics']));

      await waitFor(() => {
        expect(result.current.content).not.toBeNull();
      });

      expect(result.current.content?.matchedChapters).toHaveLength(2);
      expect(result.current.content?.matchedChapters.map(c => c.id)).toContain('chapter1');
      expect(result.current.content?.matchedChapters.map(c => c.id)).toContain('chapter3');
    });

    it('should return empty array for no interest matches', async () => {
      const { result } = renderHook(() => usePersonalizedContent(['nonexistent-interest']));

      await waitFor(() => {
        expect(result.current.content).not.toBeNull();
      });

      expect(result.current.content?.matchedChapters).toHaveLength(0);
      expect(result.current.content?.emptyState).toBe(true); // No matches = empty state
    });

    it('should return empty state when user has no interests', async () => {
      const { result } = renderHook(() => usePersonalizedContent([]));

      await waitFor(() => {
        expect(result.current.content).not.toBeNull();
      });

      expect(result.current.content?.matchedChapters).toHaveLength(0);
      expect(result.current.content?.emptyState).toBe(true);
    });
  });

  describe('sorting', () => {
    it('should sort chapters by book order', async () => {
      const { result } = renderHook(() => usePersonalizedContent(['ros2', 'kinematics']));

      await waitFor(() => {
        expect(result.current.content).not.toBeNull();
      });

      const chapters = result.current.content?.matchedChapters;
      expect(chapters?.[0].order).toBeLessThan(chapters?.[1].order || 0);
    });
  });

  describe('interest change detection (T037)', () => {
    it('should detect when interests change', async () => {
      const { result, rerender } = renderHook(
        ({ interests }) => usePersonalizedContent(interests),
        { initialProps: { interests: ['physical-ai'] } }
      );

      await waitFor(() => {
        expect(result.current.content).not.toBeNull();
      });

      // Change interests
      act(() => {
        rerender({ interests: ['kinematics'] });
      });

      // Should trigger refresh
      await waitFor(() => {
        expect(result.current.isRefreshing).toBe(true);
      }, { timeout: 100 });

      // Should clear after delay
      await waitFor(() => {
        expect(result.current.isRefreshing).toBe(false);
      }, { timeout: 500 });

      expect(console.log).toHaveBeenCalledWith(
        '[usePersonalizedContent] Interests changed, refreshing content'
      );
    });

    it('should not trigger refresh on initial render', async () => {
      const { result } = renderHook(() => usePersonalizedContent(['physical-ai']));

      await waitFor(() => {
        expect(result.current.content).not.toBeNull();
      });

      expect(result.current.isRefreshing).toBe(false);
    });

    it('should not trigger refresh when interests stay the same', async () => {
      const { result, rerender } = renderHook(
        ({ interests }) => usePersonalizedContent(interests),
        { initialProps: { interests: ['physical-ai'] } }
      );

      await waitFor(() => {
        expect(result.current.content).not.toBeNull();
      });

      jest.clearAllMocks();

      // Re-render with same interests
      rerender({ interests: ['physical-ai'] });

      await waitFor(() => {
        expect(result.current.isRefreshing).toBe(false);
      });

      expect(console.log).not.toHaveBeenCalled();
    });
  });

  describe('loading states', () => {
    it('should return null content while metadata is loading', () => {
      mockUseContentMetadata.mockReturnValue({
        chapters: [],
        isLoading: true,
        error: null,
      });

      const { result } = renderHook(() => usePersonalizedContent(['physical-ai']));

      expect(result.current.content).toBeNull();
      expect(result.current.isLoading).toBe(true);
    });

    it('should complete loading when metadata loads', async () => {
      mockUseContentMetadata.mockReturnValue({
        chapters: mockChapters,
        isLoading: false,
        error: null,
      });

      const { result } = renderHook(() => usePersonalizedContent(['physical-ai']));

      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });

      expect(result.current.content).not.toBeNull();
    });
  });

  describe('error handling', () => {
    it('should handle metadata errors', async () => {
      const mockError = new Error('Failed to load metadata');
      mockUseContentMetadata.mockReturnValue({
        chapters: [],
        isLoading: false,
        error: mockError,
      });

      const { result } = renderHook(() => usePersonalizedContent(['physical-ai']));

      await waitFor(() => {
        expect(result.current.error).toBe(mockError);
      });

      expect(result.current.content).toBeNull();
    });

    it('should handle filtering errors gracefully', async () => {
      // Mock chapters with invalid data that might cause filtering errors
      mockUseContentMetadata.mockReturnValue({
        chapters: [
          {
            id: 'broken',
            title: 'Broken',
            url: '/broken',
            interests: null as any, // Invalid data
            module: 'module1',
            order: 1,
            description: 'Broken',
          },
        ],
        isLoading: false,
        error: null,
      });

      const { result } = renderHook(() => usePersonalizedContent(['physical-ai']));

      await waitFor(() => {
        expect(result.current.isLoading).toBe(false);
      });

      // Should handle gracefully - either sets error or filters safely
      // The hook has defensive coding that prevents errors
      if (result.current.error) {
        expect(result.current.content).toBeNull();
      } else {
        // Defensive code handled it safely
        expect(result.current.content?.matchedChapters).toBeDefined();
      }
    });
  });

  describe('refetch functionality', () => {
    it('should provide refetch function', async () => {
      const { result } = renderHook(() => usePersonalizedContent(['physical-ai']));

      await waitFor(() => {
        expect(result.current.content).not.toBeNull();
      });

      expect(result.current.refetch).toBeInstanceOf(Function);
    });

    it('should trigger refresh when refetch is called', async () => {
      const { result } = renderHook(() => usePersonalizedContent(['physical-ai']));

      await waitFor(() => {
        expect(result.current.content).not.toBeNull();
      });

      act(() => {
        result.current.refetch();
      });

      await waitFor(() => {
        expect(result.current.isRefreshing).toBe(true);
      });

      await waitFor(() => {
        expect(result.current.isRefreshing).toBe(false);
      }, { timeout: 500 });
    });
  });

  describe('content metadata', () => {
    it('should include correct metadata in content view', async () => {
      const { result } = renderHook(() => usePersonalizedContent(['physical-ai', 'ros2']));

      await waitFor(() => {
        expect(result.current.content).not.toBeNull();
      });

      expect(result.current.content?.userInterests).toEqual(['physical-ai', 'ros2']);
      expect(result.current.content?.totalChapters).toBe(3);
      expect(result.current.content?.matchCount).toBe(2); // chapter1 and chapter2
    });
  });
});
