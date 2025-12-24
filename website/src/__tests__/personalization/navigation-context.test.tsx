/**
 * Integration test for preserving personalized context across navigation (T042)
 * Verifies that user interests and personalized state persist when navigating between pages
 */

import { renderHook } from '@testing-library/react';
import { usePersonalizedContent } from '@site/src/hooks/usePersonalizedContent';

// Mock dependencies
jest.mock('@site/src/hooks/useContentMetadata', () => ({
  useContentMetadata: () => ({
    chapters: [
      {
        id: 'chapter1',
        title: 'Chapter 1',
        url: '/docs/chapter1',
        interests: ['physical-ai'],
        module: 'module1',
        order: 1,
      },
    ],
    isLoading: false,
    error: null,
  }),
}));

describe('Navigation Context Preservation', () => {
  it('should preserve user interests across hook re-renders', () => {
    const interests = ['physical-ai', 'ros2'];

    const { result, rerender } = renderHook(
      ({ userInterests }) => usePersonalizedContent(userInterests),
      { initialProps: { userInterests: interests } }
    );

    const firstContent = result.current.content;
    expect(firstContent?.userInterests).toEqual(interests);

    // Simulate navigation/re-render
    rerender({ userInterests: interests });

    // Content should remain consistent
    expect(result.current.content?.userInterests).toEqual(interests);
    expect(result.current.content).toBeTruthy();
  });

  it('should handle interest changes during navigation', () => {
    const { result, rerender } = renderHook(
      ({ userInterests }) => usePersonalizedContent(userInterests),
      { initialProps: { userInterests: ['physical-ai'] } }
    );

    expect(result.current.content?.userInterests).toEqual(['physical-ai']);

    // User changes interests (e.g., via profile page)
    rerender({ userInterests: ['physical-ai', 'kinematics'] });

    // Content should update
    expect(result.current.content?.userInterests).toEqual(['physical-ai', 'kinematics']);
  });

  it('should maintain personalized content view after navigation', () => {
    const interests = ['physical-ai'];

    const { result } = renderHook(() => usePersonalizedContent(interests));

    // Verify content is built correctly
    expect(result.current.content).toBeTruthy();
    expect(result.current.content?.matchedChapters).toBeDefined();
    expect(result.current.error).toBeNull();
  });
});
