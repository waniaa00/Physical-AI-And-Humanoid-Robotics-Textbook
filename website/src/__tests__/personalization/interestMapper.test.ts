/**
 * Unit tests for interest mapper utility (T015)
 * Tests filtering chapters by user interests
 *
 * Test cases:
 * - Single interest match
 * - Multiple interest matches
 * - No matches
 * - Empty user interests
 * - Empty chapter interests
 * - Sorting by book order
 */

import {
  filterChaptersByInterests,
  sortChaptersByBookOrder,
  getInterestLabels,
} from '@site/src/utils/interestMapper';
import type { ChapterMetadata } from '@site/src/types/personalization';

describe('filterChaptersByInterests', () => {
  const mockChapters: ChapterMetadata[] = [
    {
      id: 'chapter1',
      title: 'Chapter 1: Introduction',
      url: '/docs/chapter1',
      interests: ['physical-ai', 'ros2'],
      module: 'module1',
      order: 1,
    },
    {
      id: 'chapter2',
      title: 'Chapter 2: ROS 2 Fundamentals',
      url: '/docs/chapter2',
      interests: ['ros2'],
      module: 'module1',
      order: 2,
    },
    {
      id: 'chapter3',
      title: 'Chapter 3: Kinematics',
      url: '/docs/chapter3',
      interests: ['kinematics'],
      module: 'module1',
      order: 3,
    },
    {
      id: 'chapter4',
      title: 'Chapter 4: Dynamics',
      url: '/docs/chapter4',
      interests: ['dynamics-control'],
      module: 'module1',
      order: 4,
    },
    {
      id: 'chapter5',
      title: 'Chapter 5: Sensors',
      url: '/docs/chapter5',
      interests: ['sensors', 'ros2'],
      module: 'module2',
      order: 5,
    },
  ];

  describe('single interest filtering', () => {
    it('should return chapters matching single interest', () => {
      const userInterests = ['kinematics'];
      const result = filterChaptersByInterests(mockChapters, userInterests);

      expect(result).toHaveLength(1);
      expect(result[0].id).toBe('chapter3');
      expect(result[0].interests).toContain('kinematics');
    });

    it('should return multiple chapters for interest that appears in multiple chapters', () => {
      const userInterests = ['ros2'];
      const result = filterChaptersByInterests(mockChapters, userInterests);

      expect(result).toHaveLength(3); // chapter1, chapter2, chapter5
      expect(result.map(ch => ch.id)).toEqual(
        expect.arrayContaining(['chapter1', 'chapter2', 'chapter5'])
      );
    });
  });

  describe('multiple interest filtering', () => {
    it('should return chapters matching ANY user interest (OR logic)', () => {
      const userInterests = ['kinematics', 'ros2'];
      const result = filterChaptersByInterests(mockChapters, userInterests);

      // Should match chapter1 (ros2), chapter2 (ros2), chapter3 (kinematics), chapter5 (ros2)
      expect(result).toHaveLength(4);
      expect(result.map(ch => ch.id)).toEqual(
        expect.arrayContaining(['chapter1', 'chapter2', 'chapter3', 'chapter5'])
      );
    });

    it('should not duplicate chapters with multiple matching interests', () => {
      const userInterests = ['physical-ai', 'ros2'];
      const result = filterChaptersByInterests(mockChapters, userInterests);

      // chapter1 has both interests but should appear only once
      const chapter1Count = result.filter(ch => ch.id === 'chapter1').length;
      expect(chapter1Count).toBe(1);
    });
  });

  describe('no matches', () => {
    it('should return empty array when no interests match', () => {
      const userInterests = ['nonexistent-interest'];
      const result = filterChaptersByInterests(mockChapters, userInterests);

      expect(result).toHaveLength(0);
      expect(result).toEqual([]);
    });

    it('should return empty array when user has no interests', () => {
      const userInterests: string[] = [];
      const result = filterChaptersByInterests(mockChapters, userInterests);

      expect(result).toHaveLength(0);
      expect(result).toEqual([]);
    });
  });

  describe('edge cases', () => {
    it('should handle chapters with no interests defined', () => {
      const chaptersWithEmpty: ChapterMetadata[] = [
        ...mockChapters,
        {
          id: 'chapter6',
          title: 'Chapter 6: No Interests',
          url: '/docs/chapter6',
          interests: [],
          module: 'module2',
          order: 6,
        },
      ];

      const userInterests = ['kinematics'];
      const result = filterChaptersByInterests(chaptersWithEmpty, userInterests);

      expect(result).toHaveLength(1);
      expect(result[0].id).toBe('chapter3');
    });

    it('should handle empty chapters array', () => {
      const userInterests = ['kinematics'];
      const result = filterChaptersByInterests([], userInterests);

      expect(result).toHaveLength(0);
      expect(result).toEqual([]);
    });
  });
});

describe('sortChaptersByBookOrder', () => {
  const unsortedChapters: ChapterMetadata[] = [
    {
      id: 'chapter5',
      title: 'Chapter 5',
      url: '/docs/chapter5',
      interests: ['sensors'],
      module: 'module2',
      order: 5,
    },
    {
      id: 'chapter1',
      title: 'Chapter 1',
      url: '/docs/chapter1',
      interests: ['physical-ai'],
      module: 'module1',
      order: 1,
    },
    {
      id: 'chapter3',
      title: 'Chapter 3',
      url: '/docs/chapter3',
      interests: ['kinematics'],
      module: 'module1',
      order: 3,
    },
  ];

  it('should sort chapters by module and order', () => {
    const result = sortChaptersByBookOrder(unsortedChapters);

    expect(result).toHaveLength(3);
    expect(result[0].id).toBe('chapter1'); // module1, order 1
    expect(result[1].id).toBe('chapter3'); // module1, order 3
    expect(result[2].id).toBe('chapter5'); // module2, order 5
  });

  it('should not mutate original array', () => {
    const original = [...unsortedChapters];
    sortChaptersByBookOrder(unsortedChapters);

    expect(unsortedChapters).toEqual(original);
  });

  it('should handle chapters without order field', () => {
    const chaptersWithoutOrder: ChapterMetadata[] = [
      {
        id: 'chapter-z',
        title: 'Chapter Z',
        url: '/docs/z',
        interests: ['test'],
      },
      {
        id: 'chapter-a',
        title: 'Chapter A',
        url: '/docs/a',
        interests: ['test'],
      },
    ];

    const result = sortChaptersByBookOrder(chaptersWithoutOrder);

    // Should fallback to URL sorting
    expect(result[0].id).toBe('chapter-a');
    expect(result[1].id).toBe('chapter-z');
  });
});

describe('getInterestLabels', () => {
  it('should return human-readable labels for interest IDs', () => {
    const interestIds = ['physical-ai', 'ros2', 'kinematics'];
    const result = getInterestLabels(interestIds);

    expect(result).toEqual(['Physical AI', 'ROS 2 Fundamentals', 'Kinematics']);
  });

  it('should handle empty interest IDs array', () => {
    const result = getInterestLabels([]);

    expect(result).toEqual([]);
  });

  it('should skip unknown interest IDs', () => {
    const interestIds = ['physical-ai', 'unknown-interest', 'ros2'];
    const result = getInterestLabels(interestIds);

    expect(result).toEqual(['Physical AI', 'ROS 2 Fundamentals']);
    expect(result).not.toContain('unknown-interest');
  });

  it('should preserve order of input IDs', () => {
    const interestIds = ['kinematics', 'physical-ai', 'ros2'];
    const result = getInterestLabels(interestIds);

    expect(result[0]).toBe('Kinematics');
    expect(result[1]).toBe('Physical AI');
    expect(result[2]).toBe('ROS 2 Fundamentals');
  });
});
