/**
 * Performance Tests (T053)
 * Verify that content filtering meets performance requirements
 *
 * Success Criteria:
 * - Filtering 50 chapters should complete in <500ms
 * - Sorting 50 chapters should complete in <100ms
 */

import { filterChaptersByInterests, sortChaptersByBookOrder } from '@site/src/utils/interestMapper';
import type { ChapterMetadata } from '@site/src/types/personalization';

describe('Performance Tests (T053)', () => {
  // Generate 50 mock chapters for performance testing
  const generateMockChapters = (count: number): ChapterMetadata[] => {
    const interests = ['physical-ai', 'ros2', 'kinematics', 'dynamics-control', 'sensors', 'humanoid-design', 'simulation', 'machine-learning'];
    return Array.from({ length: count }, (_, i) => ({
      id: `chapter-${i}`,
      title: `Chapter ${i}: Test Chapter`,
      url: `/docs/chapter-${i}`,
      interests: [interests[i % interests.length], interests[(i + 1) % interests.length]],
      module: `module${Math.floor(i / 10) + 1}`,
      order: i,
      description: `Test chapter ${i} description`,
    }));
  };

  describe('Filtering Performance', () => {
    it('should filter 50 chapters in less than 500ms', () => {
      const chapters = generateMockChapters(50);
      const userInterests = ['physical-ai', 'ros2', 'kinematics'];

      const startTime = performance.now();
      const result = filterChaptersByInterests(chapters, userInterests);
      const endTime = performance.now();

      const duration = endTime - startTime;

      expect(result.length).toBeGreaterThan(0);
      expect(duration).toBeLessThan(500);

      console.log(`[Performance] Filtered 50 chapters in ${duration.toFixed(2)}ms`);
    });

    it('should filter 100 chapters in less than 1000ms', () => {
      const chapters = generateMockChapters(100);
      const userInterests = ['physical-ai', 'ros2', 'kinematics', 'dynamics-control'];

      const startTime = performance.now();
      const result = filterChaptersByInterests(chapters, userInterests);
      const endTime = performance.now();

      const duration = endTime - startTime;

      expect(result.length).toBeGreaterThan(0);
      expect(duration).toBeLessThan(1000);

      console.log(`[Performance] Filtered 100 chapters in ${duration.toFixed(2)}ms`);
    });
  });

  describe('Sorting Performance', () => {
    it('should sort 50 chapters in less than 100ms', () => {
      const chapters = generateMockChapters(50);

      const startTime = performance.now();
      const result = sortChaptersByBookOrder(chapters);
      const endTime = performance.now();

      const duration = endTime - startTime;

      expect(result).toHaveLength(50);
      expect(duration).toBeLessThan(100);

      console.log(`[Performance] Sorted 50 chapters in ${duration.toFixed(2)}ms`);
    });
  });

  describe('Combined Filtering + Sorting Performance', () => {
    it('should filter and sort 50 chapters in less than 600ms', () => {
      const chapters = generateMockChapters(50);
      const userInterests = ['physical-ai', 'ros2', 'kinematics'];

      const startTime = performance.now();
      const filtered = filterChaptersByInterests(chapters, userInterests);
      const sorted = sortChaptersByBookOrder(filtered);
      const endTime = performance.now();

      const duration = endTime - startTime;

      expect(sorted.length).toBeGreaterThan(0);
      expect(duration).toBeLessThan(600);

      console.log(`[Performance] Filtered + sorted 50 chapters in ${duration.toFixed(2)}ms`);
    });
  });

  describe('Edge Cases Performance', () => {
    it('should handle empty interest list quickly', () => {
      const chapters = generateMockChapters(50);

      const startTime = performance.now();
      const result = filterChaptersByInterests(chapters, []);
      const endTime = performance.now();

      const duration = endTime - startTime;

      expect(result).toHaveLength(0);
      expect(duration).toBeLessThan(10);

      console.log(`[Performance] Empty interests handled in ${duration.toFixed(2)}ms`);
    });

    it('should handle all interests matching quickly', () => {
      const chapters = generateMockChapters(50);
      const allInterests = ['physical-ai', 'ros2', 'kinematics', 'dynamics-control', 'sensors', 'humanoid-design', 'simulation', 'machine-learning'];

      const startTime = performance.now();
      const result = filterChaptersByInterests(chapters, allInterests);
      const endTime = performance.now();

      const duration = endTime - startTime;

      expect(result.length).toBe(50); // All chapters should match
      expect(duration).toBeLessThan(100);

      console.log(`[Performance] All interests matching handled in ${duration.toFixed(2)}ms`);
    });
  });
});
