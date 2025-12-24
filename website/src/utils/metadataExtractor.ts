/**
 * Metadata Extractor Utility (T009)
 * Extracts chapter metadata from Docusaurus configuration
 *
 * NOTE: This is a simplified implementation using static data.
 * In production, this would use Docusaurus plugin APIs to extract
 * metadata from actual documentation files.
 *
 * TODO (Research Phase - T005): Replace with actual Docusaurus metadata API
 */

import type { ChapterMetadata } from '../types/personalization';

/**
 * Static chapter metadata
 * This represents what would be extracted from Docusaurus frontmatter
 *
 * In production, this would be generated at build time from actual
 * documentation files with frontmatter like:
 * ---
 * title: "Chapter 1: Introduction"
 * interests: ["physical-ai", "ros2"]
 * ---
 */
const STATIC_CHAPTER_METADATA: ChapterMetadata[] = [
  {
    id: 'chapter1-introduction',
    title: 'Chapter 1: Introduction to Physical AI',
    url: '/docs/module1/chapter1-introduction',
    interests: ['physical-ai', 'ros2'],
    module: 'module1',
    order: 1,
    description:
      'Introduction to physical AI systems and their role in robotics',
  },
  {
    id: 'chapter2-ros2-fundamentals',
    title: 'Chapter 2: ROS 2 Fundamentals',
    url: '/docs/module1/chapter2-ros2-fundamentals',
    interests: ['ros2'],
    module: 'module1',
    order: 2,
    description:
      'Learn about ROS 2 architecture, nodes, topics, services, and actions',
  },
  {
    id: 'chapter3-kinematics',
    title: 'Chapter 3: Kinematics',
    url: '/docs/module1/chapter3-kinematics',
    interests: ['kinematics'],
    module: 'module1',
    order: 3,
    description:
      'Forward and inverse kinematics, Jacobians, and robot motion planning',
  },
  {
    id: 'chapter4-dynamics-control',
    title: 'Chapter 4: Dynamics and Control',
    url: '/docs/module1/chapter4-dynamics-control',
    interests: ['dynamics-control'],
    module: 'module1',
    order: 4,
    description:
      'Robot dynamics, control systems, PID controllers, and stability analysis',
  },
  {
    id: 'chapter5-sensors-perception',
    title: 'Chapter 5: Sensors and Perception',
    url: '/docs/module1/chapter5-sensors-perception',
    interests: ['sensors', 'ros2'],
    module: 'module1',
    order: 5,
    description: 'IMUs, cameras, LIDAR, sensor fusion, and perception algorithms',
  },
  {
    id: 'chapter6-humanoid-design',
    title: 'Chapter 6: Humanoid Robot Design',
    url: '/docs/module1/chapter6-humanoid-design',
    interests: ['humanoid-design', 'physical-ai'],
    module: 'module1',
    order: 6,
    description: 'Humanoid robot architecture, hardware design, and mechanical systems',
  },
  {
    id: 'chapter7-simulation',
    title: 'Chapter 7: Simulation and Testing',
    url: '/docs/module1/chapter7-simulation',
    interests: ['simulation', 'ros2'],
    module: 'module1',
    order: 7,
    description: 'Robot simulation environments, Gazebo, and virtual testing',
  },
  {
    id: 'chapter8-machine-learning',
    title: 'Chapter 8: Machine Learning for Robotics',
    url: '/docs/module1/chapter8-machine-learning',
    interests: ['machine-learning', 'physical-ai'],
    module: 'module1',
    order: 8,
    description: 'ML for robotics, reinforcement learning, and neural networks',
  },
];

/**
 * Extract all chapter metadata
 * Returns the complete list of available chapters
 *
 * @returns Array of chapter metadata
 */
export function extractAllChapterMetadata(): ChapterMetadata[] {
  return STATIC_CHAPTER_METADATA;
}

/**
 * Get chapter by ID
 *
 * @param chapterId - Chapter ID to find
 * @returns Chapter metadata or undefined
 */
export function getChapterById(chapterId: string): ChapterMetadata | undefined {
  return STATIC_CHAPTER_METADATA.find((chapter) => chapter.id === chapterId);
}

/**
 * Get chapters by module
 *
 * @param moduleId - Module ID to filter by
 * @returns Chapters in the specified module
 */
export function getChaptersByModule(moduleId: string): ChapterMetadata[] {
  return STATIC_CHAPTER_METADATA.filter((chapter) => chapter.module === moduleId);
}

/**
 * Get total chapter count
 *
 * @returns Total number of chapters
 */
export function getTotalChapterCount(): number {
  return STATIC_CHAPTER_METADATA.length;
}

/**
 * Future: Replace with actual Docusaurus plugin integration
 *
 * This function will be implemented in the research phase (T005) to:
 * 1. Use Docusaurus @docusaurus/plugin-content-docs API
 * 2. Extract frontmatter from all .md files in docs/
 * 3. Build chapter metadata dynamically at build time
 * 4. Cache metadata for runtime access
 *
 * Example of future implementation:
 * ```typescript
 * import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
 * import { useAllDocsData } from '@docusaurus/plugin-content-docs/client';
 *
 * export function extractChapterMetadataFromDocusaurus(): ChapterMetadata[] {
 *   const docsData = useAllDocsData();
 *   // Extract and transform metadata from Docusaurus...
 * }
 * ```
 */
