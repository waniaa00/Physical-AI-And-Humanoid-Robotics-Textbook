/**
 * Interest configuration - available interest categories
 *
 * This file defines all available interests that users can select.
 * To add a new interest:
 * 1. Add an entry to the AVAILABLE_INTERESTS array
 * 2. Tag relevant chapters in docs/ with the interest ID in frontmatter
 */

import type { Interest } from '../types/personalization';

/**
 * Available interests for personalization
 *
 * Each interest maps to content via frontmatter tags in Docusaurus chapters.
 * The `id` field must match the interest tags in chapter frontmatter.
 */
export const AVAILABLE_INTERESTS: Interest[] = [
  {
    id: 'physical-ai',
    label: 'Physical AI',
    description: 'AI systems that interact with the physical world through sensors and actuators',
    icon: '🤖',
  },
  {
    id: 'ros2',
    label: 'ROS 2 Fundamentals',
    description: 'Robot Operating System 2 - middleware, nodes, topics, services',
    icon: '⚙️',
  },
  {
    id: 'kinematics',
    label: 'Kinematics',
    description: 'Forward kinematics, inverse kinematics, Jacobians, and robot motion',
    icon: '🦾',
  },
  {
    id: 'dynamics-control',
    label: 'Dynamics & Control',
    description: 'Robot dynamics, control systems, PID controllers, and stability',
    icon: '🎛️',
  },
  {
    id: 'sensors',
    label: 'Sensors & Perception',
    description: 'IMUs, cameras, LIDAR, sensor fusion, and perception algorithms',
    icon: '👁️',
  },
  {
    id: 'humanoid-design',
    label: 'Humanoid Design',
    description: 'Humanoid robot architecture, hardware design, and mechanical systems',
    icon: '🧍',
  },
  {
    id: 'simulation',
    label: 'Simulation',
    description: 'Robot simulation environments, Gazebo, and virtual testing',
    icon: '🎮',
  },
  {
    id: 'machine-learning',
    label: 'Machine Learning',
    description: 'ML for robotics, reinforcement learning, and neural networks',
    icon: '🧠',
  },
];

/**
 * Get interest by ID
 */
export function getInterestById(id: string): Interest | undefined {
  return AVAILABLE_INTERESTS.find((interest) => interest.id === id);
}

/**
 * Get interests by IDs
 */
export function getInterestsByIds(ids: string[]): Interest[] {
  return ids
    .map((id) => getInterestById(id))
    .filter((interest): interest is Interest => interest !== undefined);
}

/**
 * Validate interest IDs
 */
export function validateInterestIds(ids: string[]): boolean {
  return ids.every((id) => AVAILABLE_INTERESTS.some((interest) => interest.id === id));
}

/**
 * Get all available interest IDs
 */
export function getAllInterestIds(): string[] {
  return AVAILABLE_INTERESTS.map((interest) => interest.id);
}
