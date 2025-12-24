/**
 * TypeScript types and interfaces for Personalized Content feature
 */

/**
 * Interest definition - a topic area users can select
 */
export interface Interest {
  id: string;              // e.g., "kinematics", "ros2"
  label: string;           // e.g., "Kinematics", "ROS 2 Fundamentals"
  description: string;     // e.g., "Forward/inverse kinematics, Jacobians"
  icon?: string;           // Optional emoji or icon identifier
}

/**
 * Chapter metadata extracted from Docusaurus frontmatter
 */
export interface ChapterMetadata {
  id: string;              // e.g., "chapter3-kinematics"
  title: string;           // e.g., "Chapter 3: Kinematics"
  url: string;             // e.g., "/docs/module1/chapter3-kinematics"
  interests: string[];     // e.g., ["kinematics", "robotics-fundamentals"]
  module?: string;         // e.g., "module1"
  order?: number;          // Chapter sequence number
  description?: string;    // Brief chapter description
}

/**
 * Personalized content view model
 */
export interface PersonalizedContentView {
  userInterests: string[];           // User's selected interest IDs
  matchedChapters: ChapterMetadata[]; // Chapters matching interests
  emptyState: boolean;                // True if no interests or no matches
  totalChapters: number;              // Total available chapters
  matchCount: number;                 // Number of matched chapters
}

/**
 * User profile extension (extends existing Better Auth user profile)
 */
export interface UserProfile {
  id: string;
  email: string;
  name?: string;
  interests: string[];  // Array of interest IDs
  createdAt?: Date;
  updatedAt?: Date;
}

/**
 * Filter options for content personalization
 */
export interface ContentFilterOptions {
  interests: string[];
  sortBy?: 'book-order' | 'relevance';
  limit?: number;
}

/**
 * Personalized content page props
 */
export interface PersonalizedContentPageProps {
  user: UserProfile | null;
  isLoading?: boolean;
}
