/**
 * Component tests for PersonalizedContentView (T016)
 * Tests rendering matched chapters, empty state, and navigation
 *
 * Test cases:
 * - Renders matched chapters as cards
 * - Displays chapter count
 * - Renders navigation links
 * - Handles empty state
 * - Renders interest badges
 */

import React from 'react';
import { render, screen } from '@testing-library/react';
import '@testing-library/jest-dom';
import PersonalizedContentView from '@site/src/components/PersonalizedContent/PersonalizedContentView';
import type { PersonalizedContentView as PersonalizedContentViewType } from '@site/src/types/personalization';

describe('PersonalizedContentView', () => {
  const mockContent: PersonalizedContentViewType = {
    userInterests: ['physical-ai', 'ros2'],
    matchedChapters: [
      {
        id: 'chapter1',
        title: 'Chapter 1: Introduction to Physical AI',
        url: '/docs/chapter1',
        interests: ['physical-ai', 'ros2'],
        module: 'module1',
        order: 1,
        description: 'Introduction to physical AI systems',
      },
      {
        id: 'chapter2',
        title: 'Chapter 2: ROS 2 Fundamentals',
        url: '/docs/chapter2',
        interests: ['ros2'],
        module: 'module1',
        order: 2,
        description: 'Robot Operating System 2 basics',
      },
    ],
    emptyState: false,
    totalChapters: 8,
    matchCount: 2,
  };

  describe('rendering matched chapters', () => {
    it('should render all matched chapters', () => {
      render(<PersonalizedContentView content={mockContent} />);

      expect(screen.getByText('Chapter 1: Introduction to Physical AI')).toBeInTheDocument();
      expect(screen.getByText('Chapter 2: ROS 2 Fundamentals')).toBeInTheDocument();
    });

    it('should display chapter descriptions', () => {
      render(<PersonalizedContentView content={mockContent} />);

      expect(screen.getByText(/Introduction to physical AI systems/i)).toBeInTheDocument();
      expect(screen.getByText(/Robot Operating System 2 basics/i)).toBeInTheDocument();
    });

    it('should render clickable chapter links', () => {
      render(<PersonalizedContentView content={mockContent} />);

      const chapter1Link = screen.getByRole('link', { name: /Chapter 1: Introduction to Physical AI/i });
      const chapter2Link = screen.getByRole('link', { name: /Chapter 2: ROS 2 Fundamentals/i });

      expect(chapter1Link).toHaveAttribute('href', '/docs/chapter1');
      expect(chapter2Link).toHaveAttribute('href', '/docs/chapter2');
    });

    it('should display interest badges for each chapter', () => {
      render(<PersonalizedContentView content={mockContent} />);

      // Chapter 1 has both 'physical-ai' and 'ros2'
      const physicalAIBadges = screen.getAllByText(/Physical AI/i);
      expect(physicalAIBadges.length).toBeGreaterThanOrEqual(1);

      // Both chapters have 'ros2'
      const ros2Badges = screen.getAllByText(/ROS 2/i);
      expect(ros2Badges.length).toBeGreaterThanOrEqual(2);
    });
  });

  describe('chapter count display', () => {
    it('should display matched chapter count', () => {
      render(<PersonalizedContentView content={mockContent} />);

      expect(screen.getByText(/2.*chapters?/i)).toBeInTheDocument();
    });

    it('should display total chapter count', () => {
      render(<PersonalizedContentView content={mockContent} />);

      // Expecting something like "2 of 8 chapters" or "2 chapters (8 total)"
      expect(screen.getByText(/8/)).toBeInTheDocument();
    });

    it('should update count when content changes', () => {
      const { rerender } = render(<PersonalizedContentView content={mockContent} />);

      expect(screen.getByText(/2.*chapters?/i)).toBeInTheDocument();

      const updatedContent: PersonalizedContentViewType = {
        ...mockContent,
        matchedChapters: [mockContent.matchedChapters[0]],
        matchCount: 1,
      };

      rerender(<PersonalizedContentView content={updatedContent} />);

      expect(screen.getByText(/1.*chapter?/i)).toBeInTheDocument();
    });
  });

  describe('navigation', () => {
    it('should render "View Full Book" link', () => {
      render(<PersonalizedContentView content={mockContent} />);

      const viewFullBookLink = screen.getByRole('link', { name: /view full book/i });
      expect(viewFullBookLink).toBeInTheDocument();
      expect(viewFullBookLink).toHaveAttribute('href', '/docs/intro');
    });

    it('should render navigation component', () => {
      render(<PersonalizedContentView content={mockContent} />);

      // PersonalizedNav should be present
      expect(screen.getByRole('navigation')).toBeInTheDocument();
    });
  });

  describe('empty state', () => {
    it('should render InterestPrompt when user has no interests (T033)', () => {
      const emptyContent: PersonalizedContentViewType = {
        userInterests: [],
        matchedChapters: [],
        emptyState: true,
        totalChapters: 8,
        matchCount: 0,
      };

      render(<PersonalizedContentView content={emptyContent} />);

      // Should show InterestPrompt component
      expect(screen.getByTestId('interest-prompt')).toBeInTheDocument();
      expect(screen.getByRole('heading', { name: /select your interests/i })).toBeInTheDocument();
      expect(screen.getByRole('link', { name: /select interests/i })).toBeInTheDocument();
    });

    it('should not render chapters when emptyState is true', () => {
      const emptyContent: PersonalizedContentViewType = {
        userInterests: [],
        matchedChapters: [],
        emptyState: true,
        totalChapters: 8,
        matchCount: 0,
      };

      render(<PersonalizedContentView content={emptyContent} />);

      expect(screen.queryByText('Chapter 1')).not.toBeInTheDocument();
      expect(screen.queryByText('Chapter 2')).not.toBeInTheDocument();
    });

    it('should display empty state message when no chapters match', () => {
      const emptyContent: PersonalizedContentViewType = {
        userInterests: ['kinematics'],
        matchedChapters: [],
        emptyState: true,
        totalChapters: 8,
        matchCount: 0,
      };

      render(<PersonalizedContentView content={emptyContent} />);

      expect(screen.getByText(/no chapters found/i)).toBeInTheDocument();
    });
  });

  describe('loading state', () => {
    it('should handle null content gracefully', () => {
      render(<PersonalizedContentView content={null} />);

      expect(screen.getByText(/loading/i)).toBeInTheDocument();
    });
  });

  describe('accessibility', () => {
    it('should have proper heading structure', () => {
      render(<PersonalizedContentView content={mockContent} />);

      const headings = screen.getAllByRole('heading');
      expect(headings.length).toBeGreaterThan(0);
    });

    it('should have accessible chapter cards', () => {
      render(<PersonalizedContentView content={mockContent} />);

      const articles = screen.getAllByRole('article');
      expect(articles).toHaveLength(2); // One for each chapter
    });
  });
});
