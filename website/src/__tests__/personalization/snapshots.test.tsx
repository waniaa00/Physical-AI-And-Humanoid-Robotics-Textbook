/**
 * Component snapshot tests (T062)
 * Ensures components render consistently and catches unintended UI changes
 */

import React from 'react';
import { render } from '@testing-library/react';
import '@testing-library/jest-dom';
import ContentCard from '@site/src/components/PersonalizedContent/ContentCard';
import PersonalizedNav from '@site/src/components/PersonalizedContent/PersonalizedNav';
import InterestPrompt from '@site/src/components/PersonalizedContent/InterestPrompt';
import PersonalizedContentView from '@site/src/components/PersonalizedContent/PersonalizedContentView';
import type { ChapterMetadata, PersonalizedContentView as PersonalizedContentViewType } from '@site/src/types/personalization';

// Mock Docusaurus Link
jest.mock('@docusaurus/Link', () => {
  return function MockLink({ children, to, ...props }: any) {
    return <a href={to} {...props}>{children}</a>;
  };
});

describe('Component Snapshots (T062)', () => {
  describe('ContentCard', () => {
    const mockChapter: ChapterMetadata = {
      id: 'chapter1',
      title: 'Chapter 1: Introduction to Physical AI',
      url: '/docs/chapter1',
      interests: ['physical-ai', 'ros2'],
      module: 'module1',
      order: 1,
      description: 'Introduction to physical AI systems and their role in robotics',
    };

    it('should match snapshot with full data', () => {
      const { container } = render(<ContentCard chapter={mockChapter} />);
      expect(container.firstChild).toMatchSnapshot();
    });

    it('should match snapshot without description', () => {
      const chapterNoDesc = { ...mockChapter, description: undefined };
      const { container } = render(<ContentCard chapter={chapterNoDesc} />);
      expect(container.firstChild).toMatchSnapshot();
    });

    it('should match snapshot with no interests', () => {
      const chapterNoInterests = { ...mockChapter, interests: [] };
      const { container } = render(<ContentCard chapter={chapterNoInterests} />);
      expect(container.firstChild).toMatchSnapshot();
    });

    it('should match snapshot with single interest', () => {
      const chapterSingleInterest = { ...mockChapter, interests: ['physical-ai'] };
      const { container } = render(<ContentCard chapter={chapterSingleInterest} />);
      expect(container.firstChild).toMatchSnapshot();
    });
  });

  describe('PersonalizedNav', () => {
    it('should match snapshot', () => {
      const { container } = render(<PersonalizedNav />);
      expect(container.firstChild).toMatchSnapshot();
    });
  });

  describe('InterestPrompt', () => {
    it('should match snapshot', () => {
      const { container } = render(<InterestPrompt />);
      expect(container.firstChild).toMatchSnapshot();
    });
  });

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
          title: 'Chapter 2: ROS2 Fundamentals',
          url: '/docs/chapter2',
          interests: ['ros2'],
          module: 'module1',
          order: 2,
          description: 'Learn about ROS 2 architecture',
        },
      ],
      emptyState: false,
      totalChapters: 8,
      matchCount: 2,
    };

    it('should match snapshot with content', () => {
      const { container } = render(<PersonalizedContentView content={mockContent} />);
      expect(container.firstChild).toMatchSnapshot();
    });

    it('should match snapshot in loading state', () => {
      const { container } = render(<PersonalizedContentView content={null} />);
      expect(container.firstChild).toMatchSnapshot();
    });

    it('should match snapshot with empty state (no interests)', () => {
      const emptyContent: PersonalizedContentViewType = {
        userInterests: [],
        matchedChapters: [],
        emptyState: true,
        totalChapters: 8,
        matchCount: 0,
      };
      const { container } = render(<PersonalizedContentView content={emptyContent} />);
      expect(container.firstChild).toMatchSnapshot();
    });

    it('should match snapshot with empty state (no matches)', () => {
      const noMatchesContent: PersonalizedContentViewType = {
        userInterests: ['some-interest'],
        matchedChapters: [],
        emptyState: true,
        totalChapters: 8,
        matchCount: 0,
      };
      const { container } = render(<PersonalizedContentView content={noMatchesContent} />);
      expect(container.firstChild).toMatchSnapshot();
    });

    it('should match snapshot with single chapter', () => {
      const singleChapterContent: PersonalizedContentViewType = {
        ...mockContent,
        matchedChapters: [mockContent.matchedChapters[0]],
        matchCount: 1,
      };
      const { container } = render(<PersonalizedContentView content={singleChapterContent} />);
      expect(container.firstChild).toMatchSnapshot();
    });

    it('should match snapshot with many chapters', () => {
      const manyChapters = Array.from({ length: 5 }, (_, i) => ({
        id: `chapter${i + 1}`,
        title: `Chapter ${i + 1}: Test Chapter`,
        url: `/docs/chapter${i + 1}`,
        interests: ['physical-ai'],
        module: 'module1',
        order: i + 1,
        description: `Test chapter ${i + 1}`,
      }));

      const manyChaptersContent: PersonalizedContentViewType = {
        userInterests: ['physical-ai'],
        matchedChapters: manyChapters,
        emptyState: false,
        totalChapters: 10,
        matchCount: 5,
      };

      const { container } = render(<PersonalizedContentView content={manyChaptersContent} />);
      expect(container.firstChild).toMatchSnapshot();
    });
  });
});
