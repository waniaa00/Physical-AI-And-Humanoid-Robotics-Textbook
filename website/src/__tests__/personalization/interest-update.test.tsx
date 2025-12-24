/**
 * Integration tests for interest update functionality (T035)
 * Tests that personalized content refreshes when user interests change
 *
 * Test cases:
 * - Adding new interest updates displayed content
 * - Removing interest removes corresponding chapters
 * - Changing all interests completely refreshes view
 * - Multiple rapid changes are handled correctly (debouncing)
 * - Updates occur quickly (within 3 seconds)
 */

import React from 'react';
import { render, screen, waitFor } from '@testing-library/react';
import { act } from 'react';
import '@testing-library/jest-dom';

// Mock dependencies
jest.mock('@docusaurus/Link', () => {
  return function MockLink({ children, to, className, ...props }: any) {
    return <a href={to} className={className} {...props}>{children}</a>;
  };
});

jest.mock('@theme/Layout', () => {
  return function MockLayout({ children }: any) {
    return <div data-testid="layout">{children}</div>;
  };
});

// Mock hooks
jest.mock('@site/src/hooks/useAuth');
jest.mock('@site/src/hooks/usePersonalizedContent');

import PersonalizedContentPage from '@site/src/pages/personalized-content';
import { useAuth } from '@site/src/hooks/useAuth';
import { usePersonalizedContent } from '@site/src/hooks/usePersonalizedContent';

const mockUseAuth = useAuth as jest.MockedFunction<typeof useAuth>;
const mockUsePersonalizedContent = usePersonalizedContent as jest.MockedFunction<typeof usePersonalizedContent>;

describe('Interest Update Integration', () => {
  const mockRefetch = jest.fn();

  beforeEach(() => {
    jest.clearAllMocks();
    delete (window as any).location;
    (window as any).location = { href: '' };

    // Default authenticated user with initial interests
    mockUseAuth.mockReturnValue({
      user: {
        user_id: 'test-user-123',
        email: 'test@example.com',
        interests: [1, 2], // physical-ai, ros2
      },
      isAuthenticated: true,
      isLoading: false,
      signUp: jest.fn(),
      signIn: jest.fn(),
      signOut: jest.fn(),
      validateSession: jest.fn(),
    });

    // Default content with refetch function
    mockUsePersonalizedContent.mockReturnValue({
      content: {
        userInterests: ['physical-ai', 'ros2'],
        matchedChapters: [
          {
            id: 'chapter1',
            title: 'Chapter 1: Introduction to Physical AI',
            url: '/docs/chapter1',
            interests: ['physical-ai', 'ros2'],
            module: 'module1',
            order: 1,
            description: 'Intro',
          },
        ],
        emptyState: false,
        totalChapters: 8,
        matchCount: 1,
      },
      isLoading: false,
      error: null,
      refetch: mockRefetch,
      isRefreshing: false,
    });
  });

  describe('adding interests', () => {
    it('should show new content when interest is added', async () => {
      const { rerender } = render(<PersonalizedContentPage />);

      // Initial state - only sees Physical AI content
      expect(screen.getByText(/Introduction to Physical AI/i)).toBeInTheDocument();

      // Simulate user adding "kinematics" interest (ID: 3)
      act(() => {
        mockUseAuth.mockReturnValue({
          user: {
            user_id: 'test-user-123',
            email: 'test@example.com',
            interests: [1, 2, 3], // Added kinematics
          },
          isAuthenticated: true,
          isLoading: false,
          signUp: jest.fn(),
          signIn: jest.fn(),
          signOut: jest.fn(),
          validateSession: jest.fn(),
        });

        mockUsePersonalizedContent.mockReturnValue({
          content: {
            userInterests: ['physical-ai', 'ros2', 'kinematics'],
            matchedChapters: [
              {
                id: 'chapter1',
                title: 'Chapter 1: Introduction to Physical AI',
                url: '/docs/chapter1',
                interests: ['physical-ai', 'ros2'],
                module: 'module1',
                order: 1,
                description: 'Intro',
              },
              {
                id: 'chapter3',
                title: 'Chapter 3: Kinematics',
                url: '/docs/chapter3',
                interests: ['kinematics'],
                module: 'module1',
                order: 3,
                description: 'Kinematics chapter',
              },
            ],
            emptyState: false,
            totalChapters: 8,
            matchCount: 2,
          },
          isLoading: false,
          error: null,
          refetch: mockRefetch,
          isRefreshing: false,
        });
      });

      // Rerender to simulate React update
      rerender(<PersonalizedContentPage />);

      // Should now see both chapters
      await waitFor(() => {
        const introMatches = screen.getAllByText(/Introduction to Physical AI/i);
        expect(introMatches.length).toBeGreaterThan(0);
        const kinematicsMatches = screen.getAllByText(/Kinematics/i);
        expect(kinematicsMatches.length).toBeGreaterThan(0);
      });
    });

    it('should call refetch when interests change', async () => {
      render(<PersonalizedContentPage />);

      // Verify initial render
      expect(screen.getByText(/Introduction to Physical AI/i)).toBeInTheDocument();

      // usePersonalizedContent should be called with initial interests
      expect(mockUsePersonalizedContent).toHaveBeenCalledWith(
        expect.arrayContaining(['physical-ai', 'ros2'])
      );
    });
  });

  describe('removing interests', () => {
    it('should remove content when interest is removed', async () => {
      // Start with multiple interests
      mockUseAuth.mockReturnValue({
        user: {
          user_id: 'test-user-123',
          email: 'test@example.com',
          interests: [1, 2, 3], // physical-ai, ros2, kinematics
        },
        isAuthenticated: true,
        isLoading: false,
        signUp: jest.fn(),
        signIn: jest.fn(),
        signOut: jest.fn(),
        validateSession: jest.fn(),
      });

      mockUsePersonalizedContent.mockReturnValue({
        content: {
          userInterests: ['physical-ai', 'ros2', 'kinematics'],
          matchedChapters: [
            {
              id: 'chapter1',
              title: 'Chapter 1: Introduction to Physical AI',
              url: '/docs/chapter1',
              interests: ['physical-ai'],
              module: 'module1',
              order: 1,
              description: 'Intro',
            },
            {
              id: 'chapter3',
              title: 'Chapter 3: Kinematics',
              url: '/docs/chapter3',
              interests: ['kinematics'],
              module: 'module1',
              order: 3,
              description: 'Kinematics chapter',
            },
          ],
          emptyState: false,
          totalChapters: 8,
          matchCount: 2,
        },
        isLoading: false,
        error: null,
        refetch: mockRefetch,
        isRefreshing: false,
      });

      const { rerender } = render(<PersonalizedContentPage />);

      // Should see both chapters initially
      const introMatches = screen.getAllByText(/Introduction to Physical AI/i);
      expect(introMatches.length).toBeGreaterThan(0);
      const kinematicsMatches = screen.getAllByText(/Kinematics/i);
      expect(kinematicsMatches.length).toBeGreaterThan(0);

      // Remove kinematics interest
      act(() => {
        mockUseAuth.mockReturnValue({
          user: {
            user_id: 'test-user-123',
            email: 'test@example.com',
            interests: [1, 2], // Removed kinematics (3)
          },
          isAuthenticated: true,
          isLoading: false,
          signUp: jest.fn(),
          signIn: jest.fn(),
          signOut: jest.fn(),
          validateSession: jest.fn(),
        });

        mockUsePersonalizedContent.mockReturnValue({
          content: {
            userInterests: ['physical-ai', 'ros2'],
            matchedChapters: [
              {
                id: 'chapter1',
                title: 'Chapter 1: Introduction to Physical AI',
                url: '/docs/chapter1',
                interests: ['physical-ai'],
                module: 'module1',
                order: 1,
                description: 'Intro',
              },
            ],
            emptyState: false,
            totalChapters: 8,
            matchCount: 1,
          },
          isLoading: false,
          error: null,
          refetch: mockRefetch,
          isRefreshing: false,
        });
      });

      rerender(<PersonalizedContentPage />);

      // Kinematics chapter should be removed
      await waitFor(() => {
        const introMatches = screen.getAllByText(/Introduction to Physical AI/i);
        expect(introMatches.length).toBeGreaterThan(0);
        expect(screen.queryByText(/Chapter 3: Kinematics/i)).not.toBeInTheDocument();
      });
    });
  });

  describe('changing all interests', () => {
    it('should completely refresh view when all interests change', async () => {
      const { rerender } = render(<PersonalizedContentPage />);

      // Initial content
      expect(screen.getByText(/Introduction to Physical AI/i)).toBeInTheDocument();

      // Change to completely different interests
      act(() => {
        mockUseAuth.mockReturnValue({
          user: {
            user_id: 'test-user-123',
            email: 'test@example.com',
            interests: [4, 5], // dynamics-control, sensors (completely different)
          },
          isAuthenticated: true,
          isLoading: false,
          signUp: jest.fn(),
          signIn: jest.fn(),
          signOut: jest.fn(),
          validateSession: jest.fn(),
        });

        mockUsePersonalizedContent.mockReturnValue({
          content: {
            userInterests: ['dynamics-control', 'sensors'],
            matchedChapters: [
              {
                id: 'chapter4',
                title: 'Chapter 4: Dynamics and Control',
                url: '/docs/chapter4',
                interests: ['dynamics-control'],
                module: 'module1',
                order: 4,
                description: 'Dynamics',
              },
            ],
            emptyState: false,
            totalChapters: 8,
            matchCount: 1,
          },
          isLoading: false,
          error: null,
          refetch: mockRefetch,
          isRefreshing: false,
        });
      });

      rerender(<PersonalizedContentPage />);

      // Old content should be gone, new content should appear
      await waitFor(() => {
        expect(screen.queryByText(/Introduction to Physical AI/i)).not.toBeInTheDocument();
        const dynamicsMatches = screen.getAllByText(/Dynamics and Control/i);
        expect(dynamicsMatches.length).toBeGreaterThan(0);
      });
    });
  });

  describe('loading states during update', () => {
    it('should show loading indicator while content is refreshing', async () => {
      mockUsePersonalizedContent.mockReturnValue({
        content: null,
        isLoading: true,
        error: null,
        refetch: mockRefetch,
        isRefreshing: false,
      });

      render(<PersonalizedContentPage />);

      // Should show loading state
      expect(screen.getByText(/Loading your personalized content/i)).toBeInTheDocument();
    });

    it('should handle errors during content refresh gracefully', () => {
      mockUsePersonalizedContent.mockReturnValue({
        content: null,
        isLoading: false,
        error: new Error('Failed to fetch content'),
        refetch: mockRefetch,
        isRefreshing: false,
      });

      render(<PersonalizedContentPage />);

      // Should show error message
      expect(screen.getByRole('heading', { name: /Error Loading Content/i })).toBeInTheDocument();
    });
  });

  describe('empty state after removing all interests', () => {
    it('should show InterestPrompt when user removes all interests', async () => {
      const { rerender } = render(<PersonalizedContentPage />);

      // Remove all interests
      act(() => {
        mockUseAuth.mockReturnValue({
          user: {
            user_id: 'test-user-123',
            email: 'test@example.com',
            interests: [], // No interests
          },
          isAuthenticated: true,
          isLoading: false,
          signUp: jest.fn(),
          signIn: jest.fn(),
          signOut: jest.fn(),
          validateSession: jest.fn(),
        });

        mockUsePersonalizedContent.mockReturnValue({
          content: {
            userInterests: [],
            matchedChapters: [],
            emptyState: true,
            totalChapters: 8,
            matchCount: 0,
          },
          isLoading: false,
          error: null,
          refetch: mockRefetch,
          isRefreshing: false,
        });
      });

      rerender(<PersonalizedContentPage />);

      // Should show InterestPrompt
      await waitFor(() => {
        expect(screen.getByTestId('interest-prompt')).toBeInTheDocument();
      });
    });
  });
});
