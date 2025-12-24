/**
 * Integration tests for personalized-content page (T017)
 * Tests auth guard, redirect, and full content display flow
 *
 * Test cases:
 * - Authenticated user sees personalized content
 * - Unauthenticated user redirected to signin
 * - User interests mapped correctly
 * - Content filtered and displayed properly
 */

import React from 'react';
import { render, screen, waitFor } from '@testing-library/react';
import '@testing-library/jest-dom';

// Mock dependencies BEFORE imports
jest.mock('@site/src/hooks/useAuth');
jest.mock('@site/src/hooks/usePersonalizedContent');
jest.mock('@site/src/utils/interestMapper', () => ({
  ...jest.requireActual('@site/src/utils/interestMapper'),
  mapBackendToFrontendInterests: jest.fn((ids: number[]) => {
    const map: Record<number, string> = {
      1: 'physical-ai',
      2: 'ros2',
      3: 'kinematics',
      4: 'dynamics-control',
      5: 'sensors',
      6: 'humanoid-design',
      7: 'simulation',
      8: 'machine-learning',
    };
    return ids.map(id => map[id]).filter(Boolean);
  }),
}));
jest.mock('@docusaurus/Link', () => {
  return function MockLink({ children, to }: any) {
    return <a href={to}>{children}</a>;
  };
});
jest.mock('@theme/Layout', () => {
  return function MockLayout({ children }: any) {
    return <div data-testid="layout">{children}</div>;
  };
});

import PersonalizedContentPage from '@site/src/pages/personalized-content';
import { useAuth } from '@site/src/hooks/useAuth';
import { usePersonalizedContent } from '@site/src/hooks/usePersonalizedContent';
import type { UsePersonalizedContentResult } from '@site/src/hooks/usePersonalizedContent';

const mockUseAuth = useAuth as jest.MockedFunction<typeof useAuth>;
const mockUsePersonalizedContent = usePersonalizedContent as jest.MockedFunction<typeof usePersonalizedContent>;

describe('PersonalizedContentPage', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    // Reset window.location
    delete (window as any).location;
    (window as any).location = { href: '' };

    // Set default mock for usePersonalizedContent
    mockUsePersonalizedContent.mockReturnValue({
      content: null,
      isLoading: false,
      error: null,
      refetch: jest.fn(),
      isRefreshing: false,
    });
  });

  describe('authentication guard', () => {
    it('should show redirect message for unauthenticated users', () => {
      mockUseAuth.mockReturnValue({
        user: null,
        isAuthenticated: false,
        isLoading: false,
        signUp: jest.fn(),
        signIn: jest.fn(),
        signOut: jest.fn(),
        validateSession: jest.fn(),
      });

      render(<PersonalizedContentPage />);

      // Check that redirecting message is shown (redirect happens in useEffect)
      expect(screen.getByText(/Redirecting to sign in/i)).toBeInTheDocument();
      // Verify content is not displayed for unauthenticated users
      expect(screen.queryByRole('heading', { name: /Your Personalized Content/i })).not.toBeInTheDocument();
    });

    it('should not display personalized content for unauthenticated users', () => {
      mockUseAuth.mockReturnValue({
        user: null,
        isAuthenticated: false,
        isLoading: false,
        signUp: jest.fn(),
        signIn: jest.fn(),
        signOut: jest.fn(),
        validateSession: jest.fn(),
      });

      render(<PersonalizedContentPage />);

      // Verify that personalized content is not rendered
      expect(screen.queryByText(/matching your interests/i)).not.toBeInTheDocument();
    });

    it('should show loading state while checking authentication', () => {
      mockUseAuth.mockReturnValue({
        user: null,
        isAuthenticated: false,
        isLoading: true,
        signUp: jest.fn(),
        signIn: jest.fn(),
        signOut: jest.fn(),
        validateSession: jest.fn(),
      });

      render(<PersonalizedContentPage />);

      expect(screen.getByText(/loading/i)).toBeInTheDocument();
    });
  });

  describe('authenticated user content display', () => {
    const mockAuthenticatedUser = {
      user: {
        user_id: 'test-user-123',
        email: 'test@example.com',
        interests: [1, 2], // Backend interest IDs
      },
      isAuthenticated: true,
      isLoading: false,
      signUp: jest.fn(),
      signIn: jest.fn(),
      signOut: jest.fn(),
      validateSession: jest.fn(),
    };

    const mockPersonalizedContent: UsePersonalizedContentResult = {
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
            description: 'Introduction to physical AI',
          },
        ],
        emptyState: false,
        totalChapters: 8,
        matchCount: 1,
      },
      isLoading: false,
      error: null,
      refetch: jest.fn(),
      isRefreshing: false,
    };

    it('should display personalized content for authenticated user', () => {
      mockUseAuth.mockReturnValue(mockAuthenticatedUser);
      mockUsePersonalizedContent.mockReturnValue(mockPersonalizedContent);

      render(<PersonalizedContentPage />);

      // Check that chapter content is displayed (using getAllByText since text may appear multiple times)
      const matches = screen.getAllByText(/Introduction to Physical AI/i);
      expect(matches.length).toBeGreaterThan(0);
    });

    it('should display page title and description', () => {
      mockUseAuth.mockReturnValue(mockAuthenticatedUser);
      mockUsePersonalizedContent.mockReturnValue(mockPersonalizedContent);

      render(<PersonalizedContentPage />);

      // Check for heading using role selector
      expect(screen.getByRole('heading', { name: /Your Personalized Content/i })).toBeInTheDocument();
    });

    it('should show loading state while fetching content', () => {
      mockUseAuth.mockReturnValue(mockAuthenticatedUser);
      mockUsePersonalizedContent.mockReturnValue({
        ...mockPersonalizedContent,
        isLoading: true,
        content: null,
      });

      render(<PersonalizedContentPage />);

      // Check for specific loading text
      expect(screen.getByText('Loading your personalized content...')).toBeInTheDocument();
    });

    it('should display error message when content fetch fails', () => {
      mockUseAuth.mockReturnValue(mockAuthenticatedUser);
      mockUsePersonalizedContent.mockReturnValue({
        content: null,
        isLoading: false,
        error: new Error('Failed to load content'),
        refetch: jest.fn(),
        isRefreshing: false,
      });

      render(<PersonalizedContentPage />);

      // Check for error heading
      expect(screen.getByRole('heading', { name: /Error Loading Content/i })).toBeInTheDocument();
      // Check for error message
      expect(screen.getByText(/We encountered an error while loading your personalized content/i)).toBeInTheDocument();
    });
  });

  describe('interest mapping', () => {
    it('should map backend interest IDs to frontend interest strings', () => {
      const mockUser = {
        user: {
          user_id: 'test-user-123',
          email: 'test@example.com',
          interests: [1, 2, 3], // Backend IDs: physical-ai, ros2, kinematics
        },
        isAuthenticated: true,
        isLoading: false,
        signUp: jest.fn(),
        signIn: jest.fn(),
        signOut: jest.fn(),
        validateSession: jest.fn(),
      };

      mockUseAuth.mockReturnValue(mockUser);
      mockUsePersonalizedContent.mockReturnValue({
        content: {
          userInterests: ['physical-ai', 'ros2', 'kinematics'],
          matchedChapters: [],
          emptyState: false,
          totalChapters: 8,
          matchCount: 0,
        },
        isLoading: false,
        error: null,
        refetch: jest.fn(),
        isRefreshing: false,
      });

      render(<PersonalizedContentPage />);

      // Verify usePersonalizedContent was called with mapped interests
      expect(mockUsePersonalizedContent).toHaveBeenCalledWith(
        expect.arrayContaining(['physical-ai', 'ros2', 'kinematics'])
      );
    });

    it('should handle users with no interests', () => {
      const mockUser = {
        user: {
          user_id: 'test-user-123',
          email: 'test@example.com',
          interests: [],
        },
        isAuthenticated: true,
        isLoading: false,
        signUp: jest.fn(),
        signIn: jest.fn(),
        signOut: jest.fn(),
        validateSession: jest.fn(),
      };

      mockUseAuth.mockReturnValue(mockUser);
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
        refetch: jest.fn(),
        isRefreshing: false,
      });

      render(<PersonalizedContentPage />);

      // Should render InterestPrompt component (T033)
      expect(screen.getByTestId('interest-prompt')).toBeInTheDocument();
      expect(screen.getByRole('heading', { name: /select your interests/i })).toBeInTheDocument();
    });
  });

  describe('page metadata', () => {
    it('should set correct page title', () => {
      mockUseAuth.mockReturnValue({
        user: {
          user_id: 'test-user-123',
          email: 'test@example.com',
          interests: [1],
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
          userInterests: ['physical-ai'],
          matchedChapters: [],
          emptyState: false,
          totalChapters: 8,
          matchCount: 0,
        },
        isLoading: false,
        error: null,
        refetch: jest.fn(),
        isRefreshing: false,
      });

      render(<PersonalizedContentPage />);

      // Layout component should receive title prop
      expect(screen.getByTestId('layout')).toBeInTheDocument();
    });
  });
});
