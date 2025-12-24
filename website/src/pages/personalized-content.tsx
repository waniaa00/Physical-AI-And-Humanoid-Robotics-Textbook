/**
 * Personalized Content Page (T021, T022, T030, T034, T038, T040)
 * Main page displaying personalized content based on user interests
 *
 * Features:
 * - Auth guard to redirect unauthenticated users (T022, T030)
 * - Sign-out detection and redirect (T030)
 * - Empty interests handling (T034) - delegates to PersonalizedContentView
 *   which displays InterestPrompt when user has no interests selected
 * - Automatic content refresh when interests change (T038)
 * - Visual feedback during refresh (T040)
 */

import React, { useEffect, useRef, useState } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '@site/src/hooks/useAuth';
import { usePersonalizedContent } from '@site/src/hooks/usePersonalizedContent';
import { mapBackendToFrontendInterests } from '@site/src/utils/interestMapper';
import { PersonalizedContentView } from '@site/src/components/PersonalizedContent';
import { sanitizeRedirectUrl } from '@site/src/utils/urlValidator';

export default function PersonalizedContentPage(): React.JSX.Element {
  const { user, isAuthenticated, isLoading: authLoading } = useAuth();

  // Track previous authentication state to detect sign-out (T030)
  const wasAuthenticatedRef = useRef(false);

  // T022, T030: Auth guard with sign-out detection
  useEffect(() => {
    // Skip during initial loading
    if (authLoading) return;

    // User is authenticated - update tracking
    if (isAuthenticated) {
      wasAuthenticatedRef.current = true;
      return;
    }

    // User is NOT authenticated
    if (!isAuthenticated) {
      // Check if user was previously authenticated (sign-out scenario)
      if (wasAuthenticatedRef.current) {
        // T030: User signed out while on this page - redirect to homepage
        console.log('[PersonalizedContent] Sign-out detected, redirecting to homepage');
        const safeRedirect = sanitizeRedirectUrl('/', '/');
        window.location.href = safeRedirect;
        return;
      }

      // User was never authenticated (direct access or session expired)
      // Redirect to signin with return URL (validated)
      console.log('[PersonalizedContent] Unauthenticated access, redirecting to signin');
      const returnUrl = sanitizeRedirectUrl('/personalized-content', '/');
      window.location.href = `/signin?returnTo=${encodeURIComponent(returnUrl)}`;
    }
  }, [isAuthenticated, authLoading]);

  // T034: Map backend interest IDs to frontend interest strings
  // If user has no interests (empty array), PersonalizedContentView will show InterestPrompt
  const frontendInterests = mapBackendToFrontendInterests(user?.interests || []);

  // T038, T040: Get personalized content with refresh detection
  const { content, isLoading: contentLoading, error, isRefreshing } = usePersonalizedContent(frontendInterests);

  // T040: Show refresh notification
  const [showRefreshNotification, setShowRefreshNotification] = useState(false);

  // T038, T040: Display notification when content refreshes
  useEffect(() => {
    if (isRefreshing) {
      setShowRefreshNotification(true);
      const timer = setTimeout(() => {
        setShowRefreshNotification(false);
      }, 2000);
      return () => clearTimeout(timer);
    }
  }, [isRefreshing]);

  // Show loading state while checking auth
  if (authLoading) {
    return (
      <Layout title="Personalized Content" description="Your personalized learning path">
        <div style={{ textAlign: 'center', padding: '4rem 2rem' }}>
          <p>Loading...</p>
        </div>
      </Layout>
    );
  }

  // Don't render anything if not authenticated (will redirect)
  if (!isAuthenticated) {
    return (
      <Layout title="Personalized Content" description="Your personalized learning path">
        <div style={{ textAlign: 'center', padding: '4rem 2rem' }}>
          <p>Redirecting to sign in...</p>
        </div>
      </Layout>
    );
  }

  // Show loading state while fetching content
  if (contentLoading) {
    return (
      <Layout title="Personalized Content" description="Your personalized learning path">
        <div style={{ textAlign: 'center', padding: '4rem 2rem' }}>
          <p>Loading your personalized content...</p>
        </div>
      </Layout>
    );
  }

  // Show error state if content fetch failed
  if (error) {
    return (
      <Layout title="Personalized Content" description="Your personalized learning path">
        <div style={{ textAlign: 'center', padding: '4rem 2rem' }}>
          <h2>Error Loading Content</h2>
          <p>We encountered an error while loading your personalized content. Please try again later.</p>
          <p style={{ color: '#dc3545', fontSize: '0.9rem' }}>{error.message}</p>
        </div>
      </Layout>
    );
  }

  // T034, T038, T040: Render personalized content with refresh indicator
  // PersonalizedContentView handles empty interests by showing InterestPrompt component
  return (
    <Layout
      title="Your Personalized Content"
      description="Chapters and sections tailored to your interests"
    >
      {/* T040: Refresh notification banner */}
      {showRefreshNotification && (
        <div
          style={{
            position: 'fixed',
            top: '20px',
            right: '20px',
            backgroundColor: '#4CAF50',
            color: 'white',
            padding: '12px 24px',
            borderRadius: '8px',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)',
            zIndex: 1000,
            fontWeight: 500,
            animation: 'slideIn 0.3s ease-out',
          }}
        >
          ✨ Content updated!
        </div>
      )}

      {/* T040: Show subtle loading overlay during refresh */}
      {isRefreshing && (
        <div
          style={{
            position: 'fixed',
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            backgroundColor: 'rgba(0, 0, 0, 0.05)',
            zIndex: 999,
            pointerEvents: 'none',
          }}
        />
      )}

      <PersonalizedContentView content={content} />

      {/* T040: CSS animations */}
      <style>{`
        @keyframes slideIn {
          from {
            transform: translateX(100%);
            opacity: 0;
          }
          to {
            transform: translateX(0);
            opacity: 1;
          }
        }
      `}</style>
    </Layout>
  );
}
