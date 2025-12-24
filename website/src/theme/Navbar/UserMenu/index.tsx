/**
 * User Menu Component for Navbar
 * Shows user profile and logout button when authenticated
 */

import React, { useState, useRef, useEffect } from 'react';
import { useAuth } from '@site/src/hooks/useAuth';
import styles from './styles.module.css';

export default function UserMenu(): React.JSX.Element | null {
  const { user, isAuthenticated, signOut } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const menuRef = useRef<HTMLDivElement>(null);

  // Close menu when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    };

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => document.removeEventListener('mousedown', handleClickOutside);
    }
  }, [isOpen]);

  const handleSignOut = async () => {
    try {
      await signOut();
      window.location.href = '/';
    } catch (error) {
      console.error('Sign out error:', error);
    }
  };

  // Don't show menu if not authenticated
  if (!isAuthenticated || !user) {
    return (
      <div className={styles.authLinks}>
        <a href="/signin" className={styles.signInLink}>Sign In</a>
        <a href="/signup" className={styles.signUpLink}>Sign Up</a>
      </div>
    );
  }

  // Extract first letter of email for avatar
  const avatarLetter = user.email ? user.email[0].toUpperCase() : 'U';

  return (
    <div className={styles.userMenu} ref={menuRef}>
      <button
        className={styles.userButton}
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-haspopup="true"
      >
        <div className={styles.avatar}>{avatarLetter}</div>
        <span className={styles.email}>{user.email}</span>
        <svg
          className={styles.chevron}
          width="12"
          height="12"
          viewBox="0 0 12 12"
          fill="currentColor"
        >
          <path d="M6 8L2 4h8z" />
        </svg>
      </button>

      {isOpen && (
        <div className={styles.dropdown}>
          <div className={styles.userInfo}>
            <div className={styles.avatarLarge}>{avatarLetter}</div>
            <div className={styles.userDetails}>
              <div className={styles.userName}>{user.email}</div>
              <div className={styles.userInterests}>
                {user.interests.length} {user.interests.length === 1 ? 'interest' : 'interests'}
              </div>
            </div>
          </div>

          <div className={styles.divider} />

          <a href="/profile" className={styles.menuItem}>
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M8 8a3 3 0 100-6 3 3 0 000 6zm0 2c-2.67 0-8 1.34-8 4v2h16v-2c0-2.66-5.33-4-8-4z" />
            </svg>
            Profile & Interests
          </a>

          <a href="/personalized-content" className={styles.menuItem}>
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M3 2h10a1 1 0 011 1v10a1 1 0 01-1 1H3a1 1 0 01-1-1V3a1 1 0 011-1zm2 2v8h6V4H5z" />
            </svg>
            My Content
          </a>

          <div className={styles.divider} />

          <button onClick={handleSignOut} className={styles.menuItem}>
            <svg width="16" height="16" viewBox="0 0 16 16" fill="currentColor">
              <path d="M3 3h6v2H5v6h4v2H3a1 1 0 01-1-1V4a1 1 0 011-1zm9 4l3 3-3 3v-2H7V6h5V4z" />
            </svg>
            Sign Out
          </button>
        </div>
      )}
    </div>
  );
}
