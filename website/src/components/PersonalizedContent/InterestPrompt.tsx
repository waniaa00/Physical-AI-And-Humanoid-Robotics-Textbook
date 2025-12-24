/**
 * InterestPrompt Component (T032)
 * Empty state component displayed when user has no selected interests
 *
 * Features:
 * - Friendly, welcoming message
 * - "Select Interests" CTA linking to profile page
 * - "Browse All Content" alternative action
 * - Visually appealing centered layout with icons
 */

import React from 'react';
import Link from '@docusaurus/Link';
import styles from './InterestPrompt.module.css';

export default function InterestPrompt(): React.JSX.Element {
  return (
    <div className={styles.container} data-testid="interest-prompt">
      <div className={styles.content}>
        {/* Icon/Emoji for visual appeal */}
        <div className={styles.icon} aria-hidden="true">
          🎯
        </div>

        {/* Heading */}
        <h2 className={styles.heading}>Select Your Interests</h2>

        {/* Description */}
        <p className={styles.description}>
          Tell us what you're interested in to personalize your learning experience.
          We'll show you the most relevant chapters and content based on your preferences.
        </p>

        {/* Action buttons */}
        <div className={styles.actions}>
          {/* Primary CTA - Select Interests */}
          <Link to="/profile" className={`${styles.button} ${styles.buttonPrimary}`}>
            ✨ Select Interests
          </Link>

          {/* Secondary action - Browse All */}
          <Link to="/docs/intro" className={`${styles.button} ${styles.buttonSecondary}`}>
            📚 Browse All Content
          </Link>
        </div>

        {/* Additional helpful text */}
        <p className={styles.helpText}>
          You can update your interests anytime from your profile page.
        </p>
      </div>
    </div>
  );
}
