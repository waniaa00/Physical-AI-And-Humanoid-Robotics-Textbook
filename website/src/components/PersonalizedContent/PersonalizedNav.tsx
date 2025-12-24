/**
 * PersonalizedNav Component (T019)
 * Navigation links for personalized content page
 */

import React from 'react';
import Link from '@docusaurus/Link';
import styles from './PersonalizedNav.module.css';

export default function PersonalizedNav(): React.JSX.Element {
  return (
    <nav className={styles.nav} role="navigation" aria-label="Personalized content navigation">
      <div className={styles.navContainer}>
        <Link to="/docs/intro" className={styles.navLink}>
          📖 View Full Book
        </Link>
        <Link to="/profile" className={styles.navLink}>
          ⚙️ Update Interests
        </Link>
      </div>
    </nav>
  );
}
