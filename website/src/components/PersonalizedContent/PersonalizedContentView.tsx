/**
 * PersonalizedContentView Component (T020, T033)
 * Main view component rendering personalized content with chapter cards and navigation
 * Displays InterestPrompt when user has no interests selected
 */

import React from 'react';
import type { PersonalizedContentView as PersonalizedContentViewType } from '@site/src/types/personalization';
import ContentCard from './ContentCard';
import PersonalizedNav from './PersonalizedNav';
import InterestPrompt from './InterestPrompt';
import styles from './PersonalizedContentView.module.css';

interface PersonalizedContentViewProps {
  content: PersonalizedContentViewType | null;
}

export default function PersonalizedContentView({ content }: PersonalizedContentViewProps): React.JSX.Element {
  // Loading state
  if (!content) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>Loading your personalized content...</div>
      </div>
    );
  }

  const { matchedChapters, emptyState, totalChapters, matchCount, userInterests } = content;

  // T033: Empty state when user has NO interests - show InterestPrompt
  if (emptyState && userInterests.length === 0) {
    return <InterestPrompt />;
  }

  // Empty state - user HAS interests but no chapters match
  if (emptyState) {
    return (
      <div className={styles.container}>
        <div className={styles.emptyState}>
          <h2>No chapters found</h2>
          <p>
            No chapters match your current interests. Try updating your interests or browse the full book.
          </p>
          <PersonalizedNav />
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <header className={styles.header}>
        <h1>Your Personalized Content</h1>
        <p className={styles.subtitle}>
          Showing {matchCount} {matchCount === 1 ? 'chapter' : 'chapters'} matching your interests
          {totalChapters > 0 && ` (${totalChapters} total)`}
        </p>
      </header>

      <PersonalizedNav />

      <div className={styles.chaptersGrid}>
        {matchedChapters.map((chapter) => (
          <ContentCard key={chapter.id} chapter={chapter} />
        ))}
      </div>

      {matchedChapters.length > 0 && (
        <footer className={styles.footer}>
          <p>
            Found what you're looking for? <a href="/docs/intro">Browse the full book</a> for more content.
          </p>
        </footer>
      )}
    </div>
  );
}
