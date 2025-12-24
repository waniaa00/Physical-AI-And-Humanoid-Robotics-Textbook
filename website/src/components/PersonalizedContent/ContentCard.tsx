/**
 * ContentCard Component (T018, T054)
 * Displays a single chapter as a card with title, description, and interest badges
 * Performance: Memoized to prevent unnecessary re-renders
 */

import React, { useMemo } from 'react';
import Link from '@docusaurus/Link';
import type { ChapterMetadata } from '@site/src/types/personalization';
import { getInterestLabels } from '@site/src/utils/interestMapper';
import styles from './ContentCard.module.css';

interface ContentCardProps {
  chapter: ChapterMetadata;
}

function ContentCard({ chapter }: ContentCardProps): React.JSX.Element {
  const { id, title, url, interests, description } = chapter;

  // T054: Memoize interest label computation
  const interestLabels = useMemo(() => getInterestLabels(interests), [interests]);

  return (
    <article
      className={styles.card}
      data-chapter-id={id}
      aria-label={`${title} chapter`}
    >
      <Link
        to={url}
        className={styles.cardLink}
        aria-label={`Read ${title}`}
      >
        <h3 className={styles.title}>{title}</h3>
      </Link>

      {description && (
        <p className={styles.description}>{description}</p>
      )}

      {interests && interests.length > 0 && (
        <div
          className={styles.badgesContainer}
          role="list"
          aria-label="Chapter topics"
        >
          {interestLabels.map((label, index) => (
            <span
              key={`${id}-${interests[index]}`}
              className={styles.badge}
              role="listitem"
              aria-label={`Topic: ${label}`}
            >
              {label}
            </span>
          ))}
        </div>
      )}
    </article>
  );
}

// T054: Memoize component to prevent unnecessary re-renders
export default React.memo(ContentCard);
