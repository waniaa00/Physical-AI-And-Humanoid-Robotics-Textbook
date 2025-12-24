/**
 * InterestCard Component
 *
 * Displays an individual interest category as a selectable card.
 * Used within InterestSelector for interest selection UI.
 */

import React from 'react';
import styles from './InterestCard.module.css';

export interface InterestCategory {
  id: number;
  name: string;
  slug: string;
  description: string;
}

interface InterestCardProps {
  /** Interest category data */
  interest: InterestCategory;
  /** Whether this interest is currently selected */
  isSelected: boolean;
  /** Callback when card is clicked */
  onToggle: (interestId: number) => void;
  /** Optional className for styling */
  className?: string;
  /** Disable selection (e.g., when max selections reached) */
  disabled?: boolean;
}

export const InterestCard: React.FC<InterestCardProps> = ({
  interest,
  isSelected,
  onToggle,
  className = '',
  disabled = false,
}) => {
  const handleClick = () => {
    if (!disabled || isSelected) {
      // Allow deselection even when disabled
      onToggle(interest.id);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' || e.key === ' ') {
      e.preventDefault();
      handleClick();
    }
  };

  return (
    <div
      className={`${styles.interestCard} ${isSelected ? styles.selected : ''} ${
        disabled && !isSelected ? styles.disabled : ''
      } ${className}`}
      onClick={handleClick}
      onKeyPress={handleKeyPress}
      role="checkbox"
      aria-checked={isSelected}
      aria-disabled={disabled && !isSelected}
      aria-label={`${interest.name}: ${interest.description}`}
      tabIndex={0}
    >
      <div className={styles.cardHeader}>
        <div className={styles.checkbox}>
          {isSelected && (
            <svg
              className={styles.checkIcon}
              width="16"
              height="16"
              viewBox="0 0 24 24"
              fill="none"
              stroke="currentColor"
              strokeWidth="3"
            >
              <polyline points="20 6 9 17 4 12" />
            </svg>
          )}
        </div>
        <h4 className={styles.title}>{interest.name}</h4>
      </div>

      <p className={styles.description}>{interest.description}</p>
    </div>
  );
};
