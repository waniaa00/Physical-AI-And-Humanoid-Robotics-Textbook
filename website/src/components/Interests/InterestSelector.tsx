/**
 * InterestSelector Component
 *
 * Allows users to select 2-5 interests from available categories.
 * Used in sign-up/sign-in flows and profile settings.
 */

import React, { useState, useEffect } from 'react';
import { InterestCard, InterestCategory } from './InterestCard';
import styles from './InterestSelector.module.css';

interface InterestSelectorProps {
  /** Pre-selected interest IDs (for edit mode) */
  initialSelectedIds?: number[];
  /** Callback when selection changes */
  onSelectionChange: (selectedIds: number[]) => void;
  /** Optional className for styling */
  className?: string;
  /** Show selection counter */
  showCounter?: boolean;
  /** Allow proceeding without validation (for optional flows) */
  allowSkip?: boolean;
}

export const InterestSelector: React.FC<InterestSelectorProps> = ({
  initialSelectedIds = [],
  onSelectionChange,
  className = '',
  showCounter = true,
  allowSkip = false,
}) => {
  const [categories, setCategories] = useState<InterestCategory[]>([]);
  const [selectedIds, setSelectedIds] = useState<number[]>(initialSelectedIds);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  const MIN_INTERESTS = 2;
  const MAX_INTERESTS = 5;

  // Fetch interest categories on mount
  useEffect(() => {
    fetchCategories();
  }, []);

  // Notify parent of selection changes
  useEffect(() => {
    onSelectionChange(selectedIds);
  }, [selectedIds, onSelectionChange]);

  const fetchCategories = async () => {
    setIsLoading(true);
    setError(null);

    try {
      const apiUrl = 'https://wnxddev-humanoid-robotics-api.hf.space';
      const response = await fetch(`${apiUrl}/interests/categories/all`);

      if (!response.ok) {
        throw new Error('Failed to fetch interest categories');
      }

      const data = await response.json();
      setCategories(data.categories || []);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Failed to load interests';
      setError(errorMessage);
      console.error('Error fetching categories:', err);
    } finally {
      setIsLoading(false);
    }
  };

  const handleToggleInterest = (interestId: number) => {
    setSelectedIds((prev) => {
      if (prev.includes(interestId)) {
        // Deselect
        return prev.filter((id) => id !== interestId);
      } else {
        // Select (if under max limit)
        if (prev.length < MAX_INTERESTS) {
          return [...prev, interestId];
        }
        return prev;
      }
    });
  };

  const isMaxSelected = selectedIds.length >= MAX_INTERESTS;
  const isMinSelected = selectedIds.length >= MIN_INTERESTS;
  const isValid = isMinSelected || allowSkip;

  return (
    <div className={`${styles.interestSelectorContainer} ${className}`}>
      {/* Header */}
      <div className={styles.header}>
        <h3 className={styles.title}>
          Select Your Interests
        </h3>
        <p className={styles.subtitle}>
          Choose {MIN_INTERESTS}-{MAX_INTERESTS} areas that interest you. This helps us personalize your
          experience.
        </p>
      </div>

      {/* Selection Counter */}
      {showCounter && (
        <div className={styles.counter}>
          <div className={styles.counterBadge}>
            <span className={styles.counterNumber}>{selectedIds.length}</span>
            <span className={styles.counterText}>
              / {MAX_INTERESTS} selected
            </span>
          </div>
          <div className={styles.validationText}>
            {selectedIds.length < MIN_INTERESTS && (
              <span className={styles.warning}>
                Select at least {MIN_INTERESTS - selectedIds.length} more{' '}
                {MIN_INTERESTS - selectedIds.length === 1 ? 'interest' : 'interests'}
              </span>
            )}
            {selectedIds.length >= MIN_INTERESTS && selectedIds.length < MAX_INTERESTS && (
              <span className={styles.success}>
                ✓ You can select {MAX_INTERESTS - selectedIds.length} more if you'd like
              </span>
            )}
            {selectedIds.length >= MAX_INTERESTS && (
              <span className={styles.info}>Maximum selections reached</span>
            )}
          </div>
        </div>
      )}

      {/* Loading State */}
      {isLoading && (
        <div className={styles.loadingState}>
          <div className={styles.spinner}></div>
          <p>Loading interests...</p>
        </div>
      )}

      {/* Error State */}
      {error && (
        <div className={styles.errorState} role="alert">
          <p className={styles.errorMessage}>{error}</p>
          <button onClick={fetchCategories} className={styles.retryButton}>
            Retry
          </button>
        </div>
      )}

      {/* Interest Grid */}
      {!isLoading && !error && (
        <div className={styles.interestGrid}>
          {categories.map((category) => (
            <InterestCard
              key={category.id}
              interest={category}
              isSelected={selectedIds.includes(category.id)}
              onToggle={handleToggleInterest}
              disabled={isMaxSelected && !selectedIds.includes(category.id)}
            />
          ))}
        </div>
      )}

      {/* Empty State */}
      {!isLoading && !error && categories.length === 0 && (
        <div className={styles.emptyState}>
          <p>No interest categories available.</p>
        </div>
      )}
    </div>
  );
};
