/**
 * SummaryDisplay Component
 *
 * Displays generated summaries with metrics and optional key points.
 * Includes toggle functionality to switch between summary and original text.
 */

import React, { useState } from 'react';
import styles from './SummaryDisplay.module.css';

interface SummaryMetrics {
  original_word_count: number;
  summary_word_count: number;
  compression_ratio: number;
  processing_time_ms: number;
  key_points?: string[];
}

interface SummaryDisplayProps {
  /** Generated summary text */
  summary: string;
  /** Original text */
  originalText: string;
  /** Summary metrics */
  metrics: SummaryMetrics;
  /** Whether to show summary by default (default: true) */
  defaultShowSummary?: boolean;
  /** Optional className for styling */
  className?: string;
}

export const SummaryDisplay: React.FC<SummaryDisplayProps> = ({
  summary,
  originalText,
  metrics,
  defaultShowSummary = true,
  className = '',
}) => {
  const [showSummary, setShowSummary] = useState(defaultShowSummary);

  const toggleView = () => {
    setShowSummary(!showSummary);
  };

  return (
    <div className={`${styles.summaryDisplayContainer} ${className}`}>
      {/* Header with metrics */}
      <div className={styles.header}>
        <div className={styles.title}>
          <span className={styles.icon}>📝</span>
          <h3>{showSummary ? 'Summary' : 'Original Text'}</h3>
        </div>
        <button
          className={styles.toggleButton}
          onClick={toggleView}
          aria-label={showSummary ? 'Show original text' : 'Show summary'}
        >
          {showSummary ? (
            <>
              <span className={styles.icon}>🔙</span>
              Show Original
            </>
          ) : (
            <>
              <span className={styles.icon}>📝</span>
              Show Summary
            </>
          )}
        </button>
      </div>

      {/* Metrics bar */}
      <div className={styles.metrics}>
        <div className={styles.metric}>
          <span className={styles.metricLabel}>Original:</span>
          <span className={styles.metricValue}>{metrics.original_word_count} words</span>
        </div>
        <div className={styles.metric}>
          <span className={styles.metricLabel}>Summary:</span>
          <span className={styles.metricValue}>{metrics.summary_word_count} words</span>
        </div>
        <div className={styles.metric}>
          <span className={styles.metricLabel}>Compression:</span>
          <span className={styles.metricValue}>
            {Math.round(metrics.compression_ratio * 100)}%
          </span>
        </div>
        <div className={styles.metric}>
          <span className={styles.metricLabel}>Time:</span>
          <span className={styles.metricValue}>
            {(metrics.processing_time_ms / 1000).toFixed(1)}s
          </span>
        </div>
      </div>

      {/* Content display */}
      <div className={styles.content}>
        {showSummary ? (
          <>
            <div className={styles.summaryText}>{summary}</div>

            {/* Key points (if available) */}
            {metrics.key_points && metrics.key_points.length > 0 && (
              <div className={styles.keyPoints}>
                <h4 className={styles.keyPointsTitle}>Key Points:</h4>
                <ul className={styles.keyPointsList}>
                  {metrics.key_points.map((point, index) => (
                    <li key={index} className={styles.keyPoint}>
                      {point}
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </>
        ) : (
          <div className={styles.originalText}>{originalText}</div>
        )}
      </div>
    </div>
  );
};
