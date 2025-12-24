/**
 * SummarizeButton Component
 *
 * Button component for triggering text summarization.
 * Handles API calls, loading states, and error handling.
 */

import React, { useState } from 'react';
import styles from './SummarizeButton.module.css';

interface SummarizeButtonProps {
  /** Text to summarize */
  text: string;
  /** Callback when summarization completes */
  onSummarize: (summary: string, originalText: string, metrics: SummaryMetrics) => void;
  /** Optional className for styling */
  className?: string;
  /** Summary length (short/medium/long, default: short) */
  targetLength?: 'short' | 'medium' | 'long';
  /** Summary focus (concepts/code/both, default: concepts) */
  focus?: 'concepts' | 'code' | 'both';
  /** Optional user ID for logging */
  userId?: string;
  /** Inline styles to override defaults */
  style?: React.CSSProperties;
}

interface SummaryMetrics {
  original_word_count: number;
  summary_word_count: number;
  compression_ratio: number;
  processing_time_ms: number;
  key_points?: string[];
}

interface SummarizationResponse {
  summary: string;
  original_word_count: number;
  summary_word_count: number;
  compression_ratio: number;
  processing_time_ms: number;
  key_points?: string[];
}

export const SummarizeButton: React.FC<SummarizeButtonProps> = ({
  text,
  onSummarize,
  className = '',
  targetLength = 'short',
  focus = 'concepts',
  userId,
  style = {},
}) => {
  const [isSummarizing, setIsSummarizing] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleSummarize = async () => {
    // Validate text
    if (!text || text.trim().length === 0) {
      setError('Please select some text to summarize');
      return;
    }

    const wordCount = text.split(' ').filter(w => w.length > 0).length;
    if (wordCount < 50) {
      setError('Selected text is too short (minimum 50 words)');
      return;
    }

    if (wordCount > 5000) {
      setError('Selected text is too long (max 5,000 words)');
      return;
    }

    setIsSummarizing(true);
    setError(null);

    try {
      // Call summarization API
      const apiUrl = 'https://wnxddev-humanoid-robotics-api.hf.space';
      const response = await fetch(`${apiUrl}/summarize`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text,
          target_length: targetLength,
          focus,
          user_id: userId,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail?.message || 'Summarization failed');
      }

      const data: SummarizationResponse = await response.json();

      // Call callback with summary and metrics
      onSummarize(data.summary, text, {
        original_word_count: data.original_word_count,
        summary_word_count: data.summary_word_count,
        compression_ratio: data.compression_ratio,
        processing_time_ms: data.processing_time_ms,
        key_points: data.key_points,
      });

      console.log(
        `Summarization completed in ${data.processing_time_ms}ms ` +
        `(${data.compression_ratio}% compression)`
      );
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Summarization failed';
      setError(errorMessage);
      console.error('Summarization error:', err);
    } finally {
      setIsSummarizing(false);
    }
  };

  return (
    <div className={`${styles.summarizeButtonContainer} ${className}`} style={{ width: '100%', height: '100%' }}>
      <button
        className={styles.summarizeButton}
        onClick={handleSummarize}
        disabled={isSummarizing}
        aria-label="Summarize text"
        style={{
          ...style,
          width: '100%',
          height: '100%',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          gap: '4px',
          opacity: isSummarizing ? 0.7 : 1,
        }}
        onMouseEnter={(e) => {
          if (!isSummarizing) {
            e.currentTarget.style.transform = 'translateY(-1px) scale(1.02)';
            e.currentTarget.style.boxShadow = '0 4px 16px rgba(139, 92, 246, 0.4)';
          }
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = 'scale(1)';
          e.currentTarget.style.boxShadow = '0 2px 8px rgba(139, 92, 246, 0.25)';
        }}
      >
        {isSummarizing ? (
          <>
            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" className={styles.spinner}>
              <circle cx="12" cy="12" r="10" strokeOpacity="0.25" />
              <path d="M12 2a10 10 0 0 1 10 10" strokeLinecap="round" />
            </svg>
            Summarizing...
          </>
        ) : (
          <>
            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round">
              <path d="M4 7h16M4 12h10M4 17h16" />
            </svg>
            Summarize
          </>
        )}
      </button>
      {error && (
        <div className={styles.error} role="alert">
          {error}
        </div>
      )}
    </div>
  );
};
