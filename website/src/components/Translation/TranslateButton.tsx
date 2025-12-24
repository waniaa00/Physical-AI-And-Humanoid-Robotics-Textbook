/**
 * TranslateButton Component
 *
 * Button component for triggering text translation to Urdu.
 * Handles API calls, loading states, and error handling.
 */

import React, { useState } from 'react';
import styles from './TranslateButton.module.css';

interface TranslateButtonProps {
  /** Text to translate */
  text: string;
  /** Callback when translation completes */
  onTranslate: (translatedText: string, originalText: string) => void;
  /** Optional className for styling */
  className?: string;
  /** Whether to preserve code blocks (default: true) */
  preserveCode?: boolean;
  /** Optional user ID for logging */
  userId?: string;
  /** Inline styles to override defaults */
  style?: React.CSSProperties;
}

interface TranslationResponse {
  translated_text: string;
  original_text: string;
  detected_lang: string;
  target_lang: string;
  word_count: number;
  translation_time_ms: number;
  preserved_elements?: Record<string, number>;
}

export const TranslateButton: React.FC<TranslateButtonProps> = ({
  text,
  onTranslate,
  className = '',
  preserveCode = true,
  userId,
  style = {},
}) => {
  const [isTranslating, setIsTranslating] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const handleTranslate = async () => {
    // Validate text
    if (!text || text.trim().length === 0) {
      setError('Please select some text to translate');
      return;
    }

    const wordCount = text.split(' ').length;
    if (wordCount > 5000) {
      setError('Selected text is too long (max 5,000 words)');
      return;
    }

    setIsTranslating(true);
    setError(null);

    try {
      // Call translation API
      const apiUrl = 'http://localhost:8000';
      const response = await fetch(`${apiUrl}/translate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          text,
          target_lang: 'ur',
          preserve_code: preserveCode,
          user_id: userId,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.detail?.message || 'Translation failed');
      }

      const data: TranslationResponse = await response.json();

      // Call callback with translated text
      onTranslate(data.translated_text, data.original_text);

      console.log(`Translation completed in ${data.translation_time_ms}ms`);
    } catch (err) {
      const errorMessage = err instanceof Error ? err.message : 'Translation failed';
      setError(errorMessage);
      console.error('Translation error:', err);
    } finally {
      setIsTranslating(false);
    }
  };

  return (
    <div className={`${styles.translateButtonContainer} ${className}`} style={{ width: '100%', height: '100%' }}>
      <button
        className={styles.translateButton}
        onClick={handleTranslate}
        disabled={isTranslating}
        aria-label="Translate to Urdu"
        style={{
          ...style,
          width: '100%',
          height: '100%',
          display: 'flex',
          alignItems: 'center',
          justifyContent: 'center',
          gap: '4px',
          opacity: isTranslating ? 0.7 : 1,
        }}
        onMouseEnter={(e) => {
          if (!isTranslating) {
            e.currentTarget.style.transform = 'translateY(-1px) scale(1.02)';
            e.currentTarget.style.boxShadow = '0 4px 16px rgba(59, 130, 246, 0.4)';
          }
        }}
        onMouseLeave={(e) => {
          e.currentTarget.style.transform = 'scale(1)';
          e.currentTarget.style.boxShadow = '0 2px 8px rgba(59, 130, 246, 0.25)';
        }}
      >
        {isTranslating ? (
          <>
            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" className={styles.spinner}>
              <circle cx="12" cy="12" r="10" strokeOpacity="0.25" />
              <path d="M12 2a10 10 0 0 1 10 10" strokeLinecap="round" />
            </svg>
            Translating...
          </>
        ) : (
          <>
            <svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round">
              <circle cx="12" cy="12" r="10" />
              <path d="M2 12h20M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z" />
            </svg>
            Translate
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
