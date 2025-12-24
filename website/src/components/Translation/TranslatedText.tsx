/**
 * TranslatedText Component
 *
 * Displays translated Urdu text with proper RTL (right-to-left) formatting.
 * Includes toggle functionality to switch between original and translated text.
 */

import React, { useState } from 'react';
import styles from './TranslatedText.module.css';

interface TranslatedTextProps {
  /** Original text (English) */
  originalText: string;
  /** Translated text (Urdu) */
  translatedText: string;
  /** Whether to show translated text by default (default: true) */
  defaultShowTranslated?: boolean;
  /** Optional className for styling */
  className?: string;
}

export const TranslatedText: React.FC<TranslatedTextProps> = ({
  originalText,
  translatedText,
  defaultShowTranslated = true,
  className = '',
}) => {
  const [showTranslated, setShowTranslated] = useState(defaultShowTranslated);

  const toggleLanguage = () => {
    setShowTranslated(!showTranslated);
  };

  return (
    <div className={`${styles.translatedTextContainer} ${className}`}>
      {/* Toggle Button */}
      <div className={styles.controls}>
        <button
          className={styles.toggleButton}
          onClick={toggleLanguage}
          aria-label={showTranslated ? 'Show original text' : 'Show Urdu translation'}
        >
          {showTranslated ? (
            <>
              <span className={styles.icon}>🔙</span>
              Show Original
            </>
          ) : (
            <>
              <span className={styles.icon}>🌐</span>
              Show Urdu
            </>
          )}
        </button>
        <span className={styles.languageLabel}>
          {showTranslated ? 'Urdu (اردو)' : 'English'}
        </span>
      </div>

      {/* Text Display */}
      <div
        className={`${styles.textContent} ${showTranslated ? styles.rtl : styles.ltr}`}
        dir={showTranslated ? 'rtl' : 'ltr'}
        lang={showTranslated ? 'ur' : 'en'}
      >
        {showTranslated ? translatedText : originalText}
      </div>
    </div>
  );
};
