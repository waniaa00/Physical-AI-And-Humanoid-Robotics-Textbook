import React, { useEffect, useState } from 'react';
import { useDoc } from '@docusaurus/plugin-content-docs/client';
import { TranslateButton, TranslatedText } from '../Translation';
import { SummarizeButton, SummaryDisplay } from '../Summarization';
import '@site/src/css/translation.css';
import styles from './ChapterButtons.module.css';

interface SummaryMetrics {
  original_word_count: number;
  summary_word_count: number;
  compression_ratio: number;
  processing_time_ms: number;
  key_points?: string[];
}

const ChapterButtons: React.FC = () => {
  const { metadata } = useDoc();
  const [selectedText, setSelectedText] = useState('');
  const [showActionButtons, setShowActionButtons] = useState(false);
  const [translatedText, setTranslatedText] = useState<string | null>(null);
  const [originalText, setOriginalText] = useState('');
  const [summaryText, setSummaryText] = useState<string | null>(null);
  const [summaryMetrics, setSummaryMetrics] = useState<SummaryMetrics | null>(null);

  // Function to handle text selection (for both summarization and translation)
  const handleTextSelection = () => {
    // Check if we're in browser environment
    if (typeof window === 'undefined') return;

    const selected = window.getSelection()?.toString().trim();
    if (!selected || selected.length < 10) { // Require at least 10 characters to avoid accidental triggers
      setShowActionButtons(false);
      return;
    }

    // Check if the selection is in a code block or other non-content area
    const selection = window.getSelection();
    if (selection?.anchorNode) {
      const element = selection.anchorNode.parentElement;
      if (element?.tagName === 'CODE' || element?.closest('pre, code, .code-block')) {
        setShowActionButtons(false);
        return; // Don't trigger for code blocks
      }
    }

    // Store selected text and show action buttons
    setSelectedText(selected);
    setShowActionButtons(true);
  };

  // Add event listener for text selection
  useEffect(() => {
    const handleMouseUp = () => {
      // Small delay to allow selection to complete
      setTimeout(handleTextSelection, 10);
    };

    // Add the event listener to the document
    document.addEventListener('mouseup', handleMouseUp);

    // Also listen for the custom event from the chatbot to handle the selected text
    const handleSummaryRequest = (e: any) => {
      if (e.detail?.text) {
        // This would be handled by the chatbot component
      }
    };

    document.addEventListener('selectedTextSummary', handleSummaryRequest);

    // Cleanup function
    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('selectedTextSummary', handleSummaryRequest);
    };
  }, []);

  const handleTranslate = (translated: string, original: string) => {
    setTranslatedText(translated);
    setOriginalText(original);
    setShowActionButtons(false);
  };

  const handleSummarize = (summary: string, original: string, metrics: SummaryMetrics) => {
    setSummaryText(summary);
    setOriginalText(original);
    setSummaryMetrics(metrics);
    setShowActionButtons(false);
  };

  const handleCloseTranslation = () => {
    setTranslatedText(null);
    setOriginalText('');
  };

  const handleCloseSummary = () => {
    setSummaryText(null);
    setSummaryMetrics(null);
    setOriginalText('');
  };

  return (
    <>
      {/* Floating action buttons for selected text */}
      {showActionButtons && !translatedText && !summaryText && (
        <div
          style={{
            position: 'fixed',
            top: '50%',
            right: '20px',
            transform: 'translateY(-50%)',
            zIndex: 1000,
            backgroundColor: '#ffffff',
            padding: '1rem',
            borderRadius: '12px',
            boxShadow: '0 4px 20px rgba(0, 0, 0, 0.15)',
            border: '2px solid #2563eb',
          }}
        >
          <div style={{ marginBottom: '0.75rem', fontSize: '0.875rem', fontWeight: 600, color: '#1e40af' }}>
            Selected Text ({selectedText.length} chars)
          </div>
          <div style={{ display: 'flex', flexDirection: 'column', gap: '0.5rem' }}>
            <TranslateButton
              text={selectedText}
              onTranslate={handleTranslate}
              preserveCode={true}
            />
            <SummarizeButton
              text={selectedText}
              onSummarize={handleSummarize}
              targetLength="short"
              focus="concepts"
            />
          </div>
          <button
            onClick={() => setShowActionButtons(false)}
            style={{
              marginTop: '0.75rem',
              padding: '0.375rem 0.75rem',
              fontSize: '0.75rem',
              color: '#6b7280',
              backgroundColor: '#f3f4f6',
              border: 'none',
              borderRadius: '6px',
              cursor: 'pointer',
              width: '100%',
            }}
          >
            Cancel
          </button>
        </div>
      )}

      {/* Translated text modal/overlay */}
      {translatedText && (
        <div
          style={{
            position: 'fixed',
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            backgroundColor: 'rgba(0, 0, 0, 0.5)',
            zIndex: 2000,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            padding: '20px',
          }}
          onClick={handleCloseTranslation}
        >
          <div
            style={{
              backgroundColor: '#ffffff',
              borderRadius: '16px',
              padding: '2rem',
              maxWidth: '800px',
              maxHeight: '80vh',
              overflow: 'auto',
              boxShadow: '0 20px 60px rgba(0, 0, 0, 0.3)',
            }}
            onClick={(e) => e.stopPropagation()}
          >
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1.5rem' }}>
              <h3 style={{ margin: 0, color: '#1e40af', fontSize: '1.25rem', fontWeight: 600 }}>
                Translation
              </h3>
              <button
                onClick={handleCloseTranslation}
                style={{
                  backgroundColor: '#f3f4f6',
                  border: 'none',
                  borderRadius: '8px',
                  padding: '0.5rem 1rem',
                  cursor: 'pointer',
                  fontSize: '0.875rem',
                  fontWeight: 600,
                  color: '#6b7280',
                }}
              >
                Close
              </button>
            </div>
            <TranslatedText
              originalText={originalText}
              translatedText={translatedText}
              defaultShowTranslated={true}
            />
          </div>
        </div>
      )}

      {/* Summary modal/overlay */}
      {summaryText && summaryMetrics && (
        <div
          style={{
            position: 'fixed',
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            backgroundColor: 'rgba(0, 0, 0, 0.5)',
            zIndex: 2000,
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            padding: '20px',
          }}
          onClick={handleCloseSummary}
        >
          <div
            style={{
              backgroundColor: '#ffffff',
              borderRadius: '16px',
              padding: '2rem',
              maxWidth: '900px',
              maxHeight: '80vh',
              overflow: 'auto',
              boxShadow: '0 20px 60px rgba(0, 0, 0, 0.3)',
            }}
            onClick={(e) => e.stopPropagation()}
          >
            <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '1.5rem' }}>
              <h3 style={{ margin: 0, color: '#059669', fontSize: '1.25rem', fontWeight: 600 }}>
                Summary
              </h3>
              <button
                onClick={handleCloseSummary}
                style={{
                  backgroundColor: '#f3f4f6',
                  border: 'none',
                  borderRadius: '8px',
                  padding: '0.5rem 1rem',
                  cursor: 'pointer',
                  fontSize: '0.875rem',
                  fontWeight: 600,
                  color: '#6b7280',
                }}
              >
                Close
              </button>
            </div>
            <SummaryDisplay
              summary={summaryText}
              originalText={originalText}
              metrics={summaryMetrics}
              defaultShowSummary={true}
            />
          </div>
        </div>
      )}
    </>
  );
};

export default ChapterButtons;