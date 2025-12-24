/**
 * Floating Chat Button
 * Toggles chat window open/closed
 */
import React from 'react';

interface ChatButtonProps {
  isOpen: boolean;
  onClick: () => void;
  hasUnread?: boolean;
}

export function ChatButton({ isOpen, onClick, hasUnread = false }: ChatButtonProps) {
  console.log('[ChatButton] Rendering button, isOpen:', isOpen);

  return (
    <button
      onClick={onClick}
      className="fixed bottom-6 right-6 z-50 w-16 h-16 text-white rounded-full shadow-2xl flex items-center justify-center transition-all duration-300 hover:scale-110"
      aria-label={isOpen ? 'Close chat' : 'Open chat'}
      style={{
        position: 'fixed',
        bottom: '24px',
        right: '24px',
        zIndex: 9999,
        background: 'linear-gradient(135deg, #1e40af 0%, #2563eb 100%)',
        width: '64px',
        height: '64px',
        borderRadius: '9999px',
        border: '3px solid #3b82f6',
        boxShadow: '0 10px 25px -5px rgba(37, 99, 235, 0.5)'
      }}
    >
      {isOpen ? (
        // X icon when open
        <svg
          xmlns="http://www.w3.org/2000/svg"
          className="h-7 w-7"
          fill="none"
          viewBox="0 0 24 24"
          stroke="currentColor"
          strokeWidth={2.5}
        >
          <path
            strokeLinecap="round"
            strokeLinejoin="round"
            d="M6 18L18 6M6 6l12 12"
          />
        </svg>
      ) : (
        // Robot icon when closed
        <span style={{ fontSize: '32px' }}>🤖</span>
      )}
      {hasUnread && (
        <span className="absolute -top-1 -right-1 w-3 h-3 bg-red-500 rounded-full border-2 border-white" />
      )}
    </button>
  );
}
