/**
 * Chat Input Component
 * Text input with send button
 */
import React, { useState, KeyboardEvent } from 'react';

interface ChatInputProps {
  onSend: (message: string, contextText?: string) => Promise<void>;
  disabled?: boolean;
  contextText?: string;
}

export function ChatInput({ onSend, disabled = false, contextText }: ChatInputProps) {
  const [input, setInput] = useState('');

  const handleSend = async () => {
    if (!input.trim() || disabled) return;

    const message = input.trim();
    setInput('');
    await onSend(message, contextText);
  };

  const handleKeyDown = (e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  };

  return (
    <div className="flex gap-3 items-end" style={{ width: '100%', pointerEvents: 'auto' }}>
      <div style={{
        flex: 1,
        position: 'relative'
      }}>
        <textarea
          value={input}
          onChange={(e) => setInput(e.target.value)}
          onKeyDown={handleKeyDown}
          placeholder={contextText ? "Ask about the selected text..." : "Type your question about robotics..."}
          disabled={disabled}
          className="w-full resize-none px-4 py-3 text-sm focus:outline-none disabled:opacity-50 disabled:cursor-not-allowed"
          rows={2}
          style={{
            pointerEvents: 'auto',
            touchAction: 'auto',
            userSelect: 'text',
            WebkitUserSelect: 'text',
            fontSize: '14px',
            lineHeight: '1.6',
            border: '2px solid #e0e7ff',
            backgroundColor: '#ffffff',
            color: '#1e293b',
            transition: 'all 0.2s',
            fontFamily: 'system-ui, -apple-system, sans-serif',
            boxShadow: '0 1px 3px rgba(0, 0, 0, 0.05)',
            borderRadius: '20px',
            minHeight: '50px',
            maxHeight: '120px',
            overflowY: 'auto'
          }}
          autoFocus
          onFocus={(e) => {
            e.target.style.borderColor = '#3b82f6';
            e.target.style.boxShadow = '0 0 0 3px rgba(59, 130, 246, 0.1)';
          }}
          onBlur={(e) => {
            e.target.style.borderColor = '#e0e7ff';
            e.target.style.boxShadow = '0 1px 3px rgba(0, 0, 0, 0.05)';
          }}
        />
      </div>
      <button
        onClick={handleSend}
        disabled={disabled || !input.trim()}
        className="rounded-full disabled:opacity-50 disabled:cursor-not-allowed transition-all flex items-center justify-center hover:scale-105 hover:shadow-lg"
        aria-label="Send message"
        style={{
          backgroundColor: disabled || !input.trim() ? '#cbd5e1' : '#2563eb',
          color: 'white',
          minWidth: '50px',
          minHeight: '50px',
          width: '50px',
          height: '50px',
          boxShadow: '0 4px 14px rgba(37, 99, 235, 0.35)',
          border: 'none',
          cursor: disabled || !input.trim() ? 'not-allowed' : 'pointer',
          flexShrink: 0,
          transition: 'all 0.2s ease-in-out'
        }}
      >
        <svg
          xmlns="http://www.w3.org/2000/svg"
          width="20"
          height="20"
          viewBox="0 0 24 24"
          fill="none"
          stroke="currentColor"
          strokeWidth="2.5"
          strokeLinecap="round"
          strokeLinejoin="round"
          style={{
            transform: 'rotate(90deg)'
          }}
        >
          <path d="M12 19l9 2-9-18-9 18 9-2zm0 0v-8" />
        </svg>
      </button>
    </div>
  );
}
