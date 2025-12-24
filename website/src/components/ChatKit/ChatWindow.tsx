/**
 * Chat Window Component
 * Main container for the chat interface
 */
import React, { useState, useEffect } from 'react';
import { MessageList } from './MessageList';
import { ChatInput } from './ChatInput';
import { useChatContext } from '../../contexts/ChatContext';
import { useAuth } from '../../hooks/useAuth'; // T065: Use auth context
import { getCurrentUserId, getUserHasInterests, setUserHasInterests } from '../../lib/user';

interface ChatWindowProps {
  isOpen: boolean;
  onClose?: () => void;
}

export function ChatWindow({ isOpen, onClose }: ChatWindowProps) {
  const { session, uiState, sendMessage, clearChat } = useChatContext();
  const { user, isAuthenticated } = useAuth(); // T065: Use auth context
  const [isPersonalized, setIsPersonalized] = useState(false);
  const [userInterests, setUserInterests] = useState<string[]>([]);

  // T065: Check if personalization is active based on authentication
  useEffect(() => {
    const checkPersonalization = async () => {
      // Use authenticated user from auth context
      if (!isAuthenticated || !user) {
        setIsPersonalized(false);
        return;
      }

      // Check cached value first
      const cachedHasInterests = getUserHasInterests();
      if (cachedHasInterests) {
        setIsPersonalized(true);
        return;
      }

      // Check if user has interests (from auth context or API)
      if (user.interests && user.interests.length > 0) {
        setIsPersonalized(true);
        return;
      }

      // Fetch from backend to verify if not in user object
      try {
        const apiUrl = 'https://wnxddev-humanoid-robotics-api.hf.space';
        const response = await fetch(`${apiUrl}/interests/${user.user_id}`, {
          credentials: 'include', // Include session cookie
        });

        if (response.ok) {
          const data = await response.json();
          const hasInterests = data.interests && data.interests.length > 0;
          setIsPersonalized(hasInterests);
          setUserInterests(data.interests?.map((i: any) => i.name) || []);
          setUserHasInterests(hasInterests);
        } else {
          setIsPersonalized(false);
          setUserHasInterests(false);
        }
      } catch (error) {
        console.warn('[ChatWindow] Failed to check personalization:', error);
        setIsPersonalized(false);
      }
    };

    if (isOpen) {
      checkPersonalization();
    }
  }, [isOpen, isAuthenticated, user]);

  if (!isOpen) return null;

  return (
    <div
      className="fixed z-40 bg-white rounded-2xl shadow-2xl border flex flex-col"
      style={{
        position: 'fixed',
        top: '50%',
        left: '50%',
        transform: 'translate(-50%, -50%)',
        zIndex: 9998,
        width: '420px',
        height: '640px',
        maxHeight: '88vh',
        backgroundColor: '#ffffff',
        borderColor: '#e2e8f0',
        borderRadius: '16px',
        boxShadow: '0 25px 50px -12px rgba(0, 0, 0, 0.25), 0 0 0 1px rgba(0, 0, 0, 0.05)',
        overflow: 'hidden',
        display: 'flex',
        flexDirection: 'column'
      }}
    >
      {/* Header */}
      <div className="px-4 py-3 flex items-center justify-between" style={{
        background: 'linear-gradient(135deg, #1e293b 0%, #334155 100%)',
        borderBottom: '1px solid rgba(255, 255, 255, 0.1)',
        minHeight: '60px',
        alignItems: 'center'
      }}>
        <div className="flex items-center gap-3 flex-1 min-w-0">
          <div style={{
            width: '40px',
            height: '40px',
            borderRadius: '10px',
            background: 'linear-gradient(135deg, #64748b 0%, #475569 100%)',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            fontSize: '16px',
            color: '#f1f5f9',
            flexShrink: 0,
            boxShadow: '0 4px 6px -1px rgba(0, 0, 0, 0.1), 0 2px 4px -1px rgba(0, 0, 0, 0.06)'
          }}>
            🤖
          </div>
          <div className="flex-1 min-w-0">
            <div style={{ display: 'flex', alignItems: 'center', gap: '6px', marginBottom: '1px' }}>
              <h3 className="font-bold text-lg m-0 text-white" style={{
                fontSize: '15px',
                fontWeight: '600',
                lineHeight: '1.3'
              }}>
                Robotics Assistant
              </h3>
              {/* Personalization indicator (T066) */}
              {isPersonalized && (
                <span
                  style={{
                    backgroundColor: 'rgba(34, 197, 94, 0.15)',
                    color: '#a7f3d0',
                    fontSize: '9px',
                    fontWeight: 600,
                    padding: '1px 6px',
                    borderRadius: '10px',
                    border: '1px solid rgba(34, 197, 94, 0.3)',
                    display: 'inline-flex',
                    alignItems: 'center',
                    gap: '3px',
                    letterSpacing: '0.3px',
                    textTransform: 'uppercase',
                    backdropFilter: 'blur(4px)'
                  }}
                  title={`Personalized for: ${userInterests.join(', ')}`}
                >
                  ✨ Personal
                </span>
              )}
            </div>
            <p className="text-xs m-0" style={{
              color: '#cbd5e1',
              opacity: 0.9,
              fontSize: '11px',
              lineHeight: '1.3'
            }}>
              {isPersonalized
                ? `For: ${userInterests.slice(0, 2).join(', ')}${userInterests.length > 2 ? ` +${userInterests.length - 2}` : ''}`
                : 'Your AI robotics guide'
              }
            </p>
          </div>
        </div>
        <div className="flex gap-2">
          <button
            onClick={clearChat}
            className="transition-all hover:bg-white/20 active:scale-95"
            aria-label="Clear chat"
            title="Clear conversation"
            style={{
              backgroundColor: 'rgba(255, 255, 255, 0.1)',
              padding: '8px',
              borderRadius: '8px',
              color: '#e2e8f0',
              display: 'flex',
              alignItems: 'center',
              justifyContent: 'center',
              width: '40px',
              height: '40px',
              cursor: 'pointer',
              border: '1px solid rgba(255, 255, 255, 0.15)',
              backdropFilter: 'blur(4px)',
              flexShrink: 0
            }}
          >
            <svg
              xmlns="http://www.w3.org/2000/svg"
              className="h-4 w-4"
              fill="none"
              viewBox="0 0 24 24"
              stroke="currentColor"
            >
              <path
                strokeLinecap="round"
                strokeLinejoin="round"
                strokeWidth={2}
                d="M19 7l-.867 12.142A2 2 0 0116.138 21H7.862a2 2 0 01-1.995-1.858L5 7m5 4v6m4-6v6m1-10V4a1 1 0 00-1-1h-4a1 1 0 00-1 1v3M4 7h16"
              />
            </svg>
          </button>
          {onClose && (
            <button
              onClick={onClose}
              className="transition-all hover:bg-white/20 active:scale-95"
              aria-label="Close chat"
              title="Close chat"
              style={{
                backgroundColor: 'rgba(255, 255, 255, 0.1)',
                padding: '8px',
                borderRadius: '8px',
                color: '#e2e8f0',
                display: 'flex',
                alignItems: 'center',
                justifyContent: 'center',
                width: '40px',
                height: '40px',
                cursor: 'pointer',
                border: '1px solid rgba(255, 255, 255, 0.15)',
                backdropFilter: 'blur(4px)',
                flexShrink: 0
              }}
            >
              <svg
                xmlns="http://www.w3.org/2000/svg"
                className="h-4 w-4"
                fill="none"
                viewBox="0 0 24 24"
                stroke="currentColor"
              >
                <path
                  strokeLinecap="round"
                  strokeLinejoin="round"
                  strokeWidth={2}
                  d="M6 18L18 6M6 6l12 12"
                />
              </svg>
            </button>
          )}
        </div>
      </div>

      {/* Mode indicator */}
      {session.mode === 'context-constrained' && uiState.selectedText && (
        <div className="border-b p-3 bg-blue-50 border-blue-200" style={{
          backgroundColor: '#eff6ff',
          borderColor: '#93c5fd',
          padding: '12px 16px'
        }}>
          <div className="flex items-center gap-2" style={{ color: '#1e40af' }}>
            <span style={{ fontSize: '16px' }}>📌</span>
            <span className="font-semibold text-sm" style={{ color: '#1e40af', fontSize: '13px' }}>Context Mode Active</span>
          </div>
          <div className="mt-2 text-xs p-2 rounded bg-blue-100 border border-blue-200" style={{
            backgroundColor: '#dbeafe',
            color: '#1e40af',
            border: '1px solid #93c5fd',
            borderRadius: '8px',
            fontSize: '12px',
            lineHeight: '1.4'
          }}>
            "{uiState.selectedText.length > 100 ? uiState.selectedText.substring(0, 100) + '...' : uiState.selectedText}"
          </div>
        </div>
      )}

      {/* Messages */}
      <div className="flex-1 overflow-hidden" style={{
        backgroundColor: '#f8fafc',
        flex: '1 1 auto'
      }}>
        <MessageList messages={session.messages} isLoading={uiState.isLoading} />
      </div>

      {/* Error display */}
      {uiState.error && (
        <div className="border-t p-4 bg-red-50 border-red-200" style={{
          backgroundColor: '#fef2f2',
          borderColor: '#fca5a5',
          color: '#dc2626',
          padding: '12px 16px'
        }}>
          <div className="flex items-start gap-3">
            <span style={{ fontSize: '20px', display: 'flex', alignItems: 'center' }}>⚠️</span>
            <div className="flex-1">
              <div className="font-bold text-sm mb-1" style={{ color: '#b91c1c', fontSize: '13px' }}>Error</div>
              <div className="text-xs" style={{ color: '#dc2626', fontSize: '12px', lineHeight: '1.4' }}>{uiState.error.message}</div>
            </div>
          </div>
        </div>
      )}

      {/* Input */}
      <div className="border-t p-4 bg-white" style={{
        borderColor: '#e2e8f0',
        backgroundColor: '#ffffff',
        borderTopWidth: '1px',
        padding: '16px'
      }}>
        <ChatInput
          onSend={sendMessage}
          disabled={uiState.isLoading}
          contextText={uiState.selectedText || undefined}
        />
      </div>
    </div>
  );
}
