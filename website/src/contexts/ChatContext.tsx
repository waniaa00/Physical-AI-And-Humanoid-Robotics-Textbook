/**
 * Chat Context Provider
 * Manages chat state and provides chat functionality throughout the app
 */
import React, { createContext, useContext, useState, useCallback, useEffect } from 'react';
import type { ChatContextValue, Session, ChatMessage, UIState } from '../types/chat';
import { sendMessageWithRetry } from '../lib/apiClient';
import { useBackendUrl } from '../lib/config';
import { createNewSession, persistSession, loadSession, getOrCreateSessionId, generateSessionId } from '../lib/session';
import { parseCitations } from '../lib/citations';
import { getCurrentUserId } from '../lib/user';

const ChatContext = createContext<ChatContextValue | undefined>(undefined);

export function ChatProvider({ children }: { children: React.ReactNode }) {
  const backendUrl = useBackendUrl();

  // Initialize session from storage or create new
  const [session, setSession] = useState<Session>(() => {
    const loaded = loadSession();
    return loaded || createNewSession();
  });

  const [uiState, setUIState] = useState<UIState>({
    isLoading: false,
    error: null,
    selectedText: null,
  });

  // Persist session whenever it changes
  useEffect(() => {
    persistSession(session);
  }, [session]);

  const sendMessage = useCallback(async (message: string, contextText?: string) => {
    console.log('[ChatContext] sendMessage called with:', { message, contextText, sessionId: session.sessionId });
    setUIState(prev => ({ ...prev, isLoading: true, error: null }));

    // Add user message immediately
    const userMessage: ChatMessage = {
      id: generateSessionId(),
      role: 'user',
      content: message,
      timestamp: new Date(),
    };

    console.log('[ChatContext] Created user message:', userMessage);

    setSession(prev => ({
      ...prev,
      messages: [...prev.messages, userMessage],
      mode: contextText ? 'context-constrained' : 'full-book',
      selectedContext: contextText ? {
        text: contextText,
        timestamp: new Date(),
      } : null,
      lastActivityAt: new Date(),
    }));

    try {
      console.log('[ChatContext] Sending to backend:', backendUrl);

      // Get current user ID for personalization (T065)
      const userId = getCurrentUserId();
      console.log('[ChatContext] User ID for personalization:', userId);

      const response = await sendMessageWithRetry(
        {
          message,
          session_id: session.sessionId,
          context_text: contextText || null,
          user_id: userId, // Include user_id for personalization (T065)
        },
        backendUrl
      );

      console.log('[ChatContext] Received response:', response);

      // Parse citations from response
      let citations = null;
      try {
        citations = parseCitations(response.response);
        console.log('[ChatContext] Parsed citations:', citations);
      } catch (citationError) {
        console.warn('[ChatContext] Citation parsing failed:', citationError);
      }

      // Add assistant message
      const assistantMessage: ChatMessage = {
        id: generateSessionId(),
        role: 'assistant',
        content: response.response || 'No response received',
        timestamp: new Date(),
        citations: citations?.sources || undefined,
      };

      console.log('[ChatContext] Created assistant message:', assistantMessage);

      setSession(prev => {
        const updatedSession = {
          ...prev,
          messages: [...prev.messages, assistantMessage],
          lastActivityAt: new Date(),
        };
        console.log('[ChatContext] Updated session:', updatedSession);
        return updatedSession;
      });

      console.log('[ChatContext] Assistant message added to session, setting isLoading to false');
      setUIState(prev => ({ ...prev, isLoading: false }));
    } catch (error: any) {
      console.error('[ChatContext] Error sending message:', error);
      const uiError = {
        type: error.code === 'TIMEOUT' ? 'timeout' :
              error.code === 'NETWORK_ERROR' ? 'network' :
              error.status >= 500 ? 'server' :
              error.status >= 400 ? 'validation' : 'unknown',
        message: error.message || 'An error occurred',
        timestamp: new Date(),
      };

      setUIState(prev => ({
        ...prev,
        isLoading: false,
        error: uiError as any,
      }));
    }
  }, [session.sessionId, backendUrl]);

  const clearChat = useCallback(() => {
    const newSession = createNewSession();
    setSession(newSession);
    setUIState({
      isLoading: false,
      error: null,
      selectedText: null,
    });
  }, []);

  const selectText = useCallback((text: string) => {
    setUIState(prev => ({ ...prev, selectedText: text }));
  }, []);

  const clearSelection = useCallback(() => {
    setUIState(prev => ({ ...prev, selectedText: null }));
  }, []);

  const value: ChatContextValue = {
    session,
    uiState,
    sendMessage,
    clearChat,
    selectText,
    clearSelection,
  };

  return <ChatContext.Provider value={value}>{children}</ChatContext.Provider>;
}

export function useChatContext() {
  const context = useContext(ChatContext);
  if (!context) {
    throw new Error('useChatContext must be used within ChatProvider');
  }
  return context;
}
