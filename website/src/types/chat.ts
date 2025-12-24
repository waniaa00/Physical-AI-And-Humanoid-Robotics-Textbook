/**
 * Chat Entity Types
 * Core data structures for chat functionality
 */

export interface ChatMessage {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  citations?: SourceCitation[];
}

export interface SourceCitation {
  number: number;
  url: string;
  text?: string;
}

export interface SelectedTextContext {
  text: string;
  source?: string;
  timestamp: Date;
}

export interface Session {
  sessionId: string;
  messages: ChatMessage[];
  mode: 'full-book' | 'context-constrained';
  selectedContext: SelectedTextContext | null;
  createdAt: Date;
  lastActivityAt: Date;
}

export interface UIState {
  isLoading: boolean;
  error: UIError | null;
  selectedText: string | null;
}

export interface UIError {
  type: 'network' | 'timeout' | 'validation' | 'server' | 'unknown';
  message: string;
  timestamp: Date;
}

export interface ChatContextValue {
  session: Session;
  uiState: UIState;
  sendMessage: (message: string, contextText?: string) => Promise<void>;
  clearChat: () => void;
  selectText: (text: string) => void;
  clearSelection: () => void;
}
