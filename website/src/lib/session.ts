/**
 * Session management utilities
 * Handles session ID generation and sessionStorage persistence
 */
import type { Session, SelectedTextContext } from '../types/chat';

const SESSION_ID_KEY = 'chat_session_id';
const SESSION_DATA_KEY = 'chat_session';

/**
 * Generate a new UUID v4 session ID
 */
export function generateSessionId(): string {
  // Check if crypto.randomUUID is available (browser only)
  if (typeof crypto !== 'undefined' && crypto.randomUUID) {
    return crypto.randomUUID();
  }

  // Fallback for SSR or older browsers
  return 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
    const r = (Math.random() * 16) | 0;
    const v = c === 'x' ? r : (r & 0x3) | 0x8;
    return v.toString(16);
  });
}

/**
 * Validate session ID format (UUID v4)
 */
export function isValidSessionId(id: string): boolean {
  const uuidV4Regex =
    /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i;
  return uuidV4Regex.test(id);
}

/**
 * Create a new session object
 */
export function createNewSession(): Session {
  return {
    sessionId: generateSessionId(),
    messages: [],
    mode: 'full-book',
    selectedContext: null,
    createdAt: new Date(),
    lastActivityAt: new Date(),
  };
}

/**
 * Get existing session ID or create new one
 */
export function getOrCreateSessionId(): string {
  if (typeof window === 'undefined') {
    return generateSessionId(); // SSR fallback
  }

  try {
    let sessionId = sessionStorage.getItem(SESSION_ID_KEY);

    if (!sessionId || !isValidSessionId(sessionId)) {
      sessionId = generateSessionId();
      sessionStorage.setItem(SESSION_ID_KEY, sessionId);
    }

    return sessionId;
  } catch (error) {
    // SessionStorage not available (private browsing, etc.)
    return generateSessionId();
  }
}

/**
 * Persist session to sessionStorage
 */
export function persistSession(session: Session): void {
  if (typeof window === 'undefined') return;

  try {
    const serialized = JSON.stringify({
      ...session,
      createdAt: session.createdAt.toISOString(),
      lastActivityAt: session.lastActivityAt.toISOString(),
      messages: session.messages.map(msg => ({
        ...msg,
        timestamp: msg.timestamp.toISOString(),
      })),
    });
    sessionStorage.setItem(SESSION_DATA_KEY, serialized);
    sessionStorage.setItem(SESSION_ID_KEY, session.sessionId);
  } catch (error) {
    console.warn('Failed to persist session:', error);
  }
}

/**
 * Load session from sessionStorage
 */
export function loadSession(): Session | null {
  if (typeof window === 'undefined') return null;

  try {
    const serialized = sessionStorage.getItem(SESSION_DATA_KEY);
    if (!serialized) return null;

    const parsed = JSON.parse(serialized);

    // Rehydrate Date objects
    return {
      ...parsed,
      createdAt: new Date(parsed.createdAt),
      lastActivityAt: new Date(parsed.lastActivityAt),
      messages: parsed.messages.map((msg: any) => ({
        ...msg,
        timestamp: new Date(msg.timestamp),
      })),
    };
  } catch (error) {
    console.warn('Failed to load session:', error);
    return null;
  }
}

/**
 * Clear session from sessionStorage
 */
export function clearSession(): void {
  if (typeof window === 'undefined') return;

  try {
    sessionStorage.removeItem(SESSION_ID_KEY);
    sessionStorage.removeItem(SESSION_DATA_KEY);
  } catch (error) {
    console.warn('Failed to clear session:', error);
  }
}
