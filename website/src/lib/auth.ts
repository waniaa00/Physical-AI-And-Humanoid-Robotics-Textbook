/**
 * Auth client initialization and utilities (T028).
 *
 * Provides:
 * - API base URL configuration
 * - Axios instance with credentials support
 * - Cookie settings configuration
 */

import axios from 'axios';

// Use localhost for development - browser environment
export const API_BASE_URL = 'https://wnxddev-humanoid-robotics-api.hf.space';

/**
 * Axios instance configured for authentication requests.
 * Includes credentials (cookies) for HttpOnly session token.
 */
export const authClient = axios.create({
  baseURL: API_BASE_URL,
  withCredentials: true, // Include cookies in all requests
  headers: {
    'Content-Type': 'application/json',
  },
});

/**
 * Cookie settings for session tokens.
 * These match the backend cookie configuration.
 */
export const COOKIE_SETTINGS = {
  name: 'session_token',
  maxAge: 60 * 60 * 24 * 7, // 7 days in seconds
  sameSite: 'lax' as const,
  secure: false, // Set to true in production
  httpOnly: true, // Cannot be accessed by JavaScript (set by server)
};

/**
 * Check if user is authenticated by checking for session cookie.
 * Note: This is a client-side check only. Server validates the actual token.
 */
export const hasSessionCookie = (): boolean => {
  return document.cookie.includes(COOKIE_SETTINGS.name);
};
