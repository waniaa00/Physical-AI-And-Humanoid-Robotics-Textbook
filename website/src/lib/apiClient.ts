/**
 * API Client for backend communication
 * Handles HTTP requests with timeout and retry logic
 */
import type { ChatRequest, ChatResponse } from '../types/api';
import { APIError } from '../types/api';
import { useBackendUrl, API_ENDPOINTS, REQUEST_TIMEOUT, MAX_RETRIES } from './config';

/**
 * Send a chat message to the backend agent
 * Includes timeout handling and retry logic
 */
export async function sendMessage(
  request: ChatRequest,
  backendUrl: string
): Promise<ChatResponse> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), REQUEST_TIMEOUT);

  try {
    // T064: Include credentials (cookies) for authentication
    const response = await fetch(`${backendUrl}${API_ENDPOINTS.CHAT}`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include', // Include HttpOnly session cookie
      body: JSON.stringify(request),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));

      if (response.status >= 400 && response.status < 500) {
        throw new APIError(
          response.status,
          'VALIDATION_ERROR',
          errorData.detail || 'Invalid request'
        );
      } else {
        throw new APIError(
          response.status,
          'SERVER_ERROR',
          errorData.detail || 'Server error occurred'
        );
      }
    }

    const data: ChatResponse = await response.json();
    return data;
  } catch (error: any) {
    clearTimeout(timeoutId);

    if (error.name === 'AbortError') {
      throw new APIError(408, 'TIMEOUT', 'Request timed out after 30 seconds');
    }

    if (error instanceof APIError) {
      throw error;
    }

    // Network error
    throw new APIError(
      0,
      'NETWORK_ERROR',
      'Failed to connect to backend. Please check your connection.'
    );
  }
}

/**
 * Send message with automatic retry on network errors
 */
export async function sendMessageWithRetry(
  request: ChatRequest,
  backendUrl: string,
  maxRetries: number = MAX_RETRIES
): Promise<ChatResponse> {
  let lastError: APIError | null = null;

  for (let attempt = 0; attempt <= maxRetries; attempt++) {
    try {
      return await sendMessage(request, backendUrl);
    } catch (error: any) {
      lastError = error;

      // Don't retry on validation errors (4xx)
      if (error.status >= 400 && error.status < 500) {
        throw error;
      }

      // Don't retry on last attempt
      if (attempt === maxRetries) {
        throw error;
      }

      // Exponential backoff: 1s, 2s
      const delay = Math.pow(2, attempt) * 1000;
      await new Promise(resolve => setTimeout(resolve, delay));
    }
  }

  throw lastError || new APIError(0, 'UNKNOWN_ERROR', 'Request failed');
}
