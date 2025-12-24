/**
 * API types for frontend-backend communication
 * Reference: specs/4-docusaurus-rag-integration/contracts/frontend-backend-api.md
 */

/**
 * Request payload for /agent/chat endpoint
 */
export interface ChatRequest {
  message: string;
  session_id: string;
  context_text: string | null;
  user_id?: string | null; // Optional UUID for personalization
}

/**
 * Response payload from /agent/chat endpoint
 */
export interface ChatResponse {
  response: string;
  session_id: string;
  status: 'success' | 'guardrail_triggered' | 'error';
}

/**
 * API error wrapper
 */
export class APIError extends Error {
  constructor(
    public status: number,
    public code: string,
    message: string
  ) {
    super(message);
    this.name = 'APIError';
  }
}

/**
 * API client configuration
 */
export interface APIConfig {
  baseUrl: string;
  timeout: number;
  retryAttempts: number;
}
