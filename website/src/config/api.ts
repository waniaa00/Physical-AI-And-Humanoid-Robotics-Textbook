/**
 * API Configuration
 *
 * Safely handles environment variables in browser context.
 * Docusaurus runs in the browser where process.env is not available by default.
 */

// Default to localhost for development
export const API_BASE_URL = 'http://localhost:8000';

// For production, you can update this or use Docusaurus custom fields
export const getApiUrl = (): string => {
  // Check if running in browser and if a custom URL is set
  if (typeof window !== 'undefined' && (window as any).__API_URL__) {
    return (window as any).__API_URL__;
  }
  return API_BASE_URL;
};
