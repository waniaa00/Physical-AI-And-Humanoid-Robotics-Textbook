/**
 * Configuration utilities
 * Access environment variables and site config
 */
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export function useBackendUrl(): string {
  const { siteConfig } = useDocusaurusContext();
  return (siteConfig.customFields?.backendUrl as string) || 'http://localhost:8000';
}

export const API_ENDPOINTS = {
  CHAT: '/agent/chat',
  SEARCH: '/search',
  HEALTH: '/health',
} as const;

export const REQUEST_TIMEOUT = 30000; // 30 seconds
export const MAX_RETRIES = 2;
