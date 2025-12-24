/**
 * Configuration utilities
 * Access environment variables and site config
 */
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Production backend URL
export const BACKEND_URL = 'https://wnxddev-humanoid-robotics-api.hf.space';

export function useBackendUrl(): string {
  const { siteConfig } = useDocusaurusContext();
  return (siteConfig.customFields?.backendUrl as string) || BACKEND_URL;
}

// Non-hook version for use outside React components
export function getBackendUrl(): string {
  if (typeof window !== 'undefined' && (window as any).__DOCUSAURUS_CONFIG__) {
    return (window as any).__DOCUSAURUS_CONFIG__.customFields?.backendUrl || BACKEND_URL;
  }
  return BACKEND_URL;
}

export const API_ENDPOINTS = {
  CHAT: '/agent/chat',
  SEARCH: '/search',
  HEALTH: '/health',
} as const;

export const REQUEST_TIMEOUT = 30000; // 30 seconds
export const MAX_RETRIES = 2;
