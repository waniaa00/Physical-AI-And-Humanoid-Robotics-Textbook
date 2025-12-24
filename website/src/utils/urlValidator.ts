/**
 * URL Validation Utilities
 * Validates URLs to prevent open redirect vulnerabilities
 */

/**
 * Whitelist of allowed redirect paths
 * Add new paths here as needed
 */
const ALLOWED_REDIRECT_PATHS = [
  '/',
  '/personalized-content',
  '/profile',
  '/docs',
  '/docs/intro',
  '/signin',
  '/signup',
] as const;

/**
 * Validates if a redirect URL is safe to use
 * Prevents open redirect vulnerabilities
 *
 * @param url - The URL to validate
 * @returns true if URL is safe, false otherwise
 */
export function isValidRedirectUrl(url: string): boolean {
  if (!url) return false;

  try {
    // Remove leading/trailing whitespace
    const trimmedUrl = url.trim();

    // Check if it's a relative path (starts with /)
    if (trimmedUrl.startsWith('/')) {
      // Check against whitelist
      const isWhitelisted = ALLOWED_REDIRECT_PATHS.some(path =>
        trimmedUrl === path || trimmedUrl.startsWith(`${path}/`)
      );

      if (!isWhitelisted) {
        console.warn(`[URL Validator] Blocked non-whitelisted path: ${trimmedUrl}`);
        return false;
      }

      return true;
    }

    // Parse absolute URLs
    const parsedUrl = new URL(trimmedUrl);

    // Check path against whitelist first
    const isWhitelisted = ALLOWED_REDIRECT_PATHS.some(path =>
      parsedUrl.pathname === path || parsedUrl.pathname.startsWith(`${path}/`)
    );

    if (!isWhitelisted) {
      console.warn(`[URL Validator] Blocked non-whitelisted absolute path: ${parsedUrl.pathname}`);
      return false;
    }

    // In browser environment, verify same-origin
    if (typeof window !== 'undefined' && window.location) {
      const currentOrigin = window.location.origin;

      // Check if origins match
      if (parsedUrl.origin !== currentOrigin) {
        // Special case: localhost with different ports should be allowed in dev/test
        // (jsdom doesn't preserve port in window.location.origin)
        const isCurrentLocalhost = currentOrigin.includes('localhost') || currentOrigin.includes('127.0.0.1');
        const isParsedLocalhost = parsedUrl.hostname === 'localhost' || parsedUrl.hostname === '127.0.0.1' || parsedUrl.hostname === '[::1]';

        if (!(isCurrentLocalhost && isParsedLocalhost)) {
          console.warn(`[URL Validator] Blocked cross-origin redirect: ${parsedUrl.origin} (current: ${currentOrigin})`);
          return false;
        }
      }
      return true;
    }

    // In test/server environment, only allow localhost URLs
    const isLocalhost = parsedUrl.hostname === 'localhost' ||
                       parsedUrl.hostname === '127.0.0.1' ||
                       parsedUrl.hostname === '[::1]';

    if (!isLocalhost) {
      console.warn(`[URL Validator] Blocked non-localhost URL in server environment: ${parsedUrl.hostname}`);
      return false;
    }

    return true;
  } catch (error) {
    console.error('[URL Validator] Invalid URL:', error);
    return false;
  }
}

/**
 * Validates URL scheme to prevent javascript: and data: URLs
 *
 * @param url - The URL to validate
 * @returns true if scheme is safe, false otherwise
 */
export function isValidUrlScheme(url: string): boolean {
  if (!url) return false;

  const trimmedUrl = url.trim().toLowerCase();

  // Dangerous schemes
  const dangerousSchemes = [
    'javascript:',
    'data:',
    'vbscript:',
    'file:',
    'about:',
  ];

  for (const scheme of dangerousSchemes) {
    if (trimmedUrl.startsWith(scheme)) {
      console.warn(`[URL Validator] Blocked dangerous scheme: ${scheme}`);
      return false;
    }
  }

  // Block protocol-relative URLs (e.g., //evil.com)
  // IMPORTANT: Check this BEFORE allowing relative URLs
  if (trimmedUrl.startsWith('//')) {
    console.warn('[URL Validator] Blocked protocol-relative URL (open redirect attempt)');
    return false;
  }

  // Allow relative URLs (start with /)
  if (trimmedUrl.startsWith('/')) {
    return true;
  }

  // Allow http and https
  if (trimmedUrl.startsWith('http://') || trimmedUrl.startsWith('https://')) {
    return true;
  }

  // Allow anchor links
  if (trimmedUrl.startsWith('#')) {
    return true;
  }

  // Block everything else
  console.warn(`[URL Validator] Blocked unknown URL format: ${trimmedUrl}`);
  return false;
}

/**
 * Sanitizes a redirect URL, returning a safe default if invalid
 *
 * @param url - The URL to sanitize
 * @param defaultUrl - Default URL if validation fails (default: '/')
 * @returns Safe redirect URL
 */
export function sanitizeRedirectUrl(url: string, defaultUrl: string = '/'): string {
  if (!url) return defaultUrl;

  // Validate URL scheme first
  if (!isValidUrlScheme(url)) {
    return defaultUrl;
  }

  // Validate against whitelist
  if (!isValidRedirectUrl(url)) {
    return defaultUrl;
  }

  return url;
}

/**
 * Extracts and validates the returnTo parameter from URL
 *
 * @param searchParams - URLSearchParams or query string
 * @returns Validated return URL or default '/'
 */
export function getValidatedReturnUrl(searchParams: URLSearchParams | string): string {
  let params: URLSearchParams;

  if (typeof searchParams === 'string') {
    params = new URLSearchParams(searchParams);
  } else {
    params = searchParams;
  }

  const returnTo = params.get('returnTo');

  if (!returnTo) {
    return '/';
  }

  return sanitizeRedirectUrl(returnTo, '/');
}
