/**
 * Security Headers Configuration
 * Defines security headers for production deployment
 *
 * These headers should be configured in your hosting platform:
 * - Vercel: vercel.json
 * - Netlify: netlify.toml or _headers file
 * - Apache: .htaccess
 * - Nginx: nginx.conf
 * - Express: app.use() middleware
 */

/**
 * Content Security Policy (CSP)
 * Protects against XSS, clickjacking, and other injection attacks
 *
 * Adjusted for Docusaurus requirements:
 * - 'unsafe-inline' for styles (Docusaurus needs this)
 * - 'unsafe-eval' for scripts (Docusaurus build process)
 */
export const CONTENT_SECURITY_POLICY = [
  "default-src 'self'",
  "script-src 'self' 'unsafe-inline' 'unsafe-eval'", // Docusaurus requires unsafe-eval
  "style-src 'self' 'unsafe-inline'", // Docusaurus requires unsafe-inline for styles
  "img-src 'self' data: https:", // Allow images from self, data URIs, and HTTPS
  "font-src 'self' data:", // Allow fonts from self and data URIs
  "connect-src 'self' http://localhost:* ws://localhost:*", // Allow local API and WebSocket
  "frame-ancestors 'none'", // Prevent clickjacking
  "base-uri 'self'", // Restrict base tag URLs
  "form-action 'self'", // Only allow forms to submit to same origin
].join('; ');

/**
 * Strict-Transport-Security (HSTS)
 * Forces HTTPS connections for improved security
 *
 * max-age: 1 year (31536000 seconds)
 * includeSubDomains: Apply to all subdomains
 * preload: Allow inclusion in browser HSTS preload list
 */
export const HSTS_HEADER = 'max-age=31536000; includeSubDomains; preload';

/**
 * X-Content-Type-Options
 * Prevents MIME type sniffing
 */
export const X_CONTENT_TYPE_OPTIONS = 'nosniff';

/**
 * X-Frame-Options
 * Prevents clickjacking attacks
 */
export const X_FRAME_OPTIONS = 'DENY';

/**
 * X-XSS-Protection
 * Enables browser's XSS filter (legacy browsers)
 * Modern browsers rely on CSP instead
 */
export const X_XSS_PROTECTION = '1; mode=block';

/**
 * Referrer-Policy
 * Controls how much referrer information is sent
 */
export const REFERRER_POLICY = 'strict-origin-when-cross-origin';

/**
 * Permissions-Policy
 * Controls which browser features can be used
 */
export const PERMISSIONS_POLICY = [
  'camera=()',
  'microphone=()',
  'geolocation=()',
  'interest-cohort=()', // Disable FLoC
].join(', ');

/**
 * Complete security headers object
 * Ready to use in various deployment platforms
 */
export const SECURITY_HEADERS = {
  'Content-Security-Policy': CONTENT_SECURITY_POLICY,
  'Strict-Transport-Security': HSTS_HEADER,
  'X-Content-Type-Options': X_CONTENT_TYPE_OPTIONS,
  'X-Frame-Options': X_FRAME_OPTIONS,
  'X-XSS-Protection': X_XSS_PROTECTION,
  'Referrer-Policy': REFERRER_POLICY,
  'Permissions-Policy': PERMISSIONS_POLICY,
};

/**
 * Vercel Configuration (vercel.json)
 */
export const VERCEL_HEADERS_CONFIG = {
  headers: [
    {
      source: '/(.*)',
      headers: Object.entries(SECURITY_HEADERS).map(([key, value]) => ({
        key,
        value,
      })),
    },
  ],
};

/**
 * Netlify Configuration (_headers file content)
 */
export const NETLIFY_HEADERS_CONFIG = `/*
  Content-Security-Policy: ${CONTENT_SECURITY_POLICY}
  Strict-Transport-Security: ${HSTS_HEADER}
  X-Content-Type-Options: ${X_CONTENT_TYPE_OPTIONS}
  X-Frame-Options: ${X_FRAME_OPTIONS}
  X-XSS-Protection: ${X_XSS_PROTECTION}
  Referrer-Policy: ${REFERRER_POLICY}
  Permissions-Policy: ${PERMISSIONS_POLICY}
`;

/**
 * Express.js Middleware
 * Use with: app.use(applySecurityHeaders)
 */
export function applySecurityHeaders(req: any, res: any, next: any): void {
  Object.entries(SECURITY_HEADERS).forEach(([header, value]) => {
    res.setHeader(header, value);
  });
  next();
}

/**
 * CSP Meta Tag (fallback)
 * Use in HTML <head> if headers can't be configured
 * Note: Less secure than HTTP headers
 */
export const CSP_META_TAG = `<meta http-equiv="Content-Security-Policy" content="${CONTENT_SECURITY_POLICY}">`;
