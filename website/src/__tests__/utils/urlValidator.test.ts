/**
 * URL Validator Tests
 * Tests URL validation and sanitization to prevent security vulnerabilities
 */

import {
  isValidRedirectUrl,
  isValidUrlScheme,
  sanitizeRedirectUrl,
  getValidatedReturnUrl,
} from '@site/src/utils/urlValidator';

describe('URL Validator', () => {
  describe('isValidUrlScheme', () => {
    it('should allow relative URLs', () => {
      expect(isValidUrlScheme('/')).toBe(true);
      expect(isValidUrlScheme('/profile')).toBe(true);
      expect(isValidUrlScheme('/personalized-content')).toBe(true);
    });

    it('should allow http and https URLs', () => {
      expect(isValidUrlScheme('http://example.com')).toBe(true);
      expect(isValidUrlScheme('https://example.com')).toBe(true);
    });

    it('should allow anchor links', () => {
      expect(isValidUrlScheme('#section')).toBe(true);
      expect(isValidUrlScheme('#')).toBe(true);
    });

    it('should block javascript: URLs', () => {
      expect(isValidUrlScheme('javascript:alert(1)')).toBe(false);
      expect(isValidUrlScheme('JavaScript:alert(1)')).toBe(false);
      expect(isValidUrlScheme('JAVASCRIPT:alert(1)')).toBe(false);
    });

    it('should block data: URLs', () => {
      expect(isValidUrlScheme('data:text/html,<script>alert(1)</script>')).toBe(false);
      expect(isValidUrlScheme('DATA:text/html,test')).toBe(false);
    });

    it('should block vbscript: URLs', () => {
      expect(isValidUrlScheme('vbscript:msgbox(1)')).toBe(false);
    });

    it('should block file: URLs', () => {
      expect(isValidUrlScheme('file:///etc/passwd')).toBe(false);
    });

    it('should block about: URLs', () => {
      expect(isValidUrlScheme('about:blank')).toBe(false);
    });

    it('should handle empty or invalid input', () => {
      expect(isValidUrlScheme('')).toBe(false);
      expect(isValidUrlScheme('   ')).toBe(false);
    });
  });

  describe('isValidRedirectUrl', () => {
    beforeEach(() => {
      // Mock window.location for testing
      delete (window as any).location;
      (window as any).location = {
        origin: 'http://localhost:3000',
        pathname: '/',
      };
    });

    it('should allow whitelisted paths', () => {
      expect(isValidRedirectUrl('/')).toBe(true);
      expect(isValidRedirectUrl('/personalized-content')).toBe(true);
      expect(isValidRedirectUrl('/profile')).toBe(true);
      expect(isValidRedirectUrl('/docs')).toBe(true);
      expect(isValidRedirectUrl('/docs/intro')).toBe(true);
      expect(isValidRedirectUrl('/signin')).toBe(true);
      expect(isValidRedirectUrl('/signup')).toBe(true);
    });

    it('should allow paths under whitelisted directories', () => {
      expect(isValidRedirectUrl('/docs/module1/chapter1')).toBe(true);
      expect(isValidRedirectUrl('/docs/advanced/tutorial')).toBe(true);
    });

    it('should block non-whitelisted paths', () => {
      expect(isValidRedirectUrl('/admin')).toBe(false);
      expect(isValidRedirectUrl('/api/secret')).toBe(false);
      expect(isValidRedirectUrl('/malicious')).toBe(false);
    });

    it('should allow same-origin absolute URLs', () => {
      expect(isValidRedirectUrl('http://localhost:3000/personalized-content')).toBe(true);
      expect(isValidRedirectUrl('http://localhost:3000/profile')).toBe(true);
    });

    it('should block cross-origin URLs', () => {
      expect(isValidRedirectUrl('http://evil.com/phishing')).toBe(false);
      expect(isValidRedirectUrl('https://evil.com/steal-data')).toBe(false);
    });

    it('should handle edge cases', () => {
      expect(isValidRedirectUrl('')).toBe(false);
      expect(isValidRedirectUrl('   ')).toBe(false);
    });

    it('should trim whitespace', () => {
      expect(isValidRedirectUrl('  /personalized-content  ')).toBe(true);
    });
  });

  describe('sanitizeRedirectUrl', () => {
    it('should return valid URLs unchanged', () => {
      expect(sanitizeRedirectUrl('/personalized-content')).toBe('/personalized-content');
      expect(sanitizeRedirectUrl('/profile')).toBe('/profile');
      expect(sanitizeRedirectUrl('/')).toBe('/');
    });

    it('should return default for invalid URLs', () => {
      expect(sanitizeRedirectUrl('javascript:alert(1)')).toBe('/');
      expect(sanitizeRedirectUrl('/admin')).toBe('/');
      expect(sanitizeRedirectUrl('http://evil.com')).toBe('/');
    });

    it('should use custom default URL', () => {
      expect(sanitizeRedirectUrl('javascript:alert(1)', '/signin')).toBe('/signin');
      expect(sanitizeRedirectUrl('/admin', '/home')).toBe('/home');
    });

    it('should handle empty input', () => {
      expect(sanitizeRedirectUrl('')).toBe('/');
      expect(sanitizeRedirectUrl('', '/default')).toBe('/default');
    });
  });

  describe('getValidatedReturnUrl', () => {
    it('should extract and validate returnTo from URLSearchParams', () => {
      const params = new URLSearchParams('returnTo=/personalized-content');
      expect(getValidatedReturnUrl(params)).toBe('/personalized-content');
    });

    it('should extract and validate returnTo from query string', () => {
      expect(getValidatedReturnUrl('returnTo=/profile')).toBe('/profile');
      expect(getValidatedReturnUrl('foo=bar&returnTo=/docs')).toBe('/docs');
    });

    it('should return default for missing returnTo', () => {
      const params = new URLSearchParams('foo=bar');
      expect(getValidatedReturnUrl(params)).toBe('/');
      expect(getValidatedReturnUrl('')).toBe('/');
    });

    it('should sanitize invalid returnTo values', () => {
      expect(getValidatedReturnUrl('returnTo=javascript:alert(1)')).toBe('/');
      expect(getValidatedReturnUrl('returnTo=/admin')).toBe('/');
      expect(getValidatedReturnUrl('returnTo=http://evil.com')).toBe('/');
    });

    it('should handle URL-encoded values', () => {
      const params = new URLSearchParams('returnTo=%2Fpersonalized-content');
      expect(getValidatedReturnUrl(params)).toBe('/personalized-content');
    });
  });

  describe('XSS Prevention', () => {
    it('should prevent XSS via javascript: scheme', () => {
      const maliciousUrls = [
        'javascript:alert(document.cookie)',
        'javascript:void(0)',
        'JavaScript:alert(1)',
        'JAVASCRIPT:alert(1)',
      ];

      maliciousUrls.forEach(url => {
        expect(isValidUrlScheme(url)).toBe(false);
        expect(sanitizeRedirectUrl(url)).toBe('/');
      });
    });

    it('should prevent XSS via data: scheme', () => {
      const maliciousUrls = [
        'data:text/html,<script>alert(1)</script>',
        'data:text/html;base64,PHNjcmlwdD5hbGVydCgxKTwvc2NyaXB0Pg==',
      ];

      maliciousUrls.forEach(url => {
        expect(isValidUrlScheme(url)).toBe(false);
        expect(sanitizeRedirectUrl(url)).toBe('/');
      });
    });
  });

  describe('Open Redirect Prevention', () => {
    it('should prevent open redirects to external sites', () => {
      const externalUrls = [
        'http://evil.com/phishing',
        'https://malicious.site/steal',
        '//evil.com/redirect',
      ];

      externalUrls.forEach(url => {
        expect(sanitizeRedirectUrl(url)).toBe('/');
      });
    });

    it('should prevent redirects to non-whitelisted paths', () => {
      const nonWhitelistedPaths = [
        '/admin',
        '/api/secret',
        '/internal/config',
        '/../../../etc/passwd',
      ];

      nonWhitelistedPaths.forEach(path => {
        expect(sanitizeRedirectUrl(path)).toBe('/');
      });
    });
  });
});
