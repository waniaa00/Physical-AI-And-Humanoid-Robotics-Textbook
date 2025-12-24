/**
 * E2E tests for session expiry handling (T027)
 *
 * Test scenarios:
 * 1. Authenticated user with expired session is redirected to signin
 * 2. Session expiry during page view triggers redirect
 * 3. Return URL is preserved after session expiry
 * 4. User can re-authenticate and return to original page
 */

import { test, expect } from '@playwright/test';

test.describe('Session Expiry Handling', () => {
  test('should redirect to signin when session expires', async ({ page, context }) => {
    // First, sign in to establish a session
    await page.goto('/signin');
    await page.getByLabel(/email/i).fill('test@example.com');
    await page.getByLabel(/password/i).fill('password123');
    await page.getByRole('button', { name: /sign in/i }).click();

    // Wait for successful signin
    await page.waitForTimeout(1000);

    // Verify we can access personalized content
    await page.goto('/personalized-content');
    await expect(page).toHaveURL('/personalized-content');

    // Simulate session expiry by clearing session tokens
    await context.clearCookies();
    await page.evaluate(() => {
      // Clear session_token and any auth-related localStorage
      localStorage.removeItem('session_token');
      localStorage.removeItem('user_id');
      localStorage.removeItem('user_has_interests');
    });

    // Try to reload the page (simulates user refreshing or navigating)
    await page.reload();

    // Should be redirected to signin due to expired session
    await expect(page).toHaveURL(/\/signin/, { timeout: 5000 });

    // Should preserve return URL
    const url = new URL(page.url());
    expect(url.searchParams.get('returnTo')).toBe('/personalized-content');
  });

  test('should handle session expiry during navigation', async ({ page, context }) => {
    // Sign in first
    await page.goto('/signin');
    await page.getByLabel(/email/i).fill('test@example.com');
    await page.getByLabel(/password/i).fill('password123');
    await page.getByRole('button', { name: /sign in/i }).click();

    // Wait for signin to complete
    await page.waitForTimeout(1000);

    // Navigate to docs (public page)
    await page.goto('/docs/intro');
    await expect(page).toHaveURL('/docs/intro');

    // Simulate session expiry
    await context.clearCookies();
    await page.evaluate(() => {
      localStorage.removeItem('session_token');
      localStorage.removeItem('user_id');
    });

    // Now try to access protected page
    await page.goto('/personalized-content');

    // Should be redirected to signin
    await expect(page).toHaveURL(/\/signin/, { timeout: 5000 });
  });

  test('should allow re-authentication after session expiry', async ({ page, context }) => {
    // Sign in
    await page.goto('/signin');
    await page.getByLabel(/email/i).fill('test@example.com');
    await page.getByLabel(/password/i).fill('password123');
    await page.getByRole('button', { name: /sign in/i }).click();
    await page.waitForTimeout(1000);

    // Access personalized content
    await page.goto('/personalized-content');
    await expect(page).toHaveURL('/personalized-content');

    // Simulate session expiry
    await context.clearCookies();
    await page.evaluate(() => {
      localStorage.clear();
    });

    // Reload page to trigger auth check
    await page.reload();

    // Should be at signin with returnTo parameter
    await expect(page).toHaveURL(/\/signin\?returnTo=%2Fpersonalized-content/);

    // Re-authenticate
    await page.getByLabel(/email/i).fill('test@example.com');
    await page.getByLabel(/password/i).fill('password123');
    await page.getByRole('button', { name: /sign in/i }).click();

    // Should be redirected back to personalized content
    await expect(page).toHaveURL('/personalized-content', { timeout: 10000 });
  });

  test('should show appropriate message during session expiry redirect', async ({ page, context }) => {
    // Sign in
    await page.goto('/signin');
    await page.getByLabel(/email/i).fill('test@example.com');
    await page.getByLabel(/password/i).fill('password123');
    await page.getByRole('button', { name: /sign in/i }).click();
    await page.waitForTimeout(1000);

    // Clear session to simulate expiry
    await context.clearCookies();
    await page.evaluate(() => {
      localStorage.clear();
    });

    // Try to access personalized content
    await page.goto('/personalized-content');

    // Should show redirecting message
    await expect(page.getByText(/redirecting to sign in/i)).toBeVisible({ timeout: 3000 });
  });

  test('should handle missing session token gracefully', async ({ page }) => {
    // Don't sign in, just set some localStorage data without session token
    await page.goto('/');
    await page.evaluate(() => {
      localStorage.setItem('user_id', 'test-user-123');
      // Intentionally NOT setting session_token to simulate invalid state
    });

    // Try to access personalized content
    await page.goto('/personalized-content');

    // Should be redirected to signin
    await expect(page).toHaveURL(/\/signin/, { timeout: 5000 });

    // Should preserve return URL
    const url = new URL(page.url());
    expect(url.searchParams.get('returnTo')).toBe('/personalized-content');
  });

  test('should handle corrupted session data', async ({ page, context }) => {
    // Set up corrupted session (invalid token format)
    await page.goto('/');
    await page.evaluate(() => {
      localStorage.setItem('session_token', 'invalid-corrupted-token');
      localStorage.setItem('user_id', 'test-user');
    });

    // Set a cookie with invalid session data
    await context.addCookies([
      {
        name: 'session_token',
        value: 'corrupted-token-data',
        domain: 'localhost',
        path: '/',
      },
    ]);

    // Try to access personalized content
    await page.goto('/personalized-content');

    // Should be redirected to signin (auth validation should fail)
    await expect(page).toHaveURL(/\/signin/, { timeout: 5000 });
  });
});
