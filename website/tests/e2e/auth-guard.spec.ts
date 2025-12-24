/**
 * E2E tests for personalized content auth guard (T026)
 *
 * Test scenarios:
 * 1. Unauthenticated user accessing /personalized-content is redirected to /signin
 * 2. Return URL parameter is preserved during redirect
 * 3. After successful signin, user is redirected back to /personalized-content
 * 4. Authenticated user can access /personalized-content directly
 */

import { test, expect } from '@playwright/test';

test.describe('Personalized Content Auth Guard', () => {
  test.beforeEach(async ({ page, context }) => {
    // Clear all cookies and localStorage to ensure unauthenticated state
    await context.clearCookies();
    await page.goto('/');
    await page.evaluate(() => {
      localStorage.clear();
      sessionStorage.clear();
    });
  });

  test('should redirect unauthenticated user to signin page', async ({ page }) => {
    // Attempt to access personalized content without authentication
    await page.goto('/personalized-content');

    // Should be redirected to signin page
    await expect(page).toHaveURL(/\/signin/);

    // Should show signin page content
    await expect(page.getByRole('heading', { name: /sign in/i })).toBeVisible();
  });

  test('should preserve return URL in redirect', async ({ page }) => {
    // Attempt to access personalized content without authentication
    await page.goto('/personalized-content');

    // Wait for redirect to signin
    await page.waitForURL(/\/signin/);

    // URL should contain returnTo parameter
    const url = new URL(page.url());
    expect(url.searchParams.get('returnTo')).toBe('/personalized-content');
  });

  test('should redirect back to personalized content after signin', async ({ page }) => {
    // Navigate to signin with returnTo parameter
    await page.goto('/signin?returnTo=/personalized-content');

    // Fill in signin form
    await page.getByLabel(/email/i).fill('test@example.com');
    await page.getByLabel(/password/i).fill('password123');

    // Submit form
    await page.getByRole('button', { name: /sign in/i }).click();

    // Should be redirected to personalized content page
    await expect(page).toHaveURL('/personalized-content', { timeout: 10000 });

    // Should show personalized content (if user has interests)
    // OR show interest selection prompt (if user has no interests)
    await expect(
      page.getByRole('heading', { name: /personalized content|select your interests/i })
    ).toBeVisible({ timeout: 5000 });
  });

  test('should allow authenticated user to access personalized content', async ({ page }) => {
    // First, sign in
    await page.goto('/signin');
    await page.getByLabel(/email/i).fill('test@example.com');
    await page.getByLabel(/password/i).fill('password123');
    await page.getByRole('button', { name: /sign in/i }).click();

    // Wait for signin to complete
    await page.waitForTimeout(1000);

    // Now try to access personalized content
    await page.goto('/personalized-content');

    // Should NOT be redirected to signin
    await expect(page).toHaveURL('/personalized-content');

    // Should show either personalized content or interest selection prompt
    await expect(
      page.getByRole('heading', { name: /personalized content|select your interests/i })
    ).toBeVisible();
  });

  test('should not show personalized content in redirect state', async ({ page }) => {
    // Attempt to access personalized content without authentication
    await page.goto('/personalized-content');

    // Should show redirecting message, not actual content
    await expect(page.getByText(/redirecting to sign in/i)).toBeVisible();

    // Should NOT show personalized content elements
    await expect(
      page.getByRole('heading', { name: /your personalized content/i })
    ).not.toBeVisible();
  });

  test('should handle direct URL access without session', async ({ page }) => {
    // Clear any existing session
    await page.goto('/');
    await page.evaluate(() => {
      localStorage.clear();
      sessionStorage.clear();
      document.cookie.split(';').forEach(c => {
        document.cookie = c.replace(/^ +/, '').replace(/=.*/, '=;expires=' + new Date().toUTCString() + ';path=/');
      });
    });

    // Try to access personalized content with direct URL
    await page.goto('/personalized-content');

    // Should be redirected to signin
    await expect(page).toHaveURL(/\/signin/, { timeout: 5000 });

    // Should preserve the return URL
    const url = new URL(page.url());
    expect(url.searchParams.get('returnTo')).toBe('/personalized-content');
  });
});
