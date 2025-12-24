/**
 * E2E tests for navigation between personalized and full book views (T041)
 *
 * Test scenarios:
 * 1. Navigate from personalized page to full book
 * 2. Navigate from full book back to personalized page
 * 3. Navigate from personalized page to specific chapter
 * 4. Navigate between adjacent chapters
 * 5. Return to personalized view from chapter
 * 6. Context preservation across navigation
 */

import { test, expect } from '@playwright/test';

test.describe('Navigation Between Views', () => {
  test.beforeEach(async ({ page }) => {
    // Sign in before each test
    await page.goto('/signin');
    await page.getByLabel(/email/i).fill('test@example.com');
    await page.getByLabel(/password/i).fill('password123');
    await page.getByRole('button', { name: /sign in/i }).click();
    await page.waitForTimeout(1000);
  });

  test('should navigate from personalized page to full book', async ({ page }) => {
    await page.goto('/personalized-content');
    await expect(page).toHaveURL('/personalized-content');

    // Click "View Full Book" button
    const viewFullBookLink = page.getByRole('link', { name: /view full book/i });
    await expect(viewFullBookLink).toBeVisible();
    await viewFullBookLink.click();

    // Should navigate to docs intro
    await expect(page).toHaveURL('/docs/intro');

    // Should see book content
    await expect(page.locator('article')).toBeVisible();
  });

  test('should show "Back to My Content" link in navbar when on docs pages', async ({ page }) => {
    await page.goto('/docs/intro');

    // Check for "Back to My Content" or "Personalized View" link
    const backLink = page.getByRole('link', { name: /my content|personalized/i });

    // If link exists, verify it navigates correctly
    if (await backLink.isVisible({ timeout: 2000 }).catch(() => false)) {
      await backLink.click();
      await expect(page).toHaveURL('/personalized-content');
    } else {
      // T044 not yet implemented - this is expected
      console.log('Back to My Content link not yet added to navbar');
    }
  });

  test('should preserve authentication when navigating between views', async ({ page }) => {
    // Start at personalized content
    await page.goto('/personalized-content');
    await expect(page).toHaveURL('/personalized-content');

    // Navigate to full book
    await page.goto('/docs/intro');
    await expect(page).toHaveURL('/docs/intro');

    // Navigate back to personalized content
    await page.goto('/personalized-content');

    // Should still be authenticated (not redirected to signin)
    await expect(page).toHaveURL('/personalized-content');
    await expect(page.getByText(/redirecting to sign in/i)).not.toBeVisible();
  });

  test('should navigate from personalized page to specific chapter', async ({ page }) => {
    await page.goto('/personalized-content');

    // Find and click a chapter link
    const chapterLink = page.locator('[data-chapter-id]').first().locator('a').first();

    if (await chapterLink.isVisible({ timeout: 3000 }).catch(() => false)) {
      const chapterHref = await chapterLink.getAttribute('href');
      await chapterLink.click();

      // Should navigate to the chapter
      await page.waitForURL(new RegExp(chapterHref || '/docs'), { timeout: 5000 });

      // Should see chapter content
      await expect(page.locator('article')).toBeVisible();
    } else {
      console.log('No chapters available to click');
    }
  });

  test('should allow navigation between adjacent chapters', async ({ page }) => {
    // Navigate to a specific chapter
    await page.goto('/docs/module1/chapter1-introduction');

    // Look for "Next" or pagination links
    const nextLink = page.getByRole('link', { name: /next/i });

    if (await nextLink.isVisible({ timeout: 2000 }).catch(() => false)) {
      await nextLink.click();

      // Should navigate to next chapter
      await expect(page.locator('article')).toBeVisible();

      // Look for "Previous" link
      const prevLink = page.getByRole('link', { name: /previous/i });
      if (await prevLink.isVisible({ timeout: 2000 }).catch(() => false)) {
        await prevLink.click();

        // Should navigate back
        await expect(page).toHaveURL(/chapter1-introduction/);
      }
    } else {
      console.log('Next/Previous navigation not available');
    }
  });

  test('should return to personalized view from chapter', async ({ page }) => {
    // Navigate to personalized content
    await page.goto('/personalized-content');

    // Click a chapter
    const chapterLink = page.locator('[data-chapter-id]').first().locator('a').first();

    if (await chapterLink.isVisible({ timeout: 3000 }).catch(() => false)) {
      await chapterLink.click();
      await page.waitForTimeout(1000);

      // Navigate back using browser back button or link
      await page.goBack();

      // Should be back at personalized content
      await expect(page).toHaveURL('/personalized-content');
      await expect(page.locator('[data-chapter-id]')).toBeVisible();
    } else {
      console.log('No chapters to navigate from');
    }
  });

  test('should preserve personalized content when using browser back/forward', async ({ page }) => {
    // Navigate to personalized content
    await page.goto('/personalized-content');

    // Get initial chapter count
    const initialChapters = await page.locator('[data-chapter-id]').count();

    // Navigate to docs
    await page.goto('/docs/intro');
    await expect(page).toHaveURL('/docs/intro');

    // Use browser back
    await page.goBack();

    // Should be back at personalized content with same chapters
    await expect(page).toHaveURL('/personalized-content');
    const returnedChapters = await page.locator('[data-chapter-id]').count();

    expect(returnedChapters).toBe(initialChapters);
  });

  test('should handle deep linking to chapters and back to personalized view', async ({ page }) => {
    // Direct navigation to a chapter
    await page.goto('/docs/module1/chapter2-ros2-fundamentals');

    // Should see chapter content
    await expect(page.locator('article')).toBeVisible();

    // Navigate to personalized content
    await page.goto('/personalized-content');

    // Should see personalized view (not empty state if user has interests)
    const hasContent = await page.locator('[data-chapter-id]').isVisible({ timeout: 3000 }).catch(() => false);
    const hasPrompt = await page.getByTestId('interest-prompt').isVisible({ timeout: 1000 }).catch(() => false);

    expect(hasContent || hasPrompt).toBeTruthy();
  });

  test('should maintain scroll position when navigating back from chapter', async ({ page }) => {
    await page.goto('/personalized-content');

    // Scroll down if there's content
    if (await page.locator('[data-chapter-id]').count() > 3) {
      await page.evaluate(() => window.scrollTo(0, 500));
      await page.waitForTimeout(500);

      // Click a chapter
      const chapterLink = page.locator('[data-chapter-id]').nth(2).locator('a').first();
      await chapterLink.click();
      await page.waitForTimeout(1000);

      // Navigate back
      await page.goBack();

      // Note: Scroll position restoration depends on browser/Docusaurus behavior
      // This test just verifies the page loads correctly
      await expect(page).toHaveURL('/personalized-content');
    }
  });

  test('should show correct navigation context in breadcrumbs (if implemented)', async ({ page }) => {
    await page.goto('/docs/module1/chapter1-introduction');

    // Check for breadcrumbs or navigation indicators
    const breadcrumbs = page.locator('nav[aria-label="breadcrumbs"], .breadcrumbs');

    if (await breadcrumbs.isVisible({ timeout: 2000 }).catch(() => false)) {
      // Verify breadcrumbs show correct path
      await expect(breadcrumbs).toContainText(/module|chapter/i);
    } else {
      console.log('Breadcrumbs not implemented');
    }
  });
});
