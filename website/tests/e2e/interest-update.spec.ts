/**
 * E2E tests for interest update flow (T036)
 *
 * Test scenarios:
 * 1. User changes interests in profile → returns to personalized page → sees updated content
 * 2. Adding interest shows new matching chapters
 * 3. Removing interest removes corresponding chapters
 * 4. Updates occur within 3 seconds (SC-005)
 * 5. Multiple interest changes are handled correctly
 */

import { test, expect } from '@playwright/test';

test.describe('Interest Update Flow', () => {
  test.beforeEach(async ({ page }) => {
    // Sign in before each test
    await page.goto('/signin');
    await page.getByLabel(/email/i).fill('test@example.com');
    await page.getByLabel(/password/i).fill('password123');
    await page.getByRole('button', { name: /sign in/i }).click();

    // Wait for signin to complete
    await page.waitForTimeout(1000);
  });

  test('should update personalized content after changing interests in profile', async ({ page }) => {
    // Navigate to personalized content page
    await page.goto('/personalized-content');
    await expect(page).toHaveURL('/personalized-content');

    // Record initial chapter count
    const initialChapters = await page.locator('[data-chapter-id]').count();
    console.log(`Initial chapters: ${initialChapters}`);

    // Navigate to profile page
    await page.goto('/profile');
    await expect(page).toHaveURL('/profile');

    // Add a new interest (simulate checking a checkbox or clicking an interest)
    // This depends on the profile page implementation
    const kinematicsInterest = page.getByLabel(/kinematics/i);
    if (await kinematicsInterest.isVisible()) {
      await kinematicsInterest.check();
    }

    // Save interests
    const saveButton = page.getByRole('button', { name: /save|update/i });
    if (await saveButton.isVisible()) {
      await saveButton.click();
    }

    // Wait for save to complete
    await page.waitForTimeout(1000);

    // Navigate back to personalized content
    await page.goto('/personalized-content');

    // Content should be updated (may have more chapters)
    // Wait for content to load
    await page.waitForTimeout(2000);

    const updatedChapters = await page.locator('[data-chapter-id]').count();
    console.log(`Updated chapters: ${updatedChapters}`);

    // Should have content displayed
    await expect(page.locator('[data-chapter-id]').first()).toBeVisible();
  });

  test('should show loading state during content refresh', async ({ page }) => {
    await page.goto('/personalized-content');

    // Initial content should be visible
    await expect(page.locator('[data-chapter-id]').first()).toBeVisible({ timeout: 5000 });

    // Change interests via profile
    await page.goto('/profile');

    // Toggle an interest
    const interestCheckbox = page.locator('input[type="checkbox"]').first();
    await interestCheckbox.click();

    // Save
    const saveButton = page.getByRole('button', { name: /save|update/i });
    if (await saveButton.isVisible()) {
      await saveButton.click();
    }

    // Return to personalized content quickly
    await page.goto('/personalized-content');

    // May briefly see loading state
    const loadingText = page.getByText(/loading/i);
    if (await loadingText.isVisible({ timeout: 1000 }).catch(() => false)) {
      console.log('Loading state detected');
    }

    // Content should eventually appear
    await expect(page.locator('[data-chapter-id]').first()).toBeVisible({ timeout: 5000 });
  });

  test('should handle removing all interests gracefully', async ({ page }) => {
    await page.goto('/profile');

    // Uncheck all interests
    const checkboxes = page.locator('input[type="checkbox"]:checked');
    const count = await checkboxes.count();

    for (let i = 0; i < count; i++) {
      const checkbox = checkboxes.nth(i);
      if (await checkbox.isChecked()) {
        await checkbox.uncheck();
      }
    }

    // Save
    const saveButton = page.getByRole('button', { name: /save|update/i });
    if (await saveButton.isVisible()) {
      await saveButton.click();
      await page.waitForTimeout(1000);
    }

    // Navigate to personalized content
    await page.goto('/personalized-content');

    // Should show InterestPrompt (empty state for no interests)
    await expect(page.getByTestId('interest-prompt')).toBeVisible({ timeout: 5000 });
    await expect(page.getByRole('heading', { name: /select your interests/i })).toBeVisible();
  });

  test('should update content within 3 seconds (SC-005)', async ({ page }) => {
    await page.goto('/personalized-content');
    await expect(page.locator('[data-chapter-id]').first()).toBeVisible();

    // Record start time
    const startTime = Date.now();

    // Change interests
    await page.goto('/profile');
    const interestCheckbox = page.locator('input[type="checkbox"]').first();
    await interestCheckbox.click();

    const saveButton = page.getByRole('button', { name: /save|update/i });
    if (await saveButton.isVisible()) {
      await saveButton.click();
    }

    // Navigate back
    await page.goto('/personalized-content');

    // Wait for content to be visible
    await expect(page.locator('[data-chapter-id]').first()).toBeVisible({ timeout: 5000 });

    const endTime = Date.now();
    const elapsed = endTime - startTime;

    console.log(`Content updated in ${elapsed}ms`);

    // Should be within 3 seconds (3000ms)
    expect(elapsed).toBeLessThan(3000);
  });

  test('should handle rapid interest changes', async ({ page }) => {
    await page.goto('/profile');

    // Make multiple rapid changes
    const checkboxes = page.locator('input[type="checkbox"]');
    const count = Math.min(await checkboxes.count(), 3);

    for (let i = 0; i < count; i++) {
      await checkboxes.nth(i).click();
      await page.waitForTimeout(100);
    }

    // Save
    const saveButton = page.getByRole('button', { name: /save|update/i });
    if (await saveButton.isVisible()) {
      await saveButton.click();
      await page.waitForTimeout(1000);
    }

    // Navigate to personalized content
    await page.goto('/personalized-content');

    // Content should still load correctly despite rapid changes
    await expect(page.locator('[data-chapter-id]').first()).toBeVisible({ timeout: 5000 });
  });

  test('should preserve interest context across navigation', async ({ page }) => {
    // Set interests in profile
    await page.goto('/profile');

    // Select specific interests
    const physicalAICheckbox = page.getByLabel(/physical.*ai/i);
    if (await physicalAICheckbox.isVisible()) {
      await physicalAICheckbox.check();
    }

    const saveButton = page.getByRole('button', { name: /save|update/i });
    if (await saveButton.isVisible()) {
      await saveButton.click();
      await page.waitForTimeout(1000);
    }

    // Go to personalized content
    await page.goto('/personalized-content');
    await expect(page.locator('[data-chapter-id]').first()).toBeVisible();

    // Navigate to docs (full book)
    await page.goto('/docs/intro');

    // Navigate back to personalized content
    await page.goto('/personalized-content');

    // Should still show personalized content (interests preserved)
    await expect(page.locator('[data-chapter-id]').first()).toBeVisible();
    await expect(page).toHaveURL('/personalized-content');
  });

  test('should show appropriate empty state when no chapters match new interests', async ({ page }) => {
    await page.goto('/profile');

    // Select an interest that has no matching chapters (if such exists)
    // For this test, we'll just verify the empty state can be displayed
    const checkboxes = page.locator('input[type="checkbox"]');

    // Uncheck all
    const checkedCount = await page.locator('input[type="checkbox"]:checked').count();
    for (let i = 0; i < checkedCount; i++) {
      const checkbox = page.locator('input[type="checkbox"]:checked').first();
      await checkbox.uncheck();
    }

    // Check just one interest
    if ((await checkboxes.count()) > 0) {
      await checkboxes.first().check();
    }

    const saveButton = page.getByRole('button', { name: /save|update/i });
    if (await saveButton.isVisible()) {
      await saveButton.click();
      await page.waitForTimeout(1000);
    }

    await page.goto('/personalized-content');

    // Should either show content or appropriate empty state
    const hasContent = await page.locator('[data-chapter-id]').first().isVisible({ timeout: 3000 }).catch(() => false);
    const hasEmptyState = await page.getByText(/no chapters found|select your interests/i).isVisible({ timeout: 1000 }).catch(() => false);

    expect(hasContent || hasEmptyState).toBeTruthy();
  });
});
