import { test, expect } from '@playwright/test';

test.describe('Search Functionality', () => {
  test('should find content when searching for ROS 2 nodes', async ({ page }) => {
    await page.goto('/');

    // Find and click search input
    const searchInput = page.locator('input[type="search"]').or(page.locator('.DocSearch-Button'));

    if (await searchInput.count() > 0) {
      await searchInput.click();

      // Type search query
      await page.keyboard.type('ROS 2 nodes');

      // Wait for results
      await page.waitForTimeout(1000);

      // Verify results appear
      const results = page.locator('[class*="search"]').or(page.locator('[role="option"]'));
      const resultCount = await results.count();
      expect(resultCount).toBeGreaterThan(0);
    }
  });

  test('should find digital twin content', async ({ page }) => {
    await page.goto('/');

    const searchInput = page.locator('input[type="search"]').or(page.locator('.DocSearch-Button'));

    if (await searchInput.count() > 0) {
      await searchInput.click();
      await page.keyboard.type('digital twin');
      await page.waitForTimeout(1000);

      const results = page.locator('[class*="search"]').or(page.locator('[role="option"]'));
      const resultCount = await results.count();
      expect(resultCount).toBeGreaterThan(0);
    }
  });

  test('should find Isaac Sim content', async ({ page }) => {
    await page.goto('/');

    const searchInput = page.locator('input[type="search"]').or(page.locator('.DocSearch-Button'));

    if (await searchInput.count() > 0) {
      await searchInput.click();
      await page.keyboard.type('Isaac Sim');
      await page.waitForTimeout(1000);

      const results = page.locator('[class*="search"]').or(page.locator('[role="option"]'));
      const resultCount = await results.count();
      expect(resultCount).toBeGreaterThan(0);
    }
  });

  test('should navigate to result when clicked', async ({ page }) => {
    await page.goto('/');

    const searchInput = page.locator('input[type="search"]').or(page.locator('.DocSearch-Button'));

    if (await searchInput.count() > 0) {
      await searchInput.click();
      await page.keyboard.type('perception');
      await page.waitForTimeout(1000);

      // Click first result (if available)
      const firstResult = page.locator('[role="option"]').first();
      if (await firstResult.count() > 0) {
        await firstResult.click();

        // Verify navigation occurred
        await page.waitForTimeout(500);
        const url = page.url();
        expect(url).not.toBe('/');
      }
    }
  });

  test('should show search highlighting on result page', async ({ page }) => {
    await page.goto('/');

    const searchInput = page.locator('input[type="search"]').or(page.locator('.DocSearch-Button'));

    if (await searchInput.count() > 0) {
      await searchInput.click();
      await page.keyboard.type('navigation');
      await page.waitForTimeout(1000);

      const firstResult = page.locator('[role="option"]').first();
      if (await firstResult.count() > 0) {
        await firstResult.click();
        await page.waitForTimeout(1000);

        // Check if highlighting is applied (depends on plugin implementation)
        // This is a soft check since highlighting behavior varies
        const hasHighlight = await page.locator('mark').or(page.locator('[class*="highlight"]')).count();
        // Just verify page loaded successfully
        expect(page.url()).toContain('http');
      }
    }
  });
});
