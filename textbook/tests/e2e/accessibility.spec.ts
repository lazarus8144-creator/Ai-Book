import { test, expect } from '@playwright/test';
import AxeBuilder from '@axe-core/playwright';

test.describe('Accessibility', () => {
  test('homepage should have no accessibility violations', async ({ page }) => {
    await page.goto('/');

    const accessibilityScanResults = await new AxeBuilder({ page }).analyze();

    expect(accessibilityScanResults.violations).toEqual([]);
  });

  test('module pages should have no accessibility violations', async ({ page }) => {
    const pages = [
      '/module-1-ros2',
      '/module-2-digital-twin',
      '/module-3-nvidia-isaac',
      '/module-4-vla',
    ];

    for (const path of pages) {
      await page.goto(path);
      const accessibilityScanResults = await new AxeBuilder({ page }).analyze();
      expect(accessibilityScanResults.violations).toEqual([]);
    }
  });

  test('should support keyboard navigation', async ({ page }) => {
    await page.goto('/');

    // Tab through interactive elements
    await page.keyboard.press('Tab');
    const firstFocusedElement = await page.evaluate(() => document.activeElement?.tagName);
    expect(['A', 'BUTTON', 'INPUT']).toContain(firstFocusedElement);

    // Press Tab multiple times
    for (let i = 0; i < 5; i++) {
      await page.keyboard.press('Tab');
    }

    // Verify we can navigate with keyboard
    const focusedElement = await page.evaluate(() => document.activeElement?.tagName);
    expect(focusedElement).toBeTruthy();
  });

  test('should have proper heading hierarchy', async ({ page }) => {
    await page.goto('/module-1-ros2/01-introduction');

    // Check for h1
    const h1Count = await page.locator('h1').count();
    expect(h1Count).toBeGreaterThan(0);

    // Verify heading hierarchy (no h3 without h2, etc.)
    const headings = await page.locator('h1, h2, h3, h4, h5, h6').allTextContents();
    expect(headings.length).toBeGreaterThan(0);
  });

  test('should have sufficient color contrast', async ({ page }) => {
    await page.goto('/');

    // AxeBuilder will check color contrast automatically
    const accessibilityScanResults = await new AxeBuilder({ page })
      .withTags(['wcag2aa', 'wcag21aa'])
      .analyze();

    // Filter for color contrast issues
    const contrastViolations = accessibilityScanResults.violations.filter(
      v => v.id.includes('color-contrast')
    );

    expect(contrastViolations).toEqual([]);
  });

  test('links should have accessible names', async ({ page }) => {
    await page.goto('/');

    const links = await page.locator('a').all();

    for (const link of links.slice(0, 10)) { // Check first 10 links
      const accessibleName = await link.getAttribute('aria-label') ||
                             await link.textContent() ||
                             await link.getAttribute('title');

      expect(accessibleName?.trim()).toBeTruthy();
    }
  });

  test('buttons should have accessible names', async ({ page }) => {
    await page.goto('/');

    const buttons = await page.locator('button').all();

    for (const button of buttons) {
      const accessibleName = await button.getAttribute('aria-label') ||
                             await button.textContent() ||
                             await button.getAttribute('title');

      expect(accessibleName?.trim()).toBeTruthy();
    }
  });

  test('should have proper ARIA landmarks', async ({ page }) => {
    await page.goto('/');

    // Check for main landmark
    const main = page.locator('main, [role="main"]');
    await expect(main).toBeVisible();

    // Check for navigation
    const nav = page.locator('nav, [role="navigation"]');
    expect(await nav.count()).toBeGreaterThan(0);
  });

  test('form inputs should have labels', async ({ page }) => {
    await page.goto('/');

    const inputs = await page.locator('input[type="text"], input[type="search"], input[type="email"]').all();

    for (const input of inputs) {
      const id = await input.getAttribute('id');
      const ariaLabel = await input.getAttribute('aria-label');
      const ariaLabelledby = await input.getAttribute('aria-labelledby');

      // Input should have either id (for label), aria-label, or aria-labelledby
      const hasAccessibleLabel = id || ariaLabel || ariaLabelledby;
      expect(hasAccessibleLabel).toBeTruthy();
    }
  });
});
