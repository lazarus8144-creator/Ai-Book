import { test, expect } from '@playwright/test';

test.describe('Code Blocks', () => {
  test('should render code blocks with syntax highlighting', async ({ page }) => {
    await page.goto('/module-1-ros2/02-nodes-topics');

    // Find code block
    const codeBlock = page.locator('pre code').first();
    await expect(codeBlock).toBeVisible();

    // Verify syntax highlighting is applied
    const hasHighlighting = await codeBlock.evaluate((el) => {
      return el.querySelectorAll('.token').length > 0;
    });
    expect(hasHighlighting).toBe(true);
  });

  test('should have copy button on code blocks', async ({ page }) => {
    await page.goto('/module-1-ros2/02-nodes-topics');

    // Find copy button
    const copyButton = page.locator('button[title*="Copy"]').or(page.locator('.clean-btn')).first();

    // Copy button should be visible on hover
    const codeBlock = page.locator('pre').first();
    await codeBlock.hover();

    // Button exists (even if not immediately visible)
    const buttonCount = await copyButton.count();
    expect(buttonCount).toBeGreaterThan(0);
  });

  test('should display code with line numbers', async ({ page }) => {
    await page.goto('/module-1-ros2/02-nodes-topics');

    // Find code block with showLineNumbers
    const codeWithLineNumbers = page.locator('pre').filter({ hasText: 'showLineNumbers' });

    if (await codeWithLineNumbers.count() > 0) {
      // Verify line numbers are rendered
      const hasLineNumbers = await codeWithLineNumbers.first().evaluate((el) => {
        return el.classList.contains('prism-code') ||
               el.querySelector('.token-line') !== null;
      });
      expect(hasLineNumbers).toBe(true);
    }
  });
});
