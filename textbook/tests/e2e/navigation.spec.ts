import { test, expect } from '@playwright/test';

test.describe('Navigation', () => {
  test('should navigate to all modules from homepage', async ({ page }) => {
    await page.goto('/');

    // Check homepage loads
    await expect(page).toHaveTitle(/Physical AI & Humanoid Robotics/);

    // Navigate to Module 1
    await page.click('text=Module 1: ROS 2');
    await expect(page.locator('h1')).toContainText('Robotic Nervous System');
    await page.goBack();

    // Navigate to Module 2
    await page.click('text=Module 2: Digital Twin');
    await expect(page.locator('h1')).toContainText('Digital Twin');
    await page.goBack();

    // Navigate to Module 3
    await page.click('text=Module 3: NVIDIA Isaac');
    await expect(page.locator('h1')).toContainText('NVIDIA Isaac');
    await page.goBack();

    // Navigate to Module 4
    await page.click('text=Module 4: VLA');
    await expect(page.locator('h1')).toContainText('Vision-Language-Action');
  });

  test('should use sidebar navigation', async ({ page }) => {
    await page.goto('/');

    // Verify sidebar is visible
    const sidebar = page.locator('.theme-doc-sidebar-container');
    await expect(sidebar).toBeVisible();

    // Navigate through sidebar
    await page.click('text=Hardware Overview');
    await expect(page).toHaveURL(/\/hardware-overview/);
  });

  test('should navigate between chapters', async ({ page }) => {
    await page.goto('/module-1-ros2/01-introduction');

    // Check next/previous navigation
    const nextButton = page.locator('a:has-text("Next")').first();
    if (await nextButton.isVisible()) {
      await nextButton.click();
      await expect(page).toHaveURL(/\/module-1-ros2\/02-nodes-topics/);
    }
  });
});
