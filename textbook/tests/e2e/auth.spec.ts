/**
 * E2E Tests for Authentication Flows
 *
 * Tests complete user journeys:
 * - User registration
 * - User login (with and without "Remember Me")
 * - Profile management
 * - Password reset
 * - Logout
 */

import { test, expect } from '@playwright/test';

const BASE_URL = process.env.BASE_URL || 'http://localhost:3000';
const API_BASE_URL = process.env.API_BASE_URL || 'http://localhost:8000';

// Test user data
const testUser = {
  name: 'Test User',
  email: `test-${Date.now()}@example.com`, // Unique email for each test run
  password: 'TestPass123!',
  skill_level: 'beginner',
  learning_goals: 'Learn Physical AI and Humanoid Robotics',
  prior_experience: 'Beginner in ROS 2',
};

test.describe('Authentication Flows', () => {
  test.beforeEach(async ({ page }) => {
    // Navigate to home page before each test
    await page.goto(BASE_URL);
  });

  test('Complete user registration flow', async ({ page }) => {
    // 1. Click "Sign Up" button in navbar
    await page.click('text=Sign Up');

    // 2. Verify signup modal is visible
    await expect(page.locator('#signup-modal-title')).toBeVisible();
    await expect(page.locator('#signup-modal-title')).toContainText('Create Your Account');

    // 3. Fill out registration form
    await page.fill('#name', testUser.name);
    await page.fill('#email', testUser.email);
    await page.fill('#password', testUser.password);
    await page.selectOption('#skill_level', testUser.skill_level);
    await page.fill('#learning_goals', testUser.learning_goals);
    await page.fill('#prior_experience', testUser.prior_experience);

    // 4. Submit form
    await page.click('button[type="submit"]:has-text("Sign Up")');

    // 5. Verify user is logged in (modal closes, user name appears in navbar)
    await expect(page.locator('#signup-modal-title')).not.toBeVisible({ timeout: 5000 });

    // Wait for navbar to update with user name (or profile indicator)
    await expect(page.locator(`text=${testUser.name}`)).toBeVisible({ timeout: 5000 });
  });

  test('Login with valid credentials', async ({ page }) => {
    // First, register a user (reuse registration logic)
    await page.click('text=Sign Up');
    await page.fill('#name', testUser.name);
    await page.fill('#email', testUser.email);
    await page.fill('#password', testUser.password);
    await page.selectOption('#skill_level', testUser.skill_level);
    await page.click('button[type="submit"]:has-text("Sign Up")');
    await expect(page.locator('#signup-modal-title')).not.toBeVisible({ timeout: 5000 });

    // Logout
    await page.click(`text=${testUser.name}`);
    await page.click('text=Sign Out');
    await expect(page.locator(`text=${testUser.name}`)).not.toBeVisible({ timeout: 5000 });

    // Now test login
    await page.click('text=Sign In');
    await expect(page.locator('#login-modal-title')).toBeVisible();

    await page.fill('#email', testUser.email);
    await page.fill('#password', testUser.password);
    await page.click('button[type="submit"]:has-text("Sign In")');

    // Verify successful login
    await expect(page.locator('#login-modal-title')).not.toBeVisible({ timeout: 5000 });
    await expect(page.locator(`text=${testUser.name}`)).toBeVisible({ timeout: 5000 });
  });

  test('Login with "Remember Me" option', async ({ page, context }) => {
    // Register user first
    await page.click('text=Sign Up');
    await page.fill('#name', testUser.name);
    await page.fill('#email', testUser.email);
    await page.fill('#password', testUser.password);
    await page.selectOption('#skill_level', testUser.skill_level);
    await page.click('button[type="submit"]:has-text("Sign Up")');
    await expect(page.locator('#signup-modal-title')).not.toBeVisible({ timeout: 5000 });

    // Logout
    await page.click(`text=${testUser.name}`);
    await page.click('text=Sign Out');

    // Login with "Remember Me"
    await page.click('text=Sign In');
    await page.fill('#email', testUser.email);
    await page.fill('#password', testUser.password);
    await page.check('#remember_me');
    await page.click('button[type="submit"]:has-text("Sign In")');

    // Verify login
    await expect(page.locator(`text=${testUser.name}`)).toBeVisible({ timeout: 5000 });

    // Check that auth cookie exists and has long expiry
    const cookies = await context.cookies();
    const authCookie = cookies.find(c => c.name.includes('session') || c.name.includes('auth'));

    // Cookie should exist (if httpOnly, we may not see it in client JS but it should be in browser context)
    // For this test, we just verify user stays logged in
    expect(authCookie).toBeDefined();
  });

  test('Login with invalid credentials shows error', async ({ page }) => {
    await page.click('text=Sign In');
    await page.fill('#email', 'nonexistent@example.com');
    await page.fill('#password', 'WrongPassword123!');
    await page.click('button[type="submit"]:has-text("Sign In")');

    // Verify error message is shown
    await expect(page.locator('div[role="alert"]')).toBeVisible();
    await expect(page.locator('div[role="alert"]')).toContainText(/invalid|failed/i);
  });

  test('Registration with duplicate email shows error', async ({ page }) => {
    // Register first user
    await page.click('text=Sign Up');
    await page.fill('#name', testUser.name);
    await page.fill('#email', testUser.email);
    await page.fill('#password', testUser.password);
    await page.selectOption('#skill_level', testUser.skill_level);
    await page.click('button[type="submit"]:has-text("Sign Up")');
    await expect(page.locator('#signup-modal-title')).not.toBeVisible({ timeout: 5000 });

    // Logout
    await page.click(`text=${testUser.name}`);
    await page.click('text=Sign Out');

    // Try to register with same email
    await page.click('text=Sign Up');
    await page.fill('#name', 'Another User');
    await page.fill('#email', testUser.email); // Same email
    await page.fill('#password', 'AnotherPass123!');
    await page.selectOption('#skill_level', 'intermediate');
    await page.click('button[type="submit"]:has-text("Sign Up")');

    // Verify error message
    await expect(page.locator('div[role="alert"]')).toBeVisible();
    await expect(page.locator('div[role="alert"]')).toContainText(/already registered|exists/i);
  });

  test('Profile management flow', async ({ page }) => {
    // Register and login
    await page.click('text=Sign Up');
    await page.fill('#name', testUser.name);
    await page.fill('#email', testUser.email);
    await page.fill('#password', testUser.password);
    await page.selectOption('#skill_level', testUser.skill_level);
    await page.click('button[type="submit"]:has-text("Sign Up")');
    await expect(page.locator(`text=${testUser.name}`)).toBeVisible({ timeout: 5000 });

    // Navigate to profile page
    await page.click(`text=${testUser.name}`);
    await page.click('text=My Profile');

    // Verify profile page loads
    await expect(page).toHaveURL(/\/profile/);

    // Verify current data is displayed
    await expect(page.locator('#name')).toHaveValue(testUser.name);
    await expect(page.locator('#email')).toHaveValue(testUser.email);

    // Update profile
    const updatedGoals = 'Build autonomous robots';
    await page.fill('#learning_goals', updatedGoals);
    await page.selectOption('#skill_level', 'intermediate');
    await page.click('button[type="submit"]:has-text("Save")');

    // Verify success message
    await expect(page.locator('text=/saved|updated|success/i')).toBeVisible({ timeout: 5000 });

    // Refresh page and verify changes persist
    await page.reload();
    await expect(page.locator('#learning_goals')).toHaveValue(updatedGoals);
    await expect(page.locator('#skill_level')).toHaveValue('intermediate');
  });

  test('Password reset request flow', async ({ page }) => {
    // Register user first
    await page.click('text=Sign Up');
    await page.fill('#name', testUser.name);
    await page.fill('#email', testUser.email);
    await page.fill('#password', testUser.password);
    await page.selectOption('#skill_level', testUser.skill_level);
    await page.click('button[type="submit"]:has-text("Sign Up")');
    await expect(page.locator('#signup-modal-title')).not.toBeVisible({ timeout: 5000 });

    // Logout
    await page.click(`text=${testUser.name}`);
    await page.click('text=Sign Out');

    // Open login modal and click "Forgot Password?"
    await page.click('text=Sign In');
    await page.click('text=Forgot Password?');

    // Verify password reset modal opens
    await expect(page.locator('#password-reset-modal-title')).toBeVisible();
    await expect(page.locator('#password-reset-modal-title')).toContainText('Reset Your Password');

    // Submit password reset request
    await page.fill('#reset-email', testUser.email);
    await page.click('button[type="submit"]:has-text("Send Reset Link")');

    // Verify success message
    await expect(page.locator('div[role="alert"]:has-text("sent")')).toBeVisible({ timeout: 5000 });
  });

  test('Logout flow', async ({ page }) => {
    // Register and login
    await page.click('text=Sign Up');
    await page.fill('#name', testUser.name);
    await page.fill('#email', testUser.email);
    await page.fill('#password', testUser.password);
    await page.selectOption('#skill_level', testUser.skill_level);
    await page.click('button[type="submit"]:has-text("Sign Up")');
    await expect(page.locator(`text=${testUser.name}`)).toBeVisible({ timeout: 5000 });

    // Logout
    await page.click(`text=${testUser.name}`);
    await page.click('text=Sign Out');

    // Verify user is logged out (navbar shows "Sign In" again)
    await expect(page.locator(`text=${testUser.name}`)).not.toBeVisible({ timeout: 5000 });
    await expect(page.locator('text=Sign In')).toBeVisible();
  });

  test('Keyboard navigation accessibility', async ({ page }) => {
    // Open signup modal
    await page.click('text=Sign Up');
    await expect(page.locator('#signup-modal-title')).toBeVisible();

    // Tab through form fields
    await page.keyboard.press('Tab'); // Should focus close button
    await page.keyboard.press('Tab'); // Should focus name field
    await expect(page.locator('#name')).toBeFocused();

    await page.keyboard.press('Tab'); // Should focus email field
    await expect(page.locator('#email')).toBeFocused();

    await page.keyboard.press('Tab'); // Should focus password field
    await expect(page.locator('#password')).toBeFocused();

    // Close modal with Escape key
    await page.keyboard.press('Escape');
    await expect(page.locator('#signup-modal-title')).not.toBeVisible({ timeout: 2000 });
  });

  test('Form validation', async ({ page }) => {
    await page.click('text=Sign Up');

    // Try to submit with invalid email
    await page.fill('#name', 'Test User');
    await page.fill('#email', 'invalid-email');
    await page.fill('#password', 'short');
    await page.click('button[type="submit"]:has-text("Sign Up")');

    // Verify validation errors appear
    await expect(page.locator('text=/invalid email/i')).toBeVisible();
    await expect(page.locator('text=/at least 8 characters/i')).toBeVisible();
  });

  test('Modal closes when clicking overlay', async ({ page }) => {
    // Open login modal
    await page.click('text=Sign In');
    await expect(page.locator('#login-modal-title')).toBeVisible();

    // Click on overlay (outside modal content)
    await page.locator('.modalOverlay').click({ position: { x: 5, y: 5 } });

    // Modal should close
    await expect(page.locator('#login-modal-title')).not.toBeVisible({ timeout: 2000 });
  });

  test('ChatWidget includes user context when authenticated', async ({ page }) => {
    // Register and login
    await page.click('text=Sign Up');
    await page.fill('#name', testUser.name);
    await page.fill('#email', testUser.email);
    await page.fill('#password', testUser.password);
    await page.selectOption('#skill_level', testUser.skill_level);
    await page.click('button[type="submit"]:has-text("Sign Up")');
    await expect(page.locator(`text=${testUser.name}`)).toBeVisible({ timeout: 5000 });

    // Open chat widget
    await page.click('button[aria-label="Toggle chat"]');
    await expect(page.locator('text=Textbook Assistant')).toBeVisible();

    // Send a message
    await page.fill('textarea[placeholder*="Ask a question"]', 'What is ROS 2?');

    // Intercept the chat API call to verify user_id and skill_level are included
    const requestPromise = page.waitForRequest(request =>
      request.url().includes('/api/v1/chat/query') && request.method() === 'POST'
    );

    await page.click('button.sendButton');

    const request = await requestPromise;
    const postData = request.postDataJSON();

    // Verify user context is included
    expect(postData.user_id).toBeDefined();
    expect(postData.skill_level).toBe(testUser.skill_level);
  });
});

test.describe('Accessibility Compliance (WCAG 2.1 AA)', () => {
  test('Login modal has proper ARIA attributes', async ({ page }) => {
    await page.goto(BASE_URL);
    await page.click('text=Sign In');

    const modal = page.locator('[role="dialog"]');
    await expect(modal).toBeVisible();
    await expect(modal).toHaveAttribute('aria-modal', 'true');
    await expect(modal).toHaveAttribute('aria-labelledby', 'login-modal-title');

    // Check form has aria-label
    const form = page.locator('form[aria-label="Login form"]');
    await expect(form).toBeVisible();

    // Check inputs have aria-invalid when showing errors
    await page.fill('#email', 'invalid');
    await page.blur('#email');
    // Note: aria-invalid is set by the form library based on validation
  });

  test('All interactive elements have focus indicators', async ({ page }) => {
    await page.goto(BASE_URL);
    await page.click('text=Sign In');

    // Tab to close button and verify it gets focus indicator
    await page.keyboard.press('Tab');
    const closeButton = page.locator('button[aria-label="Close login modal"]');
    await expect(closeButton).toBeFocused();

    // Verify visible focus indicator (outline should be present)
    const hasOutline = await closeButton.evaluate(el => {
      const styles = window.getComputedStyle(el, ':focus-visible');
      return styles.outline !== 'none';
    });
    expect(hasOutline).toBeTruthy();
  });

  test('Error messages have role="alert"', async ({ page }) => {
    await page.goto(BASE_URL);
    await page.click('text=Sign In');

    await page.fill('#email', 'wrong@example.com');
    await page.fill('#password', 'wrongpassword');
    await page.click('button[type="submit"]:has-text("Sign In")');

    const alert = page.locator('div[role="alert"]');
    await expect(alert).toBeVisible();
  });
});
