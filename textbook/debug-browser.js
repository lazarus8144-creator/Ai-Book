const { chromium } = require('playwright');

(async () => {
  const browser = await chromium.launch({ headless: false });
  const context = await browser.newContext();
  const page = await context.newPage();

  const errors = [];
  const warnings = [];
  const logs = [];

  // Capture console messages
  page.on('console', msg => {
    const text = msg.text();
    const type = msg.type();

    if (type === 'error') {
      errors.push(text);
      console.log(`🔴 ERROR: ${text}`);
    } else if (type === 'warning') {
      warnings.push(text);
      console.log(`🟡 WARNING: ${text}`);
    } else if (type === 'log') {
      logs.push(text);
    }
  });

  // Capture JavaScript errors
  page.on('pageerror', error => {
    errors.push(error.message);
    console.log(`🔴 PAGE ERROR: ${error.message}`);
    console.log(`Stack: ${error.stack}`);
  });

  // Capture failed requests
  page.on('requestfailed', request => {
    console.log(`🔴 FAILED REQUEST: ${request.url()} - ${request.failure().errorText}`);
  });

  try {
    console.log('🌐 Navigating to http://localhost:3000...\n');
    await page.goto('http://localhost:3000', {
      waitUntil: 'networkidle',
      timeout: 30000
    });

    // Wait a bit for React to render
    await page.waitForTimeout(5000);

    // Check what's actually on the page
    const bodyHTML = await page.evaluate(() => document.body.innerHTML);
    const docusaurusDiv = await page.$('#__docusaurus');
    const docusaurusContent = docusaurusDiv ? await docusaurusDiv.innerHTML() : 'NOT FOUND';

    console.log('\n📊 === PAGE ANALYSIS ===');
    console.log(`Body HTML length: ${bodyHTML.length} characters`);
    console.log(`#__docusaurus found: ${docusaurusDiv !== null}`);
    console.log(`#__docusaurus content length: ${docusaurusContent.length} characters`);

    if (docusaurusContent.length < 500) {
      console.log(`#__docusaurus content: ${docusaurusContent.substring(0, 500)}`);
    }

    // Check for visible text
    const visibleText = await page.evaluate(() => document.body.innerText);
    console.log(`\nVisible text length: ${visibleText.length} characters`);
    if (visibleText.length < 200) {
      console.log(`Visible text: "${visibleText}"`);
    }

    // Take a screenshot
    await page.screenshot({ path: '/tmp/blank-screen-debug.png', fullPage: true });
    console.log('\n📸 Screenshot saved to /tmp/blank-screen-debug.png');

    console.log('\n🔴 === ERRORS ===');
    if (errors.length === 0) {
      console.log('No errors found');
    } else {
      errors.forEach((err, i) => console.log(`${i + 1}. ${err}`));
    }

    console.log('\n🟡 === WARNINGS ===');
    if (warnings.length === 0) {
      console.log('No warnings');
    } else {
      warnings.forEach((warn, i) => console.log(`${i + 1}. ${warn}`));
    }

    // Check for specific elements
    const navbar = await page.$('nav');
    const main = await page.$('main');
    const footer = await page.$('footer');

    console.log('\n🔍 === ELEMENT CHECK ===');
    console.log(`Navbar found: ${navbar !== null}`);
    console.log(`Main content found: ${main !== null}`);
    console.log(`Footer found: ${footer !== null}`);

  } catch (error) {
    console.log(`\n💥 FATAL ERROR: ${error.message}`);
    console.log(error.stack);
  }

  await browser.close();
  process.exit(0);
})();
