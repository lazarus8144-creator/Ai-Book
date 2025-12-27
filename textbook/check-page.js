const { chromium } = require('playwright');

(async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage();

  const consoleMessages = [];
  const pageErrors = [];

  page.on('console', msg => {
    consoleMessages.push(`${msg.type()}: ${msg.text()}`);
  });

  page.on('pageerror', error => {
    pageErrors.push(error.message);
  });

  try {
    await page.goto('http://localhost:3000', { waitUntil: 'networkidle', timeout: 15000 });

    const bodyText = await page.textContent('body');
    const docusaurusRoot = await page.$('#__docusaurus');

    console.log('===== DIAGNOSTICS =====');
    console.log(`Body text length: ${bodyText.length}`);
    console.log(`#__docusaurus found: ${docusaurusRoot !== null}`);

    if (docusaurusRoot) {
      const innerHTML = await docusaurusRoot.innerHTML();
      console.log(`#__docusaurus content length: ${innerHTML.length}`);
      if (innerHTML.length < 100) {
        console.log(`#__docusaurus content: ${innerHTML}`);
      }
    }

    if (pageErrors.length > 0) {
      console.log('\n===== PAGE ERRORS =====');
      pageErrors.forEach(err => console.log(err));
    }

    if (consoleMessages.length > 0) {
      console.log('\n===== CONSOLE MESSAGES =====');
      consoleMessages.forEach(msg => console.log(msg));
    }

  } catch (error) {
    console.log(`PLAYWRIGHT ERROR: ${error.message}`);
  }

  await browser.close();
})();
