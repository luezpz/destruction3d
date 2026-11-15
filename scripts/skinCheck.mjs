// GPU vs CPU skin agreement (PLAN M4): starts vite, opens skincheck.html in system Chrome (WebGPU and ?webgl), checks tolerances.
// Usage: npm run test:gpu   (env CHROME = path to chrome.exe overrides the default)
import { chromium } from 'playwright-core';
import { createServer } from 'vite';

const CHROME = process.env.CHROME ?? 'C:/Program Files/Google/Chrome/Application/chrome.exe';
const TOL_POS = 1e-4, TOL_NORMAL = 1e-4; // fp32 over |p| <= ~6: a few ulp per entry, x32 entries

const server = await createServer({ server: { port: 5180, strictPort: false }, logLevel: 'error' });
await server.listen();
const url = server.resolvedUrls.local[0];
const browser = await chromium.launch({ executablePath: CHROME, headless: false, args: ['--enable-unsafe-webgpu', '--window-position=-2400,0', '--window-size=400,300'] });
let ok = true;
try {
  for (const q of ['', '?webgl']) {
    const page = await browser.newPage();
    page.on('pageerror', (e) => console.log('pageerror:', e.message));
    await page.goto(`${url}skincheck.html${q}`);
    const r = await page.evaluate(() => window.skinCheck);
    console.log(JSON.stringify(r));
    if (r.error) { ok = false; continue; }
    if (!q && !r.isWebGPU) { console.log('WebGPU unavailable: the WebGPU run fell back to WebGL2'); ok = false; }
    for (const [c, d] of Object.entries(r.result)) if (!(d.pos <= TOL_POS && d.normal <= TOL_NORMAL)) { console.log(`FAIL ${r.backend} count=${c}`); ok = false; }
    await page.close();
  }
} finally {
  await browser.close();
  await server.close();
}
console.log(ok ? 'skin check OK' : 'skin check FAILED');
process.exit(ok ? 0 : 1);
