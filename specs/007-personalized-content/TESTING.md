# Cross-Browser and Mobile Testing Guide (T063-T064)

This document provides guidelines for testing the personalized content feature across different browsers and devices.

---

## Cross-Browser Testing (T063)

### Supported Browsers

The personalized content feature should work in:

**Desktop:**
- ✅ Chrome 90+ (Chromium-based)
- ✅ Firefox 88+
- ✅ Safari 14+
- ✅ Edge 90+ (Chromium-based)

**Mobile:**
- ✅ iOS Safari 14+
- ✅ Chrome Android 90+
- ✅ Samsung Internet 14+

### Browser-Specific Features

| Feature | Chrome | Firefox | Safari | Edge |
|---------|--------|---------|--------|------|
| Lazy Loading (`loading="lazy"`) | ✅ | ✅ | ✅ | ✅ |
| Intersection Observer | ✅ | ✅ | ✅ | ✅ |
| CSS `:focus-visible` | ✅ | ✅ | ✅ | ✅ |
| CSS `:has()` selector | ✅ | ⚠️ 103+ | ✅ 15.4+ | ✅ |
| React.memo | ✅ | ✅ | ✅ | ✅ |

⚠️ **Note**: `:has()` selector has fallback graceful degradation. Cards will still be interactive without it.

### Manual Testing Checklist

#### Chrome/Edge (Chromium)

```bash
# 1. Open in Chrome
http://localhost:3000/personalized-content

# 2. Test Features
□ Sign in flow works
□ Content filtering displays correctly
□ Interest changes trigger refresh
□ Keyboard navigation (Tab key)
□ Focus states visible
□ Lazy image loading (check Network tab)
□ Responsive design (use Device Toolbar - Ctrl+Shift+M)

# 3. DevTools Checks
□ No console errors
□ No failed network requests
□ Accessibility audit passes (Lighthouse)
```

**Lighthouse Audit:**
```bash
1. Open DevTools (F12)
2. Navigate to "Lighthouse" tab
3. Select "Accessibility" category
4. Click "Generate report"
5. Expected score: 90+ (target: 100)
```

#### Firefox

```bash
# 1. Open in Firefox
http://localhost:3000/personalized-content

# 2. Test Features
□ All features work identically to Chrome
□ Focus states visible
□ CSS Grid layout renders correctly
□ Flexbox layouts work

# 3. Specific Firefox Checks
□ Font rendering looks good
□ Border radius renders correctly
□ Transitions/animations smooth
```

#### Safari (macOS/iOS)

```bash
# 1. Open in Safari
http://localhost:3000/personalized-content

# 2. Test Features
□ All features work
□ Webkit-specific rendering issues
□ Touch interactions (iOS)
□ Smooth scrolling

# 3. Safari-Specific Checks
□ Date inputs work (if used)
□ Backdrop filter works (if used)
□ -webkit-prefixes applied where needed
```

### BrowserStack Testing (Optional)

If you have access to BrowserStack for comprehensive testing:

```bash
# 1. Sign up at https://www.browserstack.com
# 2. Open Live Testing
# 3. Select browsers/devices:
   - Chrome 120 (Windows 11)
   - Firefox 119 (Windows 11)
   - Safari 17 (macOS Sonoma)
   - Edge 120 (Windows 11)
   - iPhone 15 Pro (iOS 17)
   - Samsung Galaxy S23 (Android 13)
```

### Automated Cross-Browser Testing

```bash
# Using Playwright for cross-browser E2E tests
npm install --save-dev @playwright/test

# Run tests across all browsers
npx playwright test --project=chromium
npx playwright test --project=firefox
npx playwright test --project=webkit
```

**Playwright Config** (website/playwright.config.ts):
```typescript
import { defineConfig, devices } from '@playwright/test';

export default defineConfig({
  projects: [
    {
      name: 'chromium',
      use: { ...devices['Desktop Chrome'] },
    },
    {
      name: 'firefox',
      use: { ...devices['Desktop Firefox'] },
    },
    {
      name: 'webkit',
      use: { ...devices['Desktop Safari'] },
    },
  ],
});
```

---

## Mobile Responsive Testing (T064)

### Breakpoints

The personalized content feature uses the following breakpoints:

```css
/* Mobile First Approach */
/* Mobile: Default (320px - 767px) */
/* Tablet: 768px+ */
/* Desktop: 996px+ */
/* Large Desktop: 1280px+ */
```

### Device Testing Checklist

#### Mobile Devices (< 768px)

```bash
# Test on actual devices or Chrome DevTools

□ iPhone SE (375x667)
  □ Content cards stack vertically
  □ Navigation links readable
  □ Touch targets ≥ 44x44px
  □ Text readable without zoom

□ iPhone 12 Pro (390x844)
  □ All features work
  □ Safe area respected (notch)
  □ Landscape mode works

□ Samsung Galaxy S21 (360x800)
  □ Android Chrome works
  □ All interactions smooth
  □ No horizontal scroll
```

#### Tablets (768px - 995px)

```bash
□ iPad Mini (768x1024)
  □ Cards display in 2-column grid
  □ Navigation accessible
  □ Touch targets appropriate

□ iPad Pro (1024x1366)
  □ 2-3 column layout
  □ Comfortable reading size
```

#### Desktop (996px+)

```bash
□ Desktop (1920x1080)
  □ 3-column card grid
  □ Content centered with max-width
  □ No excessive whitespace
```

### Chrome DevTools Responsive Testing

```bash
# 1. Open DevTools (F12)
# 2. Toggle Device Toolbar (Ctrl+Shift+M)
# 3. Test these viewports:

# Mobile
- iPhone SE (375x667)
- iPhone 12 Pro (390x844)
- Pixel 5 (393x851)
- Samsung Galaxy S21 (360x800)

# Tablet
- iPad Mini (768x1024)
- iPad Air (820x1180)
- iPad Pro (1024x1366)

# Desktop
- 1280x720 (HD)
- 1920x1080 (Full HD)
- 2560x1440 (2K)
```

### Touch Interaction Testing

Mobile-specific features to test:

```bash
□ Touch Targets
  □ All links/buttons ≥ 44x44px
  □ Adequate spacing between touch elements
  □ No accidental taps

□ Gestures
  □ Tap to navigate works
  □ Scroll smooth
  □ No pinch-zoom issues on fixed elements

□ Viewport
  □ Meta viewport tag present: <meta name="viewport" content="width=device-width, initial-scale=1.0">
  □ No horizontal overflow
  □ Content fits screen width
```

### Mobile-Specific CSS Checks

```css
/* Verify these mobile optimizations exist */

/* Touch target sizing */
@media (max-width: 767px) {
  .cardLink {
    min-height: 44px;
    min-width: 44px;
  }
}

/* Responsive typography */
@media (max-width: 767px) {
  .title {
    font-size: 1.125rem; /* Slightly smaller on mobile */
  }
}

/* Stack layout on mobile */
@media (max-width: 767px) {
  .chaptersGrid {
    grid-template-columns: 1fr; /* Single column */
  }
}
```

### Responsive Images

Verify lazy-loaded images work on mobile:

```bash
# Test on slow 3G connection (DevTools Network tab)
1. Open DevTools → Network tab
2. Throttle to "Slow 3G"
3. Reload page
4. Verify:
   □ Images load as user scrolls
   □ Placeholder shows while loading
   □ No layout shift when images load
```

### Mobile Performance Testing

```bash
# Lighthouse Mobile Audit
1. Open DevTools (F12)
2. Lighthouse tab
3. Select "Mobile" device
4. Categories: Performance, Accessibility, Best Practices
5. Generate report

Expected Scores:
- Performance: 90+
- Accessibility: 95+
- Best Practices: 100
```

### Orientation Testing

```bash
□ Portrait Mode
  □ All content visible
  □ Navigation accessible
  □ Cards stack properly

□ Landscape Mode
  □ Layout adjusts appropriately
  □ No content cutoff
  □ Comfortable reading
```

### Network Conditions Testing

```bash
# Test on various network speeds (DevTools Network tab)

□ Fast 3G
  □ Content loads within 3 seconds
  □ Lazy loading works

□ Slow 3G
  □ Content loads within 5 seconds
  □ Progressive loading
  □ No timeouts

□ Offline
  □ Graceful error message
  □ Service worker caching (if implemented)
```

---

## Automated Responsive Testing

### Using Playwright

```typescript
// website/tests/e2e/responsive.spec.ts

import { test, expect } from '@playwright/test';

const viewports = [
  { name: 'Mobile', width: 375, height: 667 },
  { name: 'Tablet', width: 768, height: 1024 },
  { name: 'Desktop', width: 1920, height: 1080 },
];

viewports.forEach(({ name, width, height }) => {
  test(`should render correctly on ${name} (${width}x${height})`, async ({ page }) => {
    await page.setViewportSize({ width, height });
    await page.goto('/personalized-content');

    // Wait for content to load
    await page.waitForSelector('[data-chapter-id]');

    // Take screenshot for visual comparison
    await expect(page).toHaveScreenshot(`personalized-${name.toLowerCase()}.png`);
  });
});
```

### Running Responsive Tests

```bash
# Run responsive tests
npx playwright test responsive.spec.ts

# Generate visual comparison reports
npx playwright show-report
```

---

## Testing Checklist Summary

### Before Release

- [ ] **Cross-Browser Testing**
  - [ ] Chrome 120+ works perfectly
  - [ ] Firefox 119+ works perfectly
  - [ ] Safari 17+ works perfectly
  - [ ] Edge 120+ works perfectly
  - [ ] No console errors in any browser
  - [ ] Lighthouse score 90+ in all browsers

- [ ] **Mobile Responsive Testing**
  - [ ] iPhone SE (small mobile) works
  - [ ] iPhone 12 Pro (standard mobile) works
  - [ ] iPad Mini (tablet) works
  - [ ] All touch targets ≥ 44x44px
  - [ ] No horizontal scroll on any device
  - [ ] Comfortable reading on all screen sizes

- [ ] **Performance**
  - [ ] Loads < 3 seconds on Fast 3G
  - [ ] Loads < 5 seconds on Slow 3G
  - [ ] Lazy loading works correctly
  - [ ] No layout shift (CLS < 0.1)

- [ ] **Accessibility**
  - [ ] Keyboard navigation works on all devices
  - [ ] Screen reader announces correctly
  - [ ] Color contrast passes on all screens
  - [ ] Focus indicators visible

---

## Common Issues and Fixes

### Issue: Content doesn't load on Safari

**Fix**: Ensure fetch API uses credentials:
```javascript
fetch(url, { credentials: 'include' })
```

### Issue: CSS Grid not working on older browsers

**Fix**: Add fallback flexbox layout:
```css
.chaptersGrid {
  display: flex; /* Fallback */
  flex-wrap: wrap;
  display: grid; /* Modern browsers */
  grid-template-columns: repeat(auto-fill, minmax(300px, 1fr));
}
```

### Issue: Touch targets too small on mobile

**Fix**: Increase minimum touch target size:
```css
@media (max-width: 767px) {
  a, button {
    min-height: 44px;
    min-width: 44px;
    padding: 12px;
  }
}
```

### Issue: Horizontal scroll on mobile

**Fix**: Ensure max-width on containers:
```css
.container {
  max-width: 100%;
  overflow-x: hidden;
}
```

---

## Resources

- [Can I Use](https://caniuse.com/) - Browser compatibility checker
- [BrowserStack](https://www.browserstack.com/) - Cross-browser testing platform
- [Playwright](https://playwright.dev/) - Browser automation
- [Chrome DevTools Device Mode](https://developer.chrome.com/docs/devtools/device-mode/)
- [Lighthouse](https://developers.google.com/web/tools/lighthouse)
