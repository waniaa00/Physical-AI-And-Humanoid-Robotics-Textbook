# Production Deployment Guide

This guide covers deploying the Humanoid Robotics learning platform with the personalized content feature to production.

---

## Pre-Deployment Checklist

### ✅ Code Quality
- [ ] All 104+ tests passing
- [ ] Zero TypeScript errors
- [ ] No console errors in browser
- [ ] Code review completed

### ✅ Security
- [ ] Environment variables configured (see `.env.example`)
- [ ] Session secrets generated (strong random strings)
- [ ] HTTPS enforced (`FORCE_HTTPS=true`)
- [ ] Security headers configured (CSP, HSTS, etc.)
- [ ] CORS origins whitelisted
- [ ] Rate limiting enabled

### ✅ Performance
- [ ] Production build optimized
- [ ] Images optimized
- [ ] Lazy loading enabled
- [ ] Bundle size acceptable (<500KB initial)

### ✅ Accessibility
- [ ] WCAG 2.1 AA compliance verified
- [ ] Screen reader tested
- [ ] Keyboard navigation verified

---

## Environment Configuration

### 1. Create Production Environment File

```bash
# Copy example and edit
cp .env.example .env

# Generate session secret
node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"

# Edit .env with production values
```

### 2. Required Environment Variables

```bash
# Backend API
REACT_APP_BACKEND_URL=https://api.yourdomain.com

# Security
FORCE_HTTPS=true
SESSION_SECRET=<generated-secret-from-step-1>
COOKIE_SECURE=true
COOKIE_SAMESITE=strict

# Features
ENABLE_PERSONALIZED_CONTENT=true

# Environment
NODE_ENV=production
```

---

## Deployment Platforms

### Option 1: Vercel (Recommended)

**Automatic deployment with Git:**

1. **Install Vercel CLI**:
   ```bash
   npm install -g vercel
   ```

2. **Login to Vercel**:
   ```bash
   vercel login
   ```

3. **Deploy**:
   ```bash
   cd website
   vercel --prod
   ```

4. **Configure Environment Variables**:
   ```bash
   # Via Vercel Dashboard:
   # 1. Go to project settings
   # 2. Navigate to "Environment Variables"
   # 3. Add all variables from .env
   ```

5. **Security Headers**:
   - ✅ Already configured in `vercel.json`
   - Headers will be applied automatically

**Files**:
- `website/vercel.json` - Configuration with security headers
- `.env.example` - Environment variables template

---

### Option 2: Netlify

**Automatic deployment with Git:**

1. **Install Netlify CLI**:
   ```bash
   npm install -g netlify-cli
   ```

2. **Login to Netlify**:
   ```bash
   netlify login
   ```

3. **Initialize site**:
   ```bash
   cd website
   netlify init
   ```

4. **Deploy**:
   ```bash
   netlify deploy --prod
   ```

5. **Configure Environment Variables**:
   ```bash
   # Via Netlify Dashboard:
   # 1. Go to Site settings
   # 2. Navigate to "Build & deploy" > "Environment"
   # 3. Add all variables from .env
   ```

6. **Security Headers**:
   - ✅ Already configured in `_headers` file
   - Copy `website/_headers` to build output

**Build Settings**:
```toml
# netlify.toml
[build]
  command = "npm run build"
  publish = "build"

[[headers]]
  for = "/*"
  [headers.values]
    # Headers from _headers file will be applied
```

---

### Option 3: Traditional Hosting (Apache/Nginx)

#### Apache (.htaccess)

```apache
# Force HTTPS
RewriteEngine On
RewriteCond %{HTTPS} off
RewriteRule ^(.*)$ https://%{HTTP_HOST}%{REQUEST_URI} [L,R=301]

# Security Headers
Header always set Content-Security-Policy "default-src 'self'; script-src 'self' 'unsafe-inline' 'unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: https:; font-src 'self' data:; connect-src 'self'; frame-ancestors 'none'"
Header always set Strict-Transport-Security "max-age=31536000; includeSubDomains; preload"
Header always set X-Content-Type-Options "nosniff"
Header always set X-Frame-Options "DENY"
Header always set X-XSS-Protection "1; mode=block"
Header always set Referrer-Policy "strict-origin-when-cross-origin"
Header always set Permissions-Policy "camera=(), microphone=(), geolocation=()"

# Cache Control
<filesMatch "\.(html)$">
  Header set Cache-Control "public, max-age=0, must-revalidate"
</filesMatch>

<filesMatch "\.(css|js|jpg|jpeg|png|gif|svg|woff|woff2|ttf|eot)$">
  Header set Cache-Control "public, max-age=31536000, immutable"
</filesMatch>
```

#### Nginx

```nginx
server {
    listen 443 ssl http2;
    server_name yourdomain.com;

    # SSL Configuration
    ssl_certificate /path/to/cert.pem;
    ssl_certificate_key /path/to/key.pem;

    # Security Headers
    add_header Content-Security-Policy "default-src 'self'; script-src 'self' 'unsafe-inline' 'unsafe-eval'; style-src 'self' 'unsafe-inline'; img-src 'self' data: https:; font-src 'self' data:; connect-src 'self'; frame-ancestors 'none'" always;
    add_header Strict-Transport-Security "max-age=31536000; includeSubDomains; preload" always;
    add_header X-Content-Type-Options "nosniff" always;
    add_header X-Frame-Options "DENY" always;
    add_header X-XSS-Protection "1; mode=block" always;
    add_header Referrer-Policy "strict-origin-when-cross-origin" always;
    add_header Permissions-Policy "camera=(), microphone=(), geolocation=()" always;

    # Serve static files
    root /var/www/html/build;
    index index.html;

    # SPA routing
    location / {
        try_files $uri $uri/ /index.html;
    }

    # Cache static assets
    location ~* \.(css|js|jpg|jpeg|png|gif|svg|woff|woff2|ttf|eot)$ {
        expires 1y;
        add_header Cache-Control "public, immutable";
    }

    # No cache for HTML
    location ~* \.html$ {
        expires -1;
        add_header Cache-Control "public, max-age=0, must-revalidate";
    }
}

# Redirect HTTP to HTTPS
server {
    listen 80;
    server_name yourdomain.com;
    return 301 https://$server_name$request_uri;
}
```

---

## Build Process

### 1. Production Build

```bash
cd website
npm run build
```

**Output**: `website/build/` directory

### 2. Verify Build

```bash
# Test production build locally
npm install -g serve
serve -s build -p 3000

# Open in browser
open http://localhost:3000
```

### 3. Build Optimization

Check bundle size:
```bash
npm install -g source-map-explorer

# Analyze bundle
npm run build
source-map-explorer 'build/static/js/*.js'
```

**Target**: Initial bundle < 500KB gzipped

---

## Backend Deployment

### 1. Deploy Backend API

The backend must be deployed separately and configured with:

- **HTTPS enabled**
- **CORS configured** for frontend domain
- **Session store** (Redis recommended for production)
- **Database** (PostgreSQL recommended)

### 2. Backend Environment Variables

```bash
# Backend .env
DATABASE_URL=postgresql://user:pass@host:5432/db
REDIS_URL=redis://host:6379
SESSION_SECRET=<strong-secret>
ALLOWED_ORIGINS=https://yourdomain.com
PORT=3001
NODE_ENV=production
```

### 3. Update Frontend API URL

```bash
# Frontend .env
REACT_APP_BACKEND_URL=https://api.yourdomain.com
```

---

## Post-Deployment Verification

### 1. Smoke Tests

Run through these scenarios in production:

- [ ] Homepage loads
- [ ] Sign up/sign in works
- [ ] Profile page loads and allows interest selection
- [ ] Personalized content page displays filtered content
- [ ] Interest changes trigger content refresh
- [ ] Navigation between pages works
- [ ] Mobile responsive design works
- [ ] HTTPS is enforced
- [ ] No console errors

### 2. Security Headers Check

Test with: https://securityheaders.com/

Expected grade: **A** or **A+**

Required headers:
- ✅ Content-Security-Policy
- ✅ Strict-Transport-Security
- ✅ X-Content-Type-Options
- ✅ X-Frame-Options
- ✅ Referrer-Policy

### 3. Performance Check

Test with Lighthouse (Chrome DevTools):

Expected scores:
- Performance: 90+
- Accessibility: 95+
- Best Practices: 100
- SEO: 90+

### 4. SSL Certificate Check

Test with: https://www.ssllabs.com/ssltest/

Expected grade: **A** or **A+**

---

## Monitoring & Maintenance

### 1. Error Tracking

**Option A: Sentry**

```bash
npm install @sentry/react @sentry/tracing

# Configure in src/index.tsx
import * as Sentry from "@sentry/react";

Sentry.init({
  dsn: process.env.REACT_APP_SENTRY_DSN,
  environment: process.env.NODE_ENV,
  tracesSampleRate: 1.0,
});
```

**Option B: LogRocket**

```bash
npm install logrocket
```

### 2. Analytics

**Option A: Google Analytics**

```typescript
// src/utils/analytics.ts
export const GA_TRACKING_ID = process.env.REACT_APP_GA_ID;

export const pageview = (url: string) => {
  (window as any).gtag('config', GA_TRACKING_ID, {
    page_path: url,
  });
};
```

### 3. Uptime Monitoring

Use services like:
- UptimeRobot (free)
- Pingdom
- StatusCake

### 4. Performance Monitoring

Track key metrics:
- Page load time
- API response time
- Error rate
- User engagement

---

## Rollback Plan

### Quick Rollback (Vercel/Netlify)

```bash
# Vercel
vercel rollback

# Netlify
netlify rollback
```

### Manual Rollback

1. Revert to previous Git commit
2. Rebuild and redeploy
3. Verify in production

---

## Troubleshooting

### Issue: CSP Blocking Resources

**Symptom**: Console errors about CSP violations

**Fix**: Update CSP in `vercel.json` or `_headers`:
```
Content-Security-Policy: ... img-src 'self' data: https://your-cdn.com; ...
```

### Issue: CORS Errors

**Symptom**: API requests failing with CORS error

**Fix**: Update backend CORS configuration:
```javascript
app.use(cors({
  origin: 'https://yourdomain.com',
  credentials: true,
}));
```

### Issue: Session Not Persisting

**Symptom**: Users logged out after page refresh

**Fix**: Verify cookie settings:
```bash
COOKIE_SECURE=true  # Must be true with HTTPS
COOKIE_SAMESITE=strict
```

### Issue: Slow Page Load

**Symptom**: Lighthouse performance score < 90

**Fix**:
1. Enable code splitting
2. Optimize images
3. Enable CDN
4. Check bundle size

---

## Security Checklist

Before going live:

- [ ] HTTPS enforced (HSTS enabled)
- [ ] Security headers configured (CSP, X-Frame-Options, etc.)
- [ ] Session secrets are strong and unique
- [ ] CORS properly configured
- [ ] Rate limiting enabled
- [ ] Input validation on all forms
- [ ] SQL injection prevention (parameterized queries)
- [ ] XSS prevention (React auto-escaping + CSP)
- [ ] CSRF protection (SameSite cookies)
- [ ] Dependencies updated (npm audit)
- [ ] Secrets not in code (use environment variables)

---

## Support & Resources

### Documentation
- `specs/007-personalized-content/` - Feature documentation
- `SECURITY.md` - Security review
- `ACCESSIBILITY.md` - Accessibility testing
- `TESTING.md` - Testing guide

### Commands Reference

```bash
# Development
npm start                  # Start dev server
npm test                   # Run tests
npm run build             # Production build

# Deployment
vercel --prod             # Deploy to Vercel
netlify deploy --prod     # Deploy to Netlify

# Verification
npm audit                 # Check dependencies
npm run type-check        # TypeScript check
npx lighthouse https://yourdomain.com --view
```

---

## Success Criteria

Production deployment is successful when:

- ✅ All smoke tests pass
- ✅ Security headers grade: A+
- ✅ Lighthouse scores: 90+ across all categories
- ✅ SSL Labs grade: A+
- ✅ No critical errors in logs (24 hours)
- ✅ User feedback positive
- ✅ Performance metrics meet targets

**Contact**: For issues or questions, contact the development team.
