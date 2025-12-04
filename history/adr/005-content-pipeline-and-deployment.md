# ADR-005: GitHub-Based Content Pipeline and Deployment Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-04
- **Feature:** Physical AI & Humanoid Robotics Book
- **Context:** Content generation workflow (multi-agent) → validation (quality gates) → deployment (GitHub Pages) requires automation, version control, and rollback capability. Manual deployment creates error risk and delays. Public repository enables community contributions and transparency.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ Long-term - defines publication and versioning for entire book lifecycle
     2) Alternatives: ✅ Multiple options (manual deployment, Netlify, Vercel, self-hosted)
     3) Scope: ✅ Cross-cutting - affects all content updates, CI/CD, community engagement
-->

## Decision

Adopt **GitHub-Centric Content Pipeline** with automated CI/CD and GitHub Pages deployment:

**Version Control & Collaboration:**
- **Repository**: Public GitHub repository (enables community contributions, transparent development)
- **Branching**: Feature branches per chapter (e.g., `002-physical-ai-humanoid-robotics-book`)
- **Content Storage**: Markdown in `/docs/`, code in `/examples/`, planning in `/specs/`, history in `/history/`
- **Pull Request Workflow**: All content changes via PR (enables review before merge)

**CI/CD Pipeline** (GitHub Actions):
1. **Build & Test Workflow** (`.github/workflows/build-deploy.yml`)
   - **Trigger**: Push to main, PR to main
   - **Steps**:
     - Install Node.js 18 + dependencies (`npm ci`)
     - Build Docusaurus (`npm run build`)
     - Run Lighthouse audit (Performance ≥90, Accessibility = 100)
     - Upload build artifacts
   - **Outcome**: Build success/failure status visible on PR

2. **Code Validation Workflow** (`.github/workflows/validate-code.yml`)
   - **Trigger**: PR with changes to `/examples/`
   - **Steps**:
     - Install ROS 2 Humble (Docker or apt)
     - Build ROS 2 workspace (`colcon build`)
     - Run tests (`colcon test`)
     - Report test results
   - **Outcome**: Code examples validated before merge

3. **Deployment Workflow** (part of build-deploy.yml)
   - **Trigger**: Push to main (after PR merge)
   - **Steps**:
     - Download build artifacts
     - Deploy to GitHub Pages (`gh-pages` branch via `peaceiris/actions-gh-pages`)
   - **Outcome**: Site updated at `https://<username>.github.io/humanoid-robotics/`

**Deployment Target:**
- **Platform**: GitHub Pages (static hosting)
- **URL**: `https://<username>.github.io/humanoid-robotics/` (custom domain optional)
- **Constraints**: 1GB site size limit, static files only (no server-side logic)
- **Performance**: Served via GitHub CDN (fast global distribution)

**Rollback Strategy:**
- **Git Revert**: Revert problematic commit → re-trigger deployment
- **Branch Protection**: Require status checks (build + tests pass) before merge
- **Preview Deployments**: PR comments include preview URL (Netlify/Vercel for PRs, optional)

**Asset Management:**
- **Small Assets**: Images <500KB in `/static/` (deployed with site)
- **Large Assets**: Videos, 3D models hosted on external CDN (YouTube, AWS S3) (avoids GitHub Pages 1GB limit)

## Consequences

### Positive

- **Zero-Cost Hosting**: GitHub Pages free for public repos (no hosting fees)
- **Automated Deployment**: Push to main → CI builds → site updates (no manual steps)
- **Quality Enforcement**: PR cannot merge if build fails or Lighthouse <90 (prevents broken deployments)
- **Rollback Safety**: Git history enables instant rollback to previous working version
- **Community Contributions**: Public repo enables external contributors to fix errors, add chapters
- **Transparency**: All planning docs (spec, plan, tasks, ADRs, PHRs) visible in repo (educational for learners studying SDD)
- **Versioning**: Docusaurus versioning enables ROS 2 Humble vs Jazzy versions side-by-side
- **Audit Trail**: GitHub commit history + CI logs provide full deployment audit

### Negative

- **Public Exposure**: All content visible before official "launch" (mitigated by clear README stating "work in progress")
- **GitHub Pages Constraints**: 1GB limit requires external CDN for videos (added complexity)
- **Build Time**: GitHub Actions free tier has concurrency limits (2 concurrent jobs) → PRs may queue
- **No Server-Side Logic**: Cannot run ROS 2 nodes or simulations on GitHub Pages (simulation examples run locally only)
- **GitHub Dependency**: If GitHub Pages discontinued, migration required (mitigated by static site portability)

## Alternatives Considered

**Alternative A: Netlify**
- **Approach**: Deploy Docusaurus to Netlify (similar to GitHub Pages but with preview deploys, form handling)
- **Rejected Because**:
  - Free tier: 100GB bandwidth/month (may be insufficient for video-heavy content)
  - Less integrated with GitHub (requires separate Netlify account, configuration)
  - GitHub Pages sufficient for static site (Netlify features like forms unused)
  - Community familiarity (GitHub Pages more recognizable for open-source projects)

**Alternative B: Vercel**
- **Approach**: Deploy to Vercel (Next.js creators, optimized for React apps)
- **Rejected Because**:
  - Optimized for Next.js, not Docusaurus (overkill for static site)
  - Free tier: 100GB bandwidth, but vendor lock-in
  - GitHub Pages free tier more generous (no bandwidth cap)
  - Vercel edge functions unused (static site only)

**Alternative C: Self-Hosted (AWS S3 + CloudFront, Google Cloud Storage)**
- **Approach**: Host static site on cloud storage with CDN
- **Rejected Because**:
  - Cost: S3 + CloudFront ~$5-10/month (GitHub Pages free)
  - Operational overhead (SSL certificates, DNS, monitoring)
  - Requires credit card / cloud account (barrier for community contributors)
  - GitHub Pages simpler (git push = deploy)

**Alternative D: Read the Docs**
- **Approach**: Use Read the Docs (Sphinx-based documentation hosting)
- **Rejected Because**:
  - Sphinx (reStructuredText) less flexible than Docusaurus (MDX)
  - No React component support (interactive quizzes difficult)
  - Primarily for API docs, not educational books with code examples
  - Docusaurus better suited for robotics + code-heavy content

**Alternative E: Manual Deployment (FTP Upload to Shared Hosting)**
- **Approach**: Build locally, upload via FTP to traditional web host
- **Rejected Because**:
  - No automation (manual build + upload for every change)
  - No CI validation (broken builds may deploy)
  - No version control integration (difficult to track what's deployed)
  - Error-prone (forgot to build before upload, deployed wrong files)

## References

- Feature Spec: [specs/002-physical-ai-humanoid-robotics-book/spec.md](../../specs/002-physical-ai-humanoid-robotics-book/spec.md) (FR-020: GitHub Pages Deployment, SC-017 to SC-020: Platform Metrics)
- Implementation Plan: [specs/002-physical-ai-humanoid-robotics-book/plan.md](../../specs/002-physical-ai-humanoid-robotics-book/plan.md) (Section 2.3: GitHub Actions CI/CD, Section 5.1: GitHub Pages Production Deployment)
- GitHub Actions Docs: https://docs.github.com/en/actions
- GitHub Pages Docs: https://docs.github.com/en/pages
- Related ADRs: ADR-002 (Docusaurus Stack), ADR-003 (Quality Gates - Build Validation), ADR-004 (ROS 2 Stack - Code Validation CI)
