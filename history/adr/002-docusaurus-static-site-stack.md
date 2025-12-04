# ADR-002: Docusaurus Static Site Technology Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-04
- **Feature:** Physical AI & Humanoid Robotics Book
- **Context:** Educational book requires web-accessible platform with fast load times (<3s), markdown support, code syntax highlighting, interactive components (quizzes), mobile responsiveness, and GitHub Pages deployment within 1GB limit.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ Long-term - defines publishing platform for entire book lifecycle
     2) Alternatives: ✅ Multiple options (GitBook, MkDocs, custom Next.js, Gatsby)
     3) Scope: ✅ Cross-cutting - affects all chapters, deployment, performance, maintainability
-->

## Decision

Adopt **Docusaurus v3 static site stack** with integrated tooling:

**Framework & Build:**
- **Static Site Generator**: Docusaurus v3.0+ (React-based, markdown-first)
- **Runtime**: Node.js 18+, React 18+
- **Build Tool**: Webpack (built into Docusaurus)
- **Deployment**: GitHub Pages (static hosting, free, integrated with repo)

**Content & Rendering:**
- **Content Format**: Markdown with MDX (JSX in markdown for interactive components)
- **Styling**: CSS Modules (built-in) + custom CSS for theme customization
- **Code Highlighting**: Prism.js (built-in with Python, YAML, XML, C# language support)
- **Diagrams**: Mermaid.js plugin + KaTeX for math equations

**Custom Components** (React/TypeScript):
- `LearningObjectives.tsx`, `ExerciseBlock.tsx`, `QuizQuestion.tsx`, `CodeSandbox.tsx`

**Performance:**
- Ideal Image plugin (WebP conversion, lazy loading)
- PWA plugin (offline access, service worker)
- Code splitting (automatic per route)

## Consequences

### Positive

- **Zero Config Start**: Docusaurus scaffolds complete site (`npx create-docusaurus`) with sensible defaults
- **Markdown Native**: Authors write in markdown, no JSX required for basic content
- **Versioning Built-in**: Future ROS 2 version updates (Humble → Jazzy) supported via Docusaurus versioning
- **Search Included**: Algolia DocSearch integration available (local search fallback)
- **GitHub Pages Native**: `npm run deploy` pushes to `gh-pages` branch automatically
- **Performance**: Static HTML + React hydration → fast initial load, Lighthouse ≥90 achievable
- **Active Ecosystem**: Large plugin ecosystem (i18n, PWA, sitemap, analytics)
- **Accessibility**: Semantic HTML structure by default, WCAG 2.1 AA compliant with proper alt text

### Negative

- **React Dependency**: All custom components must be React (no Vue/Svelte alternatives)
- **Build Time**: Webpack rebuilds can be slow for 15+ chapters (mitigated by incremental builds)
- **Bundle Size**: React + Docusaurus core ~200KB gzipped (mitigated by code splitting)
- **Customization Limits**: Deep theme changes require ejecting or swizzling components
- **GitHub Pages Constraints**: 1GB site limit, static only (no server-side logic)

## Alternatives Considered

**Alternative A: GitBook**
- **Approach**: GitBook SaaS platform for technical documentation
- **Rejected Because**:
  - Vendor lock-in (content hosted on GitBook servers)
  - Limited customization (cannot add interactive quiz components)
  - Paid plans required for advanced features (analytics, custom domain)
  - Export to self-hosted format complex

**Alternative B: MkDocs Material**
- **Approach**: Python-based static site generator with Material Design theme
- **Rejected Because**:
  - No native React component support (interactive quizzes difficult)
  - Python dependency incompatible with JavaScript-heavy ROS 2 examples
  - Plugin ecosystem smaller than Docusaurus
  - Less suitable for code-heavy content (inferior syntax highlighting)

**Alternative C: Custom Next.js Site**
- **Approach**: Build custom site with Next.js App Router + markdown rendering
- **Rejected Because**:
  - High development overhead (navigation, search, versioning must be built)
  - Requires custom build/deployment pipeline (GitHub Actions complexity)
  - No markdown tooling out-of-box (must integrate remark/rehype manually)
  - Over-engineered for static content (Next.js server features unused)

**Alternative D: Gatsby**
- **Approach**: GraphQL-based static site generator
- **Rejected Because**:
  - GraphQL overhead unnecessary for simple markdown content
  - Slower build times than Docusaurus (all pages rebuild on change)
  - Steeper learning curve (GraphQL schema, data layer complexity)
  - Less documentation-focused than Docusaurus

## References

- Feature Spec: [specs/002-physical-ai-humanoid-robotics-book/spec.md](../../specs/002-physical-ai-humanoid-robotics-book/spec.md) (FR-016 to FR-020: Platform Integration)
- Implementation Plan: [specs/002-physical-ai-humanoid-robotics-book/plan.md](../../specs/002-physical-ai-humanoid-robotics-book/plan.md) (Section 2.1: Docusaurus Project Initialization, Section 2.4: Performance Optimization)
- Docusaurus Docs: https://docusaurus.io/
- Related ADRs: ADR-004 (Content Pipeline), ADR-005 (Deployment Strategy)
