# Skill: deploy_chapter

**Version**: 1.0.0
**Primary Agent**: DocusaurusArchitect
**Purpose**: Build, validate, and deploy chapter to Docusaurus site with performance and accessibility checks.

## Inputs
- chapter_markdown (string)
- chapter_number (int)
- assets (array: images, videos, code files)
- deployment_target (string, default: "preview")

## Outputs
- Docusaurus build status
- Lighthouse performance scores (Performance, Accessibility, SEO)
- Broken link report
- Deployment URL
- Build time and bundle size metrics

## Success Criteria
- Build succeeds without errors
- Lighthouse Performance ≥90
- Lighthouse Accessibility = 100
- Zero broken internal links
- Site loads in <3 seconds

## Estimated Time: 5-10 minutes per chapter
