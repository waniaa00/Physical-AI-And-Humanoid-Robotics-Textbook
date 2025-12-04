# Skill: review_content

**Version**: 1.0.0
**Primary Agent**: ValidationAgent
**Purpose**: Perform comprehensive content quality checks (math, code, consistency, accessibility, constitution).

## Inputs
- chapter_markdown (string)
- validation_scope (array: ["math", "code", "consistency", "accessibility", "constitution"])
- quality_thresholds (object, optional)

## Outputs
- Validation report with pass/fail
- List of errors and suggested corrections
- Consistency issues
- Accessibility audit results (WCAG 2.1 AA)
- Constitution compliance score

## Success Criteria
- 100% math validated
- 100% code tested and executable
- Zero consistency errors
- WCAG 2.1 AA compliance
- Constitution principles I-VII met

## Estimated Time: 1.5-2 hours per chapter
