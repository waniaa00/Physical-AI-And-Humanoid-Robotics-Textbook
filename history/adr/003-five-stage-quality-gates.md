# ADR-003: Five-Stage Quality Validation Gates

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-04
- **Feature:** Physical AI & Humanoid Robotics Book
- **Context:** Educational robotics content requires zero tolerance for technical errors (Constitution Principle I & VII). Incorrect mathematics, non-executable code, or inconsistent notation undermines learner trust. Manual review alone cannot scale to 15 chapters with 50-100 code examples.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ Long-term - defines quality assurance for all content, affects publication credibility
     2) Alternatives: ✅ Multiple options (manual review only, automated tests only, peer review)
     3) Scope: ✅ Cross-cutting - applies to all chapters, all agents, all deliverables
-->

## Decision

Implement **five sequential validation gates** that all content must pass before publication:

**Gate 1: Mathematical Validation** (RoboticsExpert + ValidationAgent)
- **Trigger**: After RoboticsExpert completes derivations
- **Checks**:
  - All equations cross-referenced with textbooks (Craig, Spong, Murray/Li/Sastry)
  - Notation consistency (θ, d, a, α for DH parameters; T ∈ SE(3) for transforms)
  - Dimensional analysis (forces in Newtons, torques in Nm)
  - Numerical examples validate theoretical equations
- **Pass Criteria**: 100% equations validated, zero notation errors
- **Failure Action**: Return to RoboticsExpert with corrections, re-validate

**Gate 2: Code Testing** (ROS2Engineer/SimulationEngineer/IsaacExpert + CI)
- **Trigger**: After code examples written
- **Checks**:
  - Syntax correctness (`python -m py_compile`, `colcon build`)
  - Executable in target environment (Ubuntu 22.04 + ROS 2 Humble)
  - Functional requirements met (FK service responds <100ms, matches numerical example)
  - Unit tests pass (`colcon test`)
- **Pass Criteria**: All code builds, all tests pass, no runtime errors
- **Failure Action**: Return to code author with error logs, fix and re-test

**Gate 3: Content Consistency** (ValidationAgent + EducationDesigner)
- **Trigger**: After EducationDesigner completes chapter draft
- **Checks**:
  - Terminology matches glossary (no "Forward Kinematics" vs "FK" mismatches)
  - Mathematical notation consistent across chapters
  - Formatting uniform (code blocks, callouts, headers)
  - Cross-references valid (no broken links to other chapters)
- **Pass Criteria**: Zero consistency errors
- **Failure Action**: EducationDesigner fixes, re-submit for validation

**Gate 4: Build Validation** (DocusaurusArchitect + GitHub Actions)
- **Trigger**: After chapter ready for deployment
- **Checks**:
  - Docusaurus build succeeds (`npm run build` exit code 0)
  - Lighthouse Performance ≥90
  - Lighthouse Accessibility = 100 (WCAG 2.1 AA)
  - Zero broken links (internal)
  - Site loads <3s
- **Pass Criteria**: All checks pass
- **Failure Action**: DocusaurusArchitect optimizes (image compression, code splitting), rebuild

**Gate 5: Constitution Compliance** (ValidationAgent + Human Review)
- **Trigger**: Before final production deployment
- **Checks**:
  - Principle I: Technical accuracy verified (Gates 1 & 2 passed)
  - Principle II: Pedagogical structure enforced (outline_chapter skill compliance)
  - Principle III: Modularity maintained (chapter self-contained, clear dependencies)
  - Principle IV: Consistency enforced (Gate 3 passed)
  - Principle V: AI-native workflow followed (PHRs documented)
  - Principle VI: Code standards met (PEP 8, ROS 2 style guide)
  - Principle VII: All quality gates passed
- **Pass Criteria**: 100% compliance across all principles
- **Failure Action**: Escalate to human review, remediate violations

**Automation:**
- Gates 2 & 4 automated via GitHub Actions CI/CD
- Gates 1, 3, 5 semi-automated (scripts + human validation)

## Consequences

### Positive

- **Zero Technical Errors**: Multi-stage validation catches math errors, code bugs, inconsistencies before publication
- **Scalability**: Automated gates (2, 4) scale to hundreds of code examples without manual review bottleneck
- **Audit Trail**: Each gate produces validation report (stored with chapter), traceable quality process
- **Early Detection**: Gate 1 catches math errors before Gate 2 code implementation (fail fast)
- **Confidence**: Learners can trust content accuracy (all equations textbook-validated, all code tested)
- **Constitution Enforcement**: Gate 5 ensures no principle violations slip through

### Negative

- **Iteration Overhead**: Failed validation → revision cycle → re-validation (adds 1-2 hours per chapter if errors found)
- **Bottleneck Risk**: If ValidationAgent becomes overloaded, gates create sequential dependency (mitigated by parallel chapter development)
- **False Positives**: Automated checks may flag non-issues (e.g., Lighthouse performance affected by network, not code) (mitigated by human override)
- **Maintenance**: Validation scripts require updates when tools change (e.g., ROS 2 Humble → Jazzy migration)

## Alternatives Considered

**Alternative A: Manual Review Only**
- **Approach**: Human experts review all content before publication
- **Rejected Because**:
  - Does not scale to 50-100 code examples (manual execution testing infeasible)
  - Human error possible (missed notation inconsistency, forgot to test code)
  - Bottleneck: single reviewer delays all chapters
  - No audit trail (review comments lost)

**Alternative B: Automated Testing Only (No Math Validation)**
- **Approach**: CI runs code tests + Lighthouse, skip math validation
- **Rejected Because**:
  - Violates Constitution Principle I (mathematical accuracy non-negotiable)
  - Code may execute but compute wrong results (e.g., FK incorrect due to math error)
  - No consistency enforcement (chapters diverge in terminology)

**Alternative C: Peer Review (External Roboticists)**
- **Approach**: Submit chapters to external reviewers for validation
- **Rejected Because**:
  - Slow feedback loop (weeks for external review)
  - Inconsistent standards across reviewers
  - Not sustainable for iterative content updates
  - External reviewers may not understand agent workflow

**Alternative D: Post-Publication Error Correction**
- **Approach**: Publish first, fix errors reported by learners
- **Rejected Because**:
  - Violates Constitution Principle VII (quality gates NON-NEGOTIABLE)
  - Damages credibility (learners lose trust if errors found)
  - Reactive rather than proactive quality

## References

- Feature Spec: [specs/002-physical-ai-humanoid-robotics-book/spec.md](../../specs/002-physical-ai-humanoid-robotics-book/spec.md) (Success Criteria SC-005 to SC-008: Quality Metrics)
- Implementation Plan: [specs/002-physical-ai-humanoid-robotics-book/plan.md](../../specs/002-physical-ai-humanoid-robotics-book/plan.md) (Section 1.4: Validation Gates Definition, Evaluation & Testing Workflow)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (Principle VII: Quality Gates & Validation)
- Contracts: [specs/002-physical-ai-humanoid-robotics-book/contracts/validation-gates.md](../../specs/002-physical-ai-humanoid-robotics-book/contracts/validation-gates.md) (when created)
- Related ADRs: ADR-001 (Multi-Agent Architecture), ADR-004 (Content Pipeline)
