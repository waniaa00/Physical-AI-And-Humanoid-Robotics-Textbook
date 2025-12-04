# ADR-001: Multi-Agent Content Generation Architecture

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-04
- **Feature:** Physical AI & Humanoid Robotics Book
- **Context:** Educational content generation at scale (12-18 chapters, 30,000 words, 50-100 code examples) requires specialized domain expertise in robotics mathematics, ROS 2, simulation, and AI systems. Traditional single-author or monolithic approaches would create bottlenecks and consistency issues.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: ✅ Long-term consequence - defines entire authoring workflow for multi-year project
     2) Alternatives: ✅ Multiple viable options (single agent, human-only, template-based)
     3) Scope: ✅ Cross-cutting - affects all content generation, quality control, deployment
-->

## Decision

Adopt a **multi-agent orchestration architecture** with 8 specialized domain agents executing 10 reusable skills:

**Agents:**
- **BookPlanner**: Architecture and implementation strategy
- **EducationDesigner**: Content integration and pedagogical structure (orchestrator)
- **RoboticsExpert**: Mathematical validation and robotics theory
- **ROS2Engineer**: ROS 2 code examples and workspace management
- **SimulationEngineer**: Gazebo and Unity simulation environments
- **IsaacExpert**: NVIDIA Isaac Sim and synthetic data generation
- **VLAResearcher**: Vision-Language-Action systems and AI integration
- **DocusaurusArchitect**: Static site infrastructure and deployment

**Skills** (reusable operations):
- `outline_chapter`, `validate_mathematics`, `generate_exercises`, `create_quiz`, `write_glossary_entry`, `design_diagram`, `request_code_example`, `coordinate_agents`, `review_content`, `deploy_chapter`

**Coordination Pattern:**
- EducationDesigner orchestrates per-chapter workflows
- Domain agents provide specialized content via skill invocations
- ValidationAgent enforces quality gates at each handoff
- DocusaurusArchitect handles build and deployment

## Consequences

### Positive

- **Domain Expertise**: Each agent specializes in narrow domain (ROS 2, robotics math, Isaac Sim), ensuring technical accuracy
- **Parallel Execution**: Multiple chapters can be developed concurrently; within chapters, agents work in parallel (math + code + diagrams)
- **Reusability**: 10 skills standardize common operations, reducing inconsistency
- **Quality Gates**: Validation embedded at agent boundaries (math validation, code testing, consistency checks)
- **Scalability**: Adding Chapter 16-18 requires no architectural changes, just executing same workflow
- **Audit Trail**: All agent interactions documented via PHRs (Prompt History Records)

### Negative

- **Coordination Complexity**: 8 agents with handoff protocols require careful orchestration (mitigated by agent-coordination contracts)
- **Single Points of Failure**: EducationDesigner as orchestrator could become bottleneck (mitigated by clear skill contracts and parallel work)
- **Learning Curve**: Contributors must understand agent roles and skill invocation patterns (mitigated by quickstart guide)
- **Debugging Difficulty**: Multi-agent failures harder to trace than monolithic workflow (mitigated by PHR audit trail)

## Alternatives Considered

**Alternative A: Single General-Purpose Agent**
- **Approach**: One LLM agent writes entire chapter (math, code, exercises, deployment)
- **Rejected Because**:
  - No domain specialization → higher error rate in robotics math and ROS 2 code
  - Sequential bottleneck → cannot parallelize math validation + code generation
  - No clear quality gates → errors propagate downstream

**Alternative B: Human-Only Authoring with AI Assistance**
- **Approach**: Human authors write chapters, use AI for editing/suggestions only
- **Rejected Because**:
  - Scales poorly (1 author × 15 chapters × 2 weeks = 30 weeks sequential)
  - Consistency issues across chapters (different human writing styles)
  - Code examples require manual testing (no automated validation pipeline)

**Alternative C: Template-Based Generation with Placeholders**
- **Approach**: Static chapter templates with placeholders filled by single agent
- **Rejected Because**:
  - Rigid structure limits pedagogical flexibility
  - No domain expertise injection (all content from one model)
  - Templates don't handle complexity of robotics math derivations

## References

- Feature Spec: [specs/002-physical-ai-humanoid-robotics-book/spec.md](../../specs/002-physical-ai-humanoid-robotics-book/spec.md)
- Implementation Plan: [specs/002-physical-ai-humanoid-robotics-book/plan.md](../../specs/002-physical-ai-humanoid-robotics-book/plan.md) (Sections: Multi-Agent Collaboration Flow, Agent-to-Skill Mapping)
- Agent Definitions: [.specify/agents/README.md](../../.specify/agents/README.md)
- Skills Registry: [.specify/skills/SKILLS.md](../../.specify/skills/SKILLS.md)
- Related ADRs: ADR-003 (Quality Gates), ADR-004 (Content Pipeline)
