# Skills Registry

**Project**: Physical AI & Humanoid Robotics Book
**Created**: 2025-12-04
**Version**: 1.0.0

## Overview

Skills are reusable, task-specific operations that can be invoked by agents to perform standardized workflows. Each skill has clear input parameters, output formats, and success criteria. Skills enable consistent execution of common tasks across the book development lifecycle.

## Skill Categories

### Content Planning & Structure
- **outline_chapter**: Generate pedagogically sound chapter outlines
- **coordinate_agents**: Orchestrate content requests across multiple domain agents

### Content Creation
- **write_glossary_entry**: Create standardized glossary term definitions
- **generate_exercises**: Design exercises spanning Bloom's taxonomy levels
- **create_quiz**: Generate assessment questions aligned to learning objectives

### Technical Content
- **validate_mathematics**: Verify mathematical equations against robotics standards
- **request_code_example**: Specify requirements for executable code from technical agents
- **design_diagram**: Request technical diagrams and visualizations

### Quality Assurance
- **review_content**: Perform comprehensive content quality checks
- **deploy_chapter**: Build, validate, and deploy chapter to Docusaurus

## Active Skills

### 1. outline_chapter
**Directory**: `.specify/skills/outline_chapter/`
**Version**: 1.0.0
**Primary Agent**: EducationDesigner
**Status**: Active

**Purpose**: Generate comprehensive, pedagogically sound chapter outlines following Constitution Principle II (Educational Accessibility & Structured Learning).

**Inputs**:
- chapter_number (integer, required)
- chapter_title (string, required)
- chapter_topic (string, required)
- target_word_count (integer, optional, default: 2000)
- prerequisites (array, required)
- difficulty_level (string, optional, default: "intermediate")
- special_requirements (object, optional)

**Outputs**:
- Comprehensive markdown outline with 9 sections
- Learning objectives (SMART, Bloom's taxonomy)
- Content structure (Overview → Quiz)
- Agent coordination plan
- Quality checklists
- Timeline estimates

**Success Criteria**:
- All sections have clear purpose and content elements
- Learning objectives span Bloom's levels (Remember → Create)
- Agent requests are specific and actionable
- Quality checkpoints embedded throughout

**Typical Usage**:
```yaml
EducationDesigner → outline_chapter(
  chapter_number: 9,
  chapter_title: "Kinematics",
  chapter_topic: "DH parameters, FK, IK",
  prerequisites: ["Chapter 2", "Linear Algebra"]
) → Comprehensive 4,800-word outline
```

**Estimated Time**: 1-2 hours

---

### 2. validate_mathematics
**Directory**: `.specify/skills/validate_mathematics/`
**Version**: 1.0.0
**Primary Agent**: RoboticsExpert
**Status**: Active

**Purpose**: Verify mathematical equations, derivations, and notation against standard robotics textbooks and constitution standards.

**Inputs**:
- equations (array of strings, LaTeX format, required)
- derivation_steps (array of strings, optional)
- notation_standard (string, optional, default: "Craig")
- textbook_reference (string, required)
- numerical_example (object, optional)

**Outputs**:
- Validation report (pass/fail for each equation)
- Notation compliance check
- Textbook cross-reference verification
- Suggested corrections if errors found

**Success Criteria**:
- 100% equations match textbook references
- All variables defined before use
- Units specified for physical quantities
- Numerical examples validate theoretical results

**Typical Usage**:
```yaml
RoboticsExpert → validate_mathematics(
  equations: ["T = \\begin{bmatrix}...\\end{bmatrix}"],
  textbook_reference: "Craig, J.J. Introduction to Robotics, eq. 3.6",
  notation_standard: "Craig"
) → Validation report: PASS
```

**Estimated Time**: 30-60 minutes per set of equations

---

### 3. generate_exercises
**Directory**: `.specify/skills/generate_exercises/`
**Version**: 1.0.0
**Primary Agent**: EducationDesigner
**Status**: Active

**Purpose**: Design exercises spanning all Bloom's taxonomy levels aligned to chapter learning objectives.

**Inputs**:
- learning_objectives (array, required)
- chapter_concepts (array, required)
- difficulty_level (string, required)
- number_of_exercises (integer, optional, default: 7)
- include_solutions (boolean, optional, default: true)

**Outputs**:
- 5-10 exercises organized by Bloom's level
- Problem statements with clear requirements
- Hints for challenging problems
- Complete solutions with explanations
- Rubrics for open-ended problems

**Success Criteria**:
- Exercises span all Bloom's levels (Remember → Create)
- At least 2 exercises per learning objective
- Progressive difficulty (easy → challenging)
- Solutions are complete and pedagogically explained

**Typical Usage**:
```yaml
EducationDesigner → generate_exercises(
  learning_objectives: ["Compute FK using DH", "Analyze Jacobian singularities"],
  chapter_concepts: ["DH parameters", "Transformation matrices", "Jacobian"],
  difficulty_level: "intermediate"
) → 7 exercises with solutions
```

**Estimated Time**: 1.5-2 hours

---

### 4. create_quiz
**Directory**: `.specify/skills/create_quiz/`
**Version**: 1.0.0
**Primary Agent**: EducationDesigner
**Status**: Active

**Purpose**: Generate assessment questions (MCQ, T/F, short answer) aligned to learning objectives with target pass rate >70%.

**Inputs**:
- learning_objectives (array, required)
- chapter_sections (array, required)
- number_of_questions (integer, optional, default: 12)
- question_types (object, optional, default: {mcq: 7, true_false: 3, short_answer: 2})
- target_pass_rate (float, optional, default: 0.70)

**Outputs**:
- 10-15 quiz questions with answers
- Question type breakdown (MCQ, T/F, short answer)
- Explanations for correct/incorrect answers
- Difficulty calibration (40% easy, 40% medium, 20% hard)
- Alignment matrix (questions → learning objectives)

**Success Criteria**:
- Every learning objective covered by ≥2 questions
- Distractors (wrong answers) based on common misconceptions
- Explanations provided for all answers
- Target pass rate achievable after reading chapter

**Typical Usage**:
```yaml
EducationDesigner → create_quiz(
  learning_objectives: ["Compute FK", "Explain IK", "Analyze Jacobian"],
  chapter_sections: ["Overview", "Core Concepts", "Math", "Code"],
  number_of_questions: 12
) → 12 quiz questions with explanations
```

**Estimated Time**: 1-1.5 hours

---

### 5. write_glossary_entry
**Directory**: `.specify/skills/write_glossary_entry/`
**Version**: 1.0.0
**Primary Agent**: EducationDesigner
**Status**: Active

**Purpose**: Create standardized glossary term definitions (1-3 sentences) with cross-references and notation.

**Inputs**:
- term (string, required)
- chapter_introduced (integer, required)
- technical_domain (string, required)
- related_terms (array, optional)
- mathematical_notation (string, optional, LaTeX)

**Outputs**:
- Concise definition (1-3 sentences)
- Synonyms and related terms
- Mathematical notation (if applicable)
- Chapter first introduced
- Cross-references to related terms

**Success Criteria**:
- Definition is technically accurate
- Length: 1-3 sentences (50-150 words)
- No circular definitions
- Links to related glossary terms
- Mathematical symbols use standard robotics notation

**Typical Usage**:
```yaml
EducationDesigner → write_glossary_entry(
  term: "Forward Kinematics",
  chapter_introduced: 9,
  technical_domain: "Kinematics",
  related_terms: ["Inverse Kinematics", "DH Parameters"],
  mathematical_notation: "$T = T_0 T_1 ... T_n$"
) → Glossary entry with cross-references
```

**Estimated Time**: 10-15 minutes per term

---

### 6. design_diagram
**Directory**: `.specify/skills/design_diagram/`
**Version**: 1.0.0
**Primary Agent**: DiagramAgent
**Status**: Active

**Purpose**: Request technical diagrams and visualizations (Mermaid, conceptual, mathematical) from DiagramAgent.

**Inputs**:
- diagram_type (string, required: "conceptual" | "mathematical" | "system_architecture" | "flowchart" | "sequence")
- description (string, required)
- key_elements (array, required)
- notation (string, optional)
- output_format (string, optional, default: "mermaid")

**Outputs**:
- Mermaid diagram code or image
- Caption and alt text
- Legend (if applicable)
- Technical accuracy validation

**Success Criteria**:
- Diagram clearly communicates concept
- Labels and annotations readable
- Follows engineering conventions (color coding, arrow styles)
- Caption describes purpose and key elements
- Alt text provided for accessibility

**Typical Usage**:
```yaml
EducationDesigner → design_diagram(
  diagram_type: "mathematical",
  description: "DH coordinate frames for 3-DOF planar arm",
  key_elements: ["Base frame", "Link 1", "Link 2", "Link 3", "End-effector"],
  notation: "Craig convention"
) → Mermaid diagram + caption
```

**Estimated Time**: 30-45 minutes per diagram

---

### 7. request_code_example
**Directory**: `.specify/skills/request_code_example/`
**Version**: 1.0.0
**Primary Agent**: ROS2Engineer (or SimulationEngineer, IsaacExpert)
**Status**: Active

**Purpose**: Specify requirements for executable code examples from technical agents with clear inputs, outputs, testing criteria.

**Inputs**:
- functionality (string, required)
- language (string, required: "Python" | "C++" | "YAML" | "XML")
- framework (string, optional: "ROS2" | "Gazebo" | "Isaac Sim" | "Unity")
- inputs_outputs (object, required)
- dependencies (array, required)
- testing_criteria (array, required)
- pedagogical_comments (boolean, optional, default: true)

**Outputs**:
- Executable code with inline comments
- Setup instructions (dependencies, environment)
- Example usage with expected output
- Unit tests (if applicable)
- Troubleshooting tips

**Success Criteria**:
- Code is syntactically correct
- Tested in target environment (Ubuntu 22.04 + ROS 2 Humble)
- Inline comments explain non-obvious logic
- Setup instructions are complete and accurate
- Example usage demonstrates key concepts

**Typical Usage**:
```yaml
EducationDesigner → request_code_example(
  functionality: "Forward kinematics service server",
  language: "Python",
  framework: "ROS2",
  inputs_outputs: {input: "joint_angles", output: "end_effector_pose"},
  dependencies: ["rclpy", "numpy", "geometry_msgs"],
  testing_criteria: ["Service responds within 100ms", "FK matches numerical example"]
) → ROS2Engineer → Executable Python code with tests
```

**Estimated Time**: 1-2 hours per code example

---

### 8. coordinate_agents
**Directory**: `.specify/skills/coordinate_agents/`
**Version**: 1.0.0
**Primary Agent**: EducationDesigner
**Status**: Active

**Purpose**: Orchestrate content requests across multiple domain agents (RoboticsExpert, ROS2Engineer, SimulationEngineer, IsaacExpert, VLAResearcher, DiagramAgent) with dependency tracking.

**Inputs**:
- chapter_outline (object, required)
- agent_requests (array, required)
- dependencies (object, required)
- deadline (string, optional)

**Outputs**:
- Agent request tracking sheet
- Dependency graph (which agents must complete before others)
- Timeline with milestones
- Status updates from each agent
- Consolidated content package

**Success Criteria**:
- All agent requests clearly specified
- Dependencies identified (e.g., RoboticsExpert must complete before EducationDesigner writes Math section)
- Timeline is realistic based on agent workload
- All deliverables received and validated
- Content integrated into cohesive chapter

**Typical Usage**:
```yaml
EducationDesigner → coordinate_agents(
  chapter_outline: {chapter_9_outline},
  agent_requests: [
    {agent: "RoboticsExpert", request: "DH derivation", deadline: "Day 2"},
    {agent: "ROS2Engineer", request: "FK service code", deadline: "Day 3"},
    {agent: "SimulationEngineer", request: "Gazebo viz", deadline: "Day 4"},
    {agent: "DiagramAgent", request: "Coordinate frames", deadline: "Day 2"}
  ],
  dependencies: {ROS2Engineer: ["RoboticsExpert"], SimulationEngineer: ["ROS2Engineer"]}
) → Tracking sheet + consolidated content after Day 4
```

**Estimated Time**: 30 minutes setup + agent execution time

---

### 9. review_content
**Directory**: `.specify/skills/review_content/`
**Version**: 1.0.0
**Primary Agent**: ValidationAgent
**Status**: Active

**Purpose**: Perform comprehensive content quality checks (math validation, code testing, consistency, accessibility, constitution compliance).

**Inputs**:
- chapter_markdown (string, required)
- validation_scope (array, required: ["math", "code", "consistency", "accessibility", "constitution"])
- quality_thresholds (object, optional)

**Outputs**:
- Validation report with pass/fail for each check
- List of errors and suggested corrections
- Consistency issues (notation, terminology, formatting)
- Accessibility audit results (alt text, semantic HTML, WCAG 2.1 AA)
- Constitution compliance score

**Success Criteria**:
- 100% mathematical equations validated
- 100% code examples tested and executable
- Zero consistency errors (notation, terminology)
- WCAG 2.1 AA compliance (100%)
- Constitution principles I-VII met

**Typical Usage**:
```yaml
ValidationAgent → review_content(
  chapter_markdown: {chapter_9_content},
  validation_scope: ["math", "code", "consistency", "accessibility", "constitution"]
) → Validation report: 2 math errors, 1 code issue, 3 consistency warnings
→ EducationDesigner fixes issues → Resubmit → Validation report: PASS
```

**Estimated Time**: 1.5-2 hours per chapter

---

### 10. deploy_chapter
**Directory**: `.specify/skills/deploy_chapter/`
**Version**: 1.0.0
**Primary Agent**: DocusaurusArchitect
**Status**: Active

**Purpose**: Build, validate, and deploy chapter to Docusaurus site with performance and accessibility checks.

**Inputs**:
- chapter_markdown (string, required)
- chapter_number (integer, required)
- assets (array, optional: images, videos, code files)
- deployment_target (string, optional, default: "preview")

**Outputs**:
- Docusaurus build status (success/failure)
- Lighthouse performance scores (Performance, Accessibility, SEO)
- Broken link report
- Deployment URL (preview or production)
- Build time and bundle size metrics

**Success Criteria**:
- Docusaurus build succeeds without errors
- Lighthouse Performance ≥90
- Lighthouse Accessibility = 100 (WCAG 2.1 AA)
- Zero broken internal links
- Site loads in <3 seconds

**Typical Usage**:
```yaml
DocusaurusArchitect → deploy_chapter(
  chapter_markdown: {chapter_9_content},
  chapter_number: 9,
  assets: ["dh-frames.png", "fk_service.py", "gazebo_viz.mp4"],
  deployment_target: "preview"
) → Build success, Lighthouse: 92/100/100, Preview URL: https://...
```

**Estimated Time**: 5-10 minutes per chapter

---

## Skill Invocation Patterns

### Sequential Workflow (Chapter Generation)
```
1. outline_chapter → Generate blueprint
2. coordinate_agents → Request content from domain agents
   ├─ validate_mathematics → RoboticsExpert validates equations
   ├─ request_code_example → ROS2Engineer provides code
   ├─ design_diagram → DiagramAgent creates visualizations
3. write_glossary_entry → Define new terms (×10-15 terms)
4. generate_exercises → Create practice problems
5. create_quiz → Generate assessment
6. review_content → ValidationAgent checks quality
7. deploy_chapter → DocusaurusArchitect builds and deploys
```

### Parallel Workflows (Where Possible)
```
write_glossary_entry (×15) ← Can run in parallel
generate_exercises + create_quiz ← Can run in parallel after content complete
design_diagram (×3-5) ← Can run in parallel
```

### Iterative Workflows (Revision Cycles)
```
review_content → Errors found → EducationDesigner fixes → review_content → PASS
deploy_chapter → Performance <90 → DocusaurusArchitect optimizes → deploy_chapter → PASS
```

## Skill Development Guidelines

### Creating New Skills

1. **Identify Need**: Recurring task performed by agents with clear input/output
2. **Define Scope**: Specific, reusable, composable operation (not too broad)
3. **Specify Interface**: Clear input parameters, output formats, success criteria
4. **Document Usage**: Provide examples, typical time estimates, common patterns
5. **Version Control**: Track skill versions, maintain compatibility

### Skill File Structure
```
.specify/skills/
└── [skill-name]/
    ├── skill.md              # Main skill specification
    ├── examples/             # Usage examples
    │   └── example-1.yaml
    ├── templates/            # Output templates
    │   └── output-template.md
    └── tests/                # Validation tests
        └── test-skill.md
```

### Skill Metadata Template
```yaml
skill_name: [name]
version: 1.0.0
created: 2025-12-04
primary_agent: [agent_name]
supporting_agents: [agent_1, agent_2]
status: active | planned | deprecated
estimated_time: [time_range]
```

## Skills Roadmap

### Planned Skills (Future Development)

- **optimize_performance**: Identify and fix performance bottlenecks in Docusaurus build
- **generate_capstone**: Design capstone project integrating multiple chapters
- **create_adr**: Document architectural decisions with ADR template
- **automate_testing**: Run automated tests on code examples and simulations
- **generate_changelog**: Document chapter updates and versioning
- **export_pdf**: Generate PDF version of book from Docusaurus

### Deprecated Skills
- None yet

---

## Skill Metrics

### Usage Statistics (Target)
- **outline_chapter**: 12-18 uses (one per chapter)
- **generate_exercises**: 12-18 uses
- **create_quiz**: 12-18 uses
- **write_glossary_entry**: 100-150 uses (multiple terms per chapter)
- **validate_mathematics**: 50-100 uses (multiple equation sets per chapter)
- **request_code_example**: 50-100 uses (multiple code examples per chapter)
- **design_diagram**: 30-50 uses (multiple diagrams per chapter)
- **coordinate_agents**: 12-18 uses (orchestrate per chapter)
- **review_content**: 12-18 uses (validate each chapter)
- **deploy_chapter**: 12-18 uses (deploy each chapter)

### Success Metrics
- **Skill Reuse Rate**: >90% of chapters use standard skills
- **Time Savings**: Skills reduce chapter generation time by 30-40%
- **Consistency**: Skills ensure uniform quality across all chapters
- **Error Reduction**: Validation skills catch 95%+ errors before publication

---

**Skills Registry Version**: 1.0.0
**Last Updated**: 2025-12-04
**Total Active Skills**: 10
**Total Planned Skills**: 6
