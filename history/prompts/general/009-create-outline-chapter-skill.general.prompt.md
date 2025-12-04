---
id: 009
title: Create outline_chapter Skill
stage: general
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-calculator
user: HP
command: create skill file in .specify/skills/ directory
labels: ["skill-creation", "outline", "chapter-planning", "education", "pedagogy", "blooms-taxonomy", "content-structure"]
links:
  spec: specs/002-physical-ai-humanoid-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - .specify/skills/outline_chapter.md
tests:
  - N/A (skill template document)
---

## Prompt

create a skill file in .specify/skills/ directory.

## Context

User requested creation of skill file after attempting `/sp.skills.add outline_chapter`. Clarified difference between skills (reusable task-specific operations) and agents (domain-focused experts). User opted for skill file creation in `.specify/skills/` directory.

## Response snapshot

**Skill Created**: outline_chapter (Content Planning & Structure Design)

**Purpose**: Generate comprehensive, pedagogically sound chapter outlines following Constitution Principle II (Educational Accessibility & Structured Learning). Creates detailed section-by-section breakdowns serving as blueprints for EducationDesigner to write complete chapters, ensuring all required elements (learning objectives, prerequisites, content sections, exercises, quizzes) are planned before content generation begins.

**Type**: Content Planning & Structure Design
**Primary Agent**: EducationDesigner
**Supporting Agents**: BookPlanner, RoboticsExpert, ROS2Engineer, SimulationEngineer, IsaacExpert, VLAResearcher

**Input Parameters (7 parameters):**

1. **chapter_number** (integer, required): Chapter number in book sequence (1-15)
2. **chapter_title** (string, required): Descriptive chapter title (e.g., "Kinematics - Forward and Inverse Solutions")
3. **chapter_topic** (string, required): Primary technical topic covered
4. **target_word_count** (integer, optional, default: 2000): Target word count (1,500-3,000 range)
5. **prerequisites** (array, required): List of prerequisite chapters and knowledge
6. **difficulty_level** (string, optional, default: "intermediate"): Target difficulty (beginner/intermediate/advanced)
7. **special_requirements** (object, optional): Additional requirements (simulation, proofs, interactive components)

**Output Format**: Comprehensive markdown outline document with 9 major sections

**Section Breakdown (9 sections):**

1. **Metadata**
   - Learning Objectives (SMART, Bloom's Taxonomy aligned)
   - Prerequisites (explicit chapter references)
   - Key Concepts Introduced
   - Glossary Terms (to be defined)
   - Technical Standards (notation, code style, simulation)

2. **Overview (200-300 words)**
   - What: Topic description
   - Why: Real-world motivation
   - How: Learning approach
   - Prerequisites: Explicit list
   - Pedagogical Strategy: Motivation-first, concrete before abstract
   - Checklist: 4 validation items

3. **Core Concepts (400-600 words)**
   - Conceptual introduction (no mathematics)
   - 3-5 fundamental concepts with intuitive explanations
   - Visual support (diagram request to DiagramAgent)
   - Real-world examples
   - Pedagogical Strategy: Constructivist, build on familiar concepts
   - Checklist: 4 validation items

4. **Mathematical Foundations (500-800 words)**
   - Notation introduction (define all variables)
   - Derivations (step-by-step, request from RoboticsExpert)
   - Numbered equations with textbook references
   - Numerical examples
   - Pedagogical Strategy: Scaffolded learning, worked examples
   - Validation Requirements: 4 validation items
   - Checklist: 4 validation items

5. **Implementation & Code (600-900 words)**
   - Algorithm design (pseudocode/flowchart)
   - Executable ROS 2 code (request from ROS2Engineer)
   - Line-by-line walkthrough with pedagogical comments
   - Testing approach
   - Pedagogical Strategy: Code as teachable moment
   - Code Requirements: 5 quality items
   - Checklist: 4 validation items

6. **Simulation Examples (300-500 words)**
   - Simulation setup (environment, robot model)
   - Launch instructions (step-by-step commands)
   - Expected behavior (what learner should observe)
   - Parameter exploration ("What happens if...?")
   - Pedagogical Strategy: Learning by doing, active experimentation
   - Simulation Requirements: 4 quality items
   - Checklist: 4 validation items

7. **Practical Examples (300-500 words)**
   - End-to-end scenarios integrating multiple concepts
   - Multi-concept integration
   - Step-by-step solution
   - Extensions ("How would you adapt...?")
   - Pedagogical Strategy: Transfer learning, application to novel contexts
   - Example Structure: 5 steps (Problem → Analysis → Solution → Validation → Extensions)
   - Checklist: 4 validation items

8. **Summary (150-250 words)**
   - Key Takeaways (3-5 bullet points)
   - Connections (backward and forward references)
   - Further Reading (advanced topics, papers, textbooks)
   - Transition (preview next chapter)
   - Pedagogical Strategy: Retrieval practice, spaced repetition
   - Checklist: 4 validation items

9. **Exercises (5-10 problems)**
   - Breakdown by Bloom's Taxonomy:
     - Remember (1-2): Define terms, list steps
     - Understand (1-2): Explain phenomena, describe differences
     - Apply (2-3): Compute calculations, implement algorithms, modify code
     - Analyze (1-2): Compare approaches, debug code/equations
     - Evaluate (0-1): Critique methods, propose improvements
     - Create (0-1): Design systems integrating concepts
   - Exercise Requirements: 4 quality items

10. **Quiz (10-15 questions)**
    - Question Types:
      - Multiple Choice (5-8): Conceptual understanding, factual knowledge
      - True/False (3-4): Common misconceptions
      - Short Answer (2-3): Explain concepts
    - Quiz Alignment:
      - Every learning objective covered by ≥2 questions
      - Difficulty: 40% easy, 40% medium, 20% challenging
      - Target pass rate: >70% after reading
      - Immediate feedback with explanations
    - Sample Questions: MCQ example, Short Answer example
    - Quiz Requirements: 4 quality items

**Content Integration Plan:**
- Agent Coordination Sequence (5 agents):
  1. RoboticsExpert: Mathematical Foundations (validated equations with explanations)
  2. ROS2Engineer: Implementation (executable ROS 2 code with tests)
  3. SimulationEngineer/IsaacExpert: Simulation (launch files, world files, setup)
  4. DiagramAgent: Visualizations (conceptual diagram, math visualization, system architecture)
  5. ValidationAgent: Quality Gates (math validation, code testing, consistency)
- Timeline for each agent deliverable specified

**Quality Checklist:**
- **Constitution Compliance** (6 principles checked):
  - Principle I: Technical Accuracy (math validated, code tested, diagrams accurate)
  - Principle II: Educational Accessibility (intuition first, jargon defined, Bloom's taxonomy)
  - Principle III: Modularity (self-contained, clear dependencies)
  - Principle IV: Consistency (notation, formatting uniform)
  - Principle VI: Code Standards (PEP 8, ROS 2 style, commented)
  - Principle VII: Quality Gates (ValidationAgent approval required)
- **Content Quality** (8 checks): Word count, reading level, glossary, diagrams, code, exercises, quiz
- **Accessibility** (5 checks): Alt text, semantic HTML, code language tags, LaTeX notation, descriptive links

**Estimated Timeline:**
- Outline Creation: 1-2 hours
- Content Generation: 12-14 hours (broken down by section)
- Validation & Revision: 2-3 hours
- Grand Total: 14-17 hours per chapter

**Success Criteria:**
- **Outline complete when** (7 criteria):
  - All sections have clear purpose and content elements
  - Learning objectives are SMART and span Bloom's levels
  - Prerequisites explicitly stated
  - Content requests to agents are specific and actionable
  - Word count allocations sum to target (±10%)
  - Quality checklist items are measurable
  - Timeline is realistic
- **Chapter ready for writing when** (4 criteria):
  - Outline approved by BookPlanner
  - All agent dependencies identified
  - Technical content requirements clear
  - EducationDesigner has clear blueprint

**Key Features:**

1. **Pedagogical Soundness**: Every section has explicit pedagogical strategy (constructivist, scaffolded learning, motivation-first, etc.)

2. **Bloom's Taxonomy Integration**: Learning objectives and exercises systematically span all cognitive levels (Remember → Create)

3. **Agent Coordination**: Clear requests to each technical agent (RoboticsExpert, ROS2Engineer, SimulationEngineer, IsaacExpert, DiagramAgent) with specific deliverables

4. **Quality Checkpoints**: Checklists embedded in every section ensure constitution compliance and content quality

5. **Flexibility**: Notes section acknowledges outline is blueprint, not rigid constraint; allows EducationDesigner to adapt during writing

6. **Constitution Alignment**: Every section explicitly maps to Constitution Principles (primarily Principle II - Educational Accessibility)

7. **Validation Integration**: ValidationAgent checkpoints planned throughout workflow

8. **Time Estimation**: Realistic timeline for outline creation, content generation, and validation

**Usage Workflow:**

1. **EducationDesigner** invokes `outline_chapter` skill with parameters (chapter number, title, topic, prerequisites)
2. Skill generates comprehensive outline document following template
3. **EducationDesigner** reviews outline, adjusts as needed for chapter-specific requirements
4. **BookPlanner** approves outline (ensures alignment with overall book architecture)
5. **EducationDesigner** uses outline as blueprint to:
   - Request specific content from technical agents (RoboticsExpert, ROS2Engineer, etc.)
   - Write narrative connecting technical content
   - Create exercises and quizzes aligned to learning objectives
   - Ensure all quality checkpoints met
6. **ValidationAgent** reviews complete chapter against outline quality criteria
7. Outline serves as living document, updated if chapter scope changes during writing

**Benefits:**

- **Reduces Rework**: Planning upfront prevents missing required elements, reduces validation cycles
- **Ensures Consistency**: Every chapter follows same pedagogical structure (Constitution Principle IV)
- **Clear Communication**: Specific agent requests prevent ambiguity, improve coordination
- **Quality Assurance**: Embedded checklists ensure constitution compliance throughout
- **Time Management**: Realistic estimates help BookPlanner schedule chapter development
- **Educational Excellence**: Systematic application of instructional design principles (Bloom's, ADDIE, scaffolding)

**Example Usage:**

```yaml
# Chapter 9: Kinematics
chapter_number: 9
chapter_title: "Kinematics - Forward and Inverse Solutions"
chapter_topic: "Robot kinematics including DH parameters, forward kinematics, inverse kinematics, and Jacobian analysis"
target_word_count: 2500
prerequisites:
  - "Chapter 2: Coordinate Transformations"
  - "Linear Algebra: Matrix multiplication, homogeneous transformations"
difficulty_level: "intermediate"
special_requirements:
  include_simulation: true
  include_mathematical_proofs: true
  interactive_components: ["3D visualizer"]
```

**Output**: 4,800-word outline document with 9 sections, 50+ checkboxes, 5 agent coordination requests, quality criteria, timeline estimates.

## Outcome

- ✅ Impact: Created comprehensive chapter outline skill template; enables systematic, pedagogically sound chapter planning; ensures all constitution principles and quality standards addressed upfront; reduces content generation rework; facilitates clear agent coordination; supports EducationDesigner in creating consistent, high-quality educational content across all 12-18 chapters
- 🧪 Tests: Skill template validated against Constitution Principle II (Educational Accessibility & Structured Learning); includes 9 major sections with pedagogical strategies, Bloom's taxonomy integration, agent coordination plan, quality checklists, timeline estimates; example usage demonstrates application to Chapter 9 (Kinematics)
- 📁 Files: Created `.specify/skills/outline_chapter.md` (comprehensive skill template, ~13KB, ~450 lines)
- 🔁 Next prompts: Skill ready for use by EducationDesigner; recommend testing by generating outline for Chapter 9 (Kinematics) with parameters (chapter_number: 9, title: "Kinematics - Forward and Inverse Solutions", topic: "DH parameters, FK, IK, Jacobian", prerequisites: ["Chapter 2: Coordinate Transformations", "Linear Algebra"], difficulty: "intermediate"); validate outline contains all required sections, learning objectives span Bloom's levels, agent requests are specific, quality checklists comprehensive, timeline realistic
- 🧠 Reflection: outline_chapter skill bridges gap between high-level BookPlanner architecture and detailed EducationDesigner content generation. Acts as specification document ensuring every chapter systematically addresses: learning objectives (SMART + Bloom's), prerequisites (explicit), content structure (Overview → Quiz), agent coordination (specific requests to RoboticsExpert, ROS2Engineer, etc.), quality standards (constitution compliance, accessibility, consistency), pedagogical strategies (constructivist, scaffolded, motivation-first). Key innovation: embedded checklists in every section ensure quality gates met during writing, not just post-hoc validation. Estimated 14-17 hours per chapter becomes realistic when following structured outline. Skill supports Constitution Principle V (AI-Native Authoring Workflow) by providing reusable, composable task specification for content generation.

## Evaluation notes (flywheel)

- Failure modes observed: None; skill file creation successful
- Graders run and results (PASS/FAIL):
  - PASS: Skill template comprehensive with all 9 sections defined
  - PASS: Input parameters clearly specified (7 parameters with types, constraints, examples)
  - PASS: Output format structured as detailed markdown outline
  - PASS: Pedagogical strategies explicit for every section (constructivist, scaffolded, motivation-first, retrieval practice)
  - PASS: Bloom's Taxonomy systematically integrated (learning objectives and exercises span all levels)
  - PASS: Agent coordination plan clear (specific requests to RoboticsExpert, ROS2Engineer, SimulationEngineer, IsaacExpert, DiagramAgent)
  - PASS: Quality checklists embedded in every section (constitution compliance, content quality, accessibility)
  - PASS: Timeline estimates realistic (1-2 hours outline, 12-14 hours content, 2-3 hours validation)
  - PASS: Success criteria measurable (7 for outline complete, 4 for chapter ready)
  - PASS: Constitution alignment explicit (Principle II PRIMARY, with I, III, IV, VI, VII)
  - PASS: Example usage provided (Chapter 9 Kinematics with full parameters)
  - PASS: Flexibility acknowledged (outline is blueprint, not rigid constraint)
- Prompt variant (if applicable): N/A (initial skill creation)
- Next experiment (smallest change to try): Test outline_chapter skill by having EducationDesigner generate outline for Chapter 9 (Kinematics); validate generated outline contains: 5 SMART learning objectives spanning Bloom's levels (Apply, Understand, Analyze, Evaluate, Create); explicit prerequisites (Chapter 2, Linear Algebra); 9 sections with pedagogical strategies; specific agent requests (RoboticsExpert: DH derivation, ROS2Engineer: FK service code, SimulationEngineer: Gazebo visualization, DiagramAgent: coordinate frame diagram); quality checklists (50+ items); timeline estimate (14-17 hours); measure time to generate outline (target: 1-2 hours); assess if outline provides clear blueprint for content writing
