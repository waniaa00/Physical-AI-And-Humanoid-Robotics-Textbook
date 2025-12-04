# Skill: outline_chapter

**Version**: 1.0.0
**Created**: 2025-12-04
**Type**: Content Planning & Structure Design
**Primary Agent**: EducationDesigner
**Supporting Agents**: BookPlanner, RoboticsExpert, ROS2Engineer, SimulationEngineer, IsaacExpert, VLAResearcher

## Purpose

Generate a comprehensive, pedagogically sound chapter outline following the constitution's structured learning principles (Principle II). This skill creates detailed section-by-section breakdowns that serve as blueprints for EducationDesigner to write complete chapters, ensuring all required elements (learning objectives, prerequisites, content sections, exercises, quizzes) are planned before content generation begins.

## Input Parameters

```yaml
parameters:
  chapter_number:
    type: integer
    required: true
    description: "Chapter number in book sequence (1-15)"
    example: 9

  chapter_title:
    type: string
    required: true
    description: "Descriptive chapter title"
    example: "Kinematics - Forward and Inverse Solutions"

  chapter_topic:
    type: string
    required: true
    description: "Primary technical topic covered"
    example: "Robot kinematics including DH parameters, forward kinematics, inverse kinematics, and Jacobian analysis"

  target_word_count:
    type: integer
    required: false
    default: 2000
    description: "Target word count for chapter (1,500-3,000 range)"
    constraints: "Must be between 1,500 and 3,000"

  prerequisites:
    type: array
    required: true
    description: "List of prerequisite chapters and knowledge"
    example:
      - "Chapter 2: Coordinate Transformations"
      - "Linear Algebra: Matrix multiplication, homogeneous transformations"

  difficulty_level:
    type: string
    required: false
    default: "intermediate"
    enum: ["beginner", "intermediate", "advanced"]
    description: "Target difficulty for exercises and content depth"

  special_requirements:
    type: object
    required: false
    description: "Additional requirements for chapter"
    properties:
      include_simulation:
        type: boolean
        default: true
      include_mathematical_proofs:
        type: boolean
        default: false
      interactive_components:
        type: array
        items: ["3D visualizer", "code playground", "video"]
```

## Output Format

```markdown
# Chapter {number}: {title} - Outline

**Status**: Draft Outline
**Created**: {date}
**Target Word Count**: {word_count}
**Difficulty**: {difficulty_level}
**Estimated Reading Time**: {time} hours

---

## Metadata

**Learning Objectives** (SMART - Specific, Measurable, Achievable, Relevant, Time-bound):
1. [Bloom: Apply] {objective_1}
2. [Bloom: Understand] {objective_2}
3. [Bloom: Analyze] {objective_3}
4. [Bloom: Evaluate] {objective_4}
5. [Bloom: Create] {objective_5}

**Prerequisites**:
- {prerequisite_1}
- {prerequisite_2}

**Key Concepts Introduced**:
- {concept_1}
- {concept_2}
- {concept_3}

**Glossary Terms** (to be defined):
- {term_1}: {brief_definition}
- {term_2}: {brief_definition}

**Technical Standards**:
- Notation: {mathematical_notation_standards}
- Code Style: {code_style_requirements}
- Simulation: {simulation_requirements}

---

## Section Breakdown

### 1. Overview (200-300 words)

**Purpose**: Motivate chapter topic, provide roadmap, activate prior knowledge

**Content Elements**:
- **What**: One-paragraph description of chapter topic
- **Why**: Real-world motivation and relevance (specific application examples)
- **How**: Learning approach and chapter structure overview
- **Prerequisites**: Explicit list with chapter references

**Pedagogical Strategy**: Motivation-first approach, concrete before abstract

**Checklist**:
- [ ] Answers "Why should I care about this topic?"
- [ ] Provides concrete real-world example
- [ ] Links to previous chapters explicitly
- [ ] Sets expectations for chapter complexity

**Estimated Word Count**: 250 words

---

### 2. Core Concepts (400-600 words)

**Purpose**: Intuitive understanding before formal technical presentation

**Content Elements**:
- **Conceptual Introduction**: Explain topic without mathematics (analogies, metaphors)
- **Key Ideas**: 3-5 fundamental concepts with intuitive explanations
- **Visual Support**: Request diagram from DiagramAgent (conceptual, non-mathematical)
- **Examples**: Real-world scenarios demonstrating concepts

**Pedagogical Strategy**: Constructivist approach, build on familiar concepts

**Content Requests**:
- [ ] DiagramAgent: Conceptual diagram showing {topic_visualization}
- [ ] Analogy: {analogy_to_familiar_concept}
- [ ] Example: {real_world_application}

**Checklist**:
- [ ] Zero mathematical notation in this section
- [ ] Uses analogies learners can relate to
- [ ] Includes at least one diagram
- [ ] Progressive disclosure (simple → complex)

**Estimated Word Count**: 500 words

---

### 3. Mathematical Foundations (500-800 words)

**Purpose**: Formal technical presentation with validated mathematics

**Content Elements**:
- **Notation Introduction**: Define all variables, coordinate frames, conventions
- **Derivations**: Step-by-step mathematical development (request from RoboticsExpert)
- **Equations**: Numbered equations with textbook references
- **Numerical Examples**: Concrete numbers demonstrating theory

**Pedagogical Strategy**: Scaffolded learning, worked examples before general case

**Content Requests**:
- [ ] RoboticsExpert: {specific_derivation_request}
- [ ] RoboticsExpert: Numerical example with solution
- [ ] DiagramAgent: Mathematical visualization (e.g., coordinate frames, vectors)

**Validation Requirements**:
- [ ] All equations validated against standard textbook ({textbook_reference})
- [ ] Variables defined before first use
- [ ] Units specified for all physical quantities
- [ ] Cross-references to equation numbers

**Checklist**:
- [ ] Follows constitution notation standards (q for joints, T for SE(3))
- [ ] Each equation has pedagogical commentary
- [ ] Numerical example validates theoretical result
- [ ] Links theory to upcoming implementation

**Estimated Word Count**: 700 words

---

### 4. Implementation & Code (600-900 words)

**Purpose**: Translate theory into executable code

**Content Elements**:
- **Algorithm Design**: Pseudocode or flowchart before implementation
- **Code Examples**: Executable ROS 2 code (request from ROS2Engineer)
- **Line-by-Line Walkthrough**: Pedagogical comments explaining design decisions
- **Testing**: How to verify implementation correctness

**Pedagogical Strategy**: Code as teachable moment, not just solution

**Content Requests**:
- [ ] ROS2Engineer: {specific_code_implementation}
- [ ] ROS2Engineer: Unit tests demonstrating correctness
- [ ] (Optional) SimulationEngineer: Gazebo/Unity visualization
- [ ] (Optional) IsaacExpert: Isaac Sim implementation

**Code Requirements**:
- [ ] PEP 8 compliance (Python) or ROS 2 style guide
- [ ] Inline comments explaining non-obvious logic
- [ ] Type hints for functions
- [ ] Docstrings for modules/classes/functions
- [ ] Example usage with expected output

**Checklist**:
- [ ] Code is syntactically correct and tested
- [ ] Setup instructions provided (dependencies, environment)
- [ ] Error handling demonstrated where relevant
- [ ] Links code back to mathematical formulation

**Estimated Word Count**: 800 words

---

### 5. Simulation Examples (300-500 words)

**Purpose**: Visualize concepts in realistic environment

**Content Elements**:
- **Simulation Setup**: Environment description, robot model
- **Launch Instructions**: Step-by-step commands to run simulation
- **Expected Behavior**: What learner should observe
- **Parameter Exploration**: Encourage experimentation ("What happens if...?")

**Pedagogical Strategy**: Learning by doing, active experimentation

**Content Requests**:
- [ ] SimulationEngineer: {simulator} world file with {robot_model}
- [ ] (Alternative) IsaacExpert: Isaac Sim USD scene
- [ ] DiagramAgent: Screenshot or diagram of expected simulation output

**Simulation Requirements**:
- [ ] Works on Ubuntu 22.04 + ROS 2 Humble
- [ ] Documented hardware requirements (GPU optional/required)
- [ ] Reproducible (fixed random seeds where applicable)
- [ ] Troubleshooting section for common issues

**Checklist**:
- [ ] Launch commands tested and verified
- [ ] Simulation demonstrates chapter concepts clearly
- [ ] Includes parameter exploration suggestions
- [ ] Links back to theory and code sections

**Estimated Word Count**: 400 words

---

### 6. Practical Examples (300-500 words)

**Purpose**: End-to-end scenarios integrating multiple concepts

**Content Elements**:
- **Scenario Description**: Real-world problem requiring chapter concepts
- **Multi-Concept Integration**: How different sections connect
- **Step-by-Step Solution**: Applying theory + code + simulation
- **Extensions**: "How would you adapt this for...?"

**Pedagogical Strategy**: Transfer learning, application to novel contexts

**Example Structure**:
1. **Problem Statement**: {describe_realistic_scenario}
2. **Analysis**: Break down problem using chapter concepts
3. **Solution**: Step-by-step application of theory/code
4. **Validation**: How to verify solution correctness
5. **Extensions**: Variations on the problem

**Checklist**:
- [ ] Scenario is realistic and motivating
- [ ] Requires integrating ≥2 chapter concepts
- [ ] Solution references previous sections explicitly
- [ ] Extensions encourage deeper thinking

**Estimated Word Count**: 400 words

---

### 7. Summary (150-250 words)

**Purpose**: Consolidate learning, reinforce key points, provide closure

**Content Elements**:
- **Key Takeaways**: 3-5 bullet points distilling chapter essence
- **Connections**: Links to related chapters (backward and forward references)
- **Further Reading**: Optional advanced topics, research papers, textbooks
- **Transition**: Preview of next chapter

**Pedagogical Strategy**: Retrieval practice, spaced repetition

**Checklist**:
- [ ] Concise (150-250 words)
- [ ] No new information introduced
- [ ] Explicitly connects to previous and next chapters
- [ ] Suggests resources for deeper learning

**Estimated Word Count**: 200 words

---

### 8. Exercises (5-10 problems)

**Purpose**: Active learning, assess understanding, practice application

**Exercise Breakdown by Bloom's Taxonomy**:

**Remember (1-2 exercises)**:
- Exercise 1: Define {key_term} in your own words
- Exercise 2: List the steps to {fundamental_process}

**Understand (1-2 exercises)**:
- Exercise 3: Explain why {phenomenon} occurs
- Exercise 4: Describe the difference between {concept_A} and {concept_B}

**Apply (2-3 exercises)**:
- Exercise 5: Compute {calculation} for given parameters
- Exercise 6: Implement {algorithm} in Python
- Exercise 7: Modify code to {variation}

**Analyze (1-2 exercises)**:
- Exercise 8: Compare {approach_A} vs {approach_B}, identify tradeoffs
- Exercise 9: Debug the following code/equation and explain the error

**Evaluate (0-1 exercise)**:
- Exercise 10: Critique {method} for {application}, propose improvements

**Create (0-1 exercise)**:
- Exercise 11: Design {system} integrating chapter concepts

**Exercise Requirements**:
- [ ] Progressive difficulty (easy → challenging)
- [ ] Clear problem statements
- [ ] Hints provided for challenging problems
- [ ] Solutions available in separate file
- [ ] Rubrics for open-ended problems

---

### 9. Quiz (10-15 questions)

**Purpose**: Summative assessment, measure learning objective achievement

**Question Types**:
- **Multiple Choice** (5-8 questions): Test conceptual understanding and factual knowledge
- **True/False** (3-4 questions): Test common misconceptions
- **Short Answer** (2-3 questions): Test ability to explain concepts

**Quiz Alignment**:
- [ ] Every learning objective covered by ≥2 questions
- [ ] Difficulty distribution: 40% easy, 40% medium, 20% challenging
- [ ] Target pass rate: >70% after reading chapter
- [ ] Immediate feedback with explanations for each answer

**Sample Questions**:

**Q1 (Remember - MCQ)**: What does the Jacobian matrix relate in robotics?
- a) Joint torques to end-effector forces
- b) Joint velocities to end-effector velocities ✓
- c) Joint angles to end-effector pose
- d) Joint accelerations to end-effector accelerations

**Q5 (Apply - Short Answer)**: Given DH parameters θ=30°, d=0, a=1m, α=0°, compute the transformation matrix T.

**Quiz Requirements**:
- [ ] Questions aligned to learning objectives
- [ ] Distractors (wrong answers) based on common misconceptions
- [ ] Explanations for correct and incorrect answers
- [ ] Randomized question order (if platform supports)

---

## Content Integration Plan

**Agent Coordination Sequence**:

1. **RoboticsExpert** (Mathematical Foundations):
   - Request: {specific_derivation}
   - Deliverable: Validated equations with step-by-step explanations
   - Timeline: Before Section 3 writing begins

2. **ROS2Engineer** (Implementation):
   - Request: {specific_code_implementation}
   - Deliverable: Executable ROS 2 code with tests
   - Timeline: Before Section 4 writing begins

3. **SimulationEngineer** or **IsaacExpert** (Simulation):
   - Request: {simulation_environment}
   - Deliverable: Launch files, world files, setup instructions
   - Timeline: Before Section 5 writing begins

4. **DiagramAgent** (Visualizations):
   - Request 1: Conceptual diagram for Section 2
   - Request 2: Mathematical visualization for Section 3
   - Request 3: System architecture for Section 6
   - Timeline: As needed during writing

5. **ValidationAgent** (Quality Gates):
   - Review: Complete chapter draft
   - Checks: Math validation, code testing, consistency
   - Timeline: After full chapter draft complete

---

## Quality Checklist

**Constitution Compliance**:
- [ ] **Principle I** (Technical Accuracy): All math validated, code tested, diagrams accurate
- [ ] **Principle II** (Educational Accessibility): Intuition before formalism, jargon defined, Bloom's taxonomy
- [ ] **Principle III** (Modularity): Self-contained chapter, clear dependencies
- [ ] **Principle IV** (Consistency): Notation consistent, formatting uniform
- [ ] **Principle VI** (Code Standards): PEP 8, ROS 2 style guide, commented
- [ ] **Principle VII** (Quality Gates): ValidationAgent approval required

**Content Quality**:
- [ ] Word count within 1,500-3,000 range (±10%)
- [ ] Reading level: Flesch-Kincaid Grade 12-14
- [ ] All glossary terms defined
- [ ] At least 1 diagram per major section
- [ ] At least 1 code example
- [ ] Exercises span all Bloom's levels
- [ ] Quiz aligned to learning objectives

**Accessibility**:
- [ ] All images have descriptive alt text
- [ ] Semantic HTML (proper heading hierarchy)
- [ ] Code blocks have language tags
- [ ] Math equations use proper LaTeX notation
- [ ] Links have descriptive text (not "click here")

---

## Estimated Timeline

**Outline Creation**: 1-2 hours (this document)
**Content Generation**:
- Section 1 (Overview): 30 min
- Section 2 (Concepts): 1 hour
- Section 3 (Math): 2 hours (includes RoboticsExpert time)
- Section 4 (Code): 2 hours (includes ROS2Engineer time)
- Section 5 (Simulation): 1.5 hours (includes SimulationEngineer time)
- Section 6 (Examples): 1 hour
- Section 7 (Summary): 30 min
- Section 8 (Exercises): 2 hours
- Section 9 (Quiz): 1.5 hours
**Total Estimated**: 12-14 hours
**Validation & Revision**: 2-3 hours
**Grand Total**: 14-17 hours per chapter

---

## Success Criteria

**Outline is complete when**:
- [ ] All sections have clear purpose and content elements defined
- [ ] Learning objectives are SMART and span multiple Bloom's levels
- [ ] Prerequisites are explicitly stated
- [ ] Content requests to all agents are specific and actionable
- [ ] Word count allocations sum to target (±10%)
- [ ] Quality checklist items are measurable
- [ ] Timeline is realistic based on chapter complexity

**Chapter is ready for writing when**:
- [ ] Outline approved by BookPlanner
- [ ] All agent dependencies identified
- [ ] Technical content requirements clear to domain agents
- [ ] EducationDesigner has clear blueprint to follow

---

## Notes

- **Flexibility**: Outline is a blueprint, not a rigid constraint. EducationDesigner may adjust during writing if pedagogical needs require it.
- **Iteration**: Outline may be revised after agent feedback (e.g., RoboticsExpert suggests alternative derivation approach).
- **Collaboration**: Outline facilitates clear communication between EducationDesigner and technical agents.
- **Validation**: Outline ensures all constitution principles and quality standards are planned upfront, reducing revision cycles.

---

**Outline Template Version**: 1.0.0
**Compatible with**: EducationDesigner v1.0.0, Constitution v1.0.0
**Last Updated**: 2025-12-04
