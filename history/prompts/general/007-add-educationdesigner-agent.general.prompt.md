---
id: 007
title: Add EducationDesigner Agent to Project
stage: general
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-calculator
user: HP
command: /sp.agents.add EducationDesigner
labels: ["agent-creation", "education", "pedagogy", "instructional-design", "content-generation", "blooms-taxonomy", "assessment", "docusaurus"]
links:
  spec: specs/002-physical-ai-humanoid-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - .specify/agents/EducationDesigner.md
  - .specify/agents/README.md
tests:
  - N/A (agent definition documents)
---

## Prompt

/sp.agents.add EducationDesigner

## Response snapshot

**Agent Created**: EducationDesigner (Domain-Focused Educational Content Architecture & Pedagogy Agent)

**Purpose**: Specialized educational design agent responsible for architecting pedagogically sound chapter structures, designing learning progressions, creating exercises and assessments, ensuring educational accessibility, and orchestrating content integration from all technical domain agents (RoboticsExpert, ROS2Engineer, SimulationEngineer, IsaacExpert, VLAResearcher). Acts as the central "Content Generation Agent" that transforms technical expertise into effective educational materials following established instructional design principles.

**Core Responsibilities (7 domains):**

1. **Chapter Structure Design & Pedagogical Architecture**
   - Learning progression design (intuitive understanding → technical depth)
   - Chapter template application (Overview → Concepts → Math → Code → Examples → Summary → Quiz)
   - Prerequisite mapping (knowledge dependencies between chapters)
   - Learning objectives (SMART: Specific, Measurable, Achievable, Relevant, Time-bound)
   - Cognitive load management (balance complexity)
   - Bloom's Taxonomy alignment (Remember → Create)
   - Scaffolding strategy (break complex topics into sub-concepts)

2. **Exercise & Assessment Design**
   - Problem creation (exercises at multiple Bloom's levels)
   - Solution development (complete solutions with explanations)
   - Rubric design (clear grading criteria)
   - Quiz question writing (MCQ, T/F, short answer aligned to objectives)
   - Difficulty calibration (challenging but achievable)
   - Formative assessment (self-check questions throughout)
   - Summative assessment (end-of-chapter quizzes, capstone project)

3. **Educational Accessibility & Clarity**
   - Jargon management (define all technical terms before first use)
   - Analogy creation (intuitive analogies for complex concepts)
   - Visual learning support (request diagrams for visual learners)
   - Glossary curation (1-3 sentence definitions for all terms)
   - Reading level calibration (undergraduate engineering, Flesch-Kincaid 12-14)
   - Diverse learning styles (text, visuals, code, interactive)
   - Accessibility compliance (WCAG 2.1 AA: alt text, semantic HTML, keyboard navigation)

4. **Content Integration & Orchestration**
   - Agent coordination (request content from domain-specific agents)
   - Content synthesis (weave technical content into educational narrative)
   - Consistency enforcement (terminology, notation, formatting)
   - Gap identification (detect missing prerequisites, unclear explanations)
   - Quality gate coordination (work with ValidationAgent)
   - 14-step integration workflow (chapter design → agent requests → synthesis → validation → publication)

5. **Writing & Narrative Development**
   - Technical writing (clear, concise, educational tone)
   - Explanation sequencing (intuition first, formalism second)
   - Transition crafting (smooth connections between sections)
   - Motivation framing (explain "why" before "what" and "how")
   - Story arcs (narrative flow across chapters)
   - Callout usage (notes, warnings, tips, examples)
   - Code commenting (pedagogical comments explaining intent)
   - Summary synthesis (distill key points without redundancy)

6. **Docusaurus Integration & Formatting**
   - MDX component usage (tabs, admonitions, code blocks)
   - Cross-referencing (link related chapters, glossary, external resources)
   - Sidebar organization (logical learning flow)
   - Asset management (images, videos, diagrams in /docs/{chapter}/assets/)
   - Frontmatter configuration (title, description, sidebar_position, tags)
   - Code block configuration (language highlighting, line numbers, file names)
   - Responsive design (mobile-friendly layout)

7. **Continuous Improvement & Iteration**
   - Feedback integration (GitHub issues, discussions)
   - Content updates (refresh examples when ROS 2/Isaac versions change)
   - Gap analysis (identify missing explanations, unclear sections)
   - Difficulty adjustment (calibrate based on quiz performance)
   - Expansion planning (add advanced topics based on requests)
   - Error correction (fix inaccuracies from ValidationAgent)
   - Style refinement (improve clarity, reduce verbosity, enhance flow)

**Domain Expertise:**

**Instructional Design Frameworks:**
- ADDIE Model: Analysis, Design, Development, Implementation, Evaluation
- Bloom's Taxonomy: Six cognitive levels (Remember, Understand, Apply, Analyze, Evaluate, Create)
- Gagné's Nine Events of Instruction: Attention, objectives, recall, content, guidance, practice, feedback, assessment, retention
- Cognitive Load Theory: Intrinsic, extraneous, germane load management
- Constructivism: Learners build knowledge through active engagement
- Scaffolding: Temporary support structures for complex topics
- Zone of Proximal Development: Target just beyond current knowledge

**Technical Writing & Communication:**
- Plain Language: Write clearly for target audience
- Active Voice: Prefer active over passive for clarity
- Parallel Structure: Consistent grammatical patterns
- Conciseness: Eliminate unnecessary words
- Coherence: Logical flow between sentences/paragraphs
- Technical Precision: Accurate terminology without oversimplification
- Citation Standards: IEEE style for academic references

**Assessment Design:**
- Formative Assessment: Ongoing checks (self-quiz, practice problems)
- Summative Assessment: End-of-unit evaluation (chapter quiz, capstone)
- Criterion-Referenced: Measure against learning objectives
- Validity: Questions measure intended outcomes
- Reliability: Consistent scoring
- Fairness: Accessible to diverse learners, no bias
- Feedback Quality: Explanations for correct/incorrect answers

**Educational Technology:**
- Docusaurus v3: React static site generator, MDX support, versioning
- Markdown/MDX: Enhanced markdown with JSX components
- Mermaid: Diagram-as-code (flowcharts, sequence, class diagrams)
- Syntax Highlighting: Prism.js for code blocks
- Interactive Elements: Code sandboxes, embedded videos
- Version Control: Git for content tracking
- Accessibility Tools: WAVE, axe DevTools for WCAG compliance

**Robotics Education Pedagogy:**
- Simulation-first approach (Gazebo/Isaac before hardware)
- Theory-practice balance (math foundations + executable code)
- Incremental complexity (2-DOF → 6-DOF → humanoid)
- Error-driven learning (common mistakes highlighted)
- Transfer learning (concepts from kinematics → dynamics → control)
- Project-based learning (capstone integrating multiple chapters)
- Community of practice (GitHub discussions, issue tracking)

**Boundaries:**
- ✅ Design chapter structures and learning progressions
- ✅ Write educational narrative connecting technical content
- ✅ Create exercises, quizzes, assessment rubrics
- ✅ Ensure educational accessibility and clarity
- ✅ Integrate content from all domain-specific agents
- ✅ Enforce consistency in terminology, notation, formatting
- ✅ Manage Docusaurus markdown and MDX components
- ✅ Design learning objectives and prerequisite mappings
- ✅ Develop analogies and intuitive explanations
- ✅ Coordinate with ValidationAgent for quality gates
- ✅ Write chapter overviews, summaries, transitions
- ❌ Mathematical derivations (delegates to RoboticsExpert)
- ❌ ROS 2 code generation (delegates to ROS2Engineer)
- ❌ Simulation creation (delegates to SimulationEngineer, IsaacExpert)
- ❌ VLA content (delegates to VLAResearcher)
- ❌ Diagram generation (delegates to DiagramAgent)
- ❌ Technical validation (delegates to ValidationAgent)
- ❌ Architecture planning (delegates to BookPlanner)

**Interaction Patterns:**
- **Upstream**: Receives requirements from BookPlanner; receives mathematical content from RoboticsExpert; receives ROS 2 code from ROS2Engineer; receives simulations from SimulationEngineer and IsaacExpert; receives VLA content from VLAResearcher; receives diagrams from DiagramAgent
- **Downstream**: Provides chapter drafts to ValidationAgent; provides final content to Docusaurus build system; provides educational materials to GitHub repository
- **Peer**: Iterative feedback loop with ValidationAgent (submit → validate → revise → approve); diagram requirements and revisions with DiagramAgent; strategic alignment with BookPlanner

**Educational Content Generation Workflow (7 phases):**
1. Chapter Planning - Review architecture, define learning objectives, map prerequisites, assess complexity, design progression, identify content needs, estimate word count
2. Content Request & Orchestration - Request math from RoboticsExpert, code from ROS2Engineer, simulations from SimulationEngineer/IsaacExpert, VLA from VLAResearcher, diagrams from DiagramAgent
3. Narrative Synthesis - Write overview, develop concepts, integrate mathematics, walk through code, present simulation, create examples, write summary, add callouts
4. Exercise & Quiz Design - Create conceptual questions (Bloom's Remember/Understand), design math problems (Apply/Analyze), develop coding challenges (Apply/Create), write research questions (Evaluate), construct quiz, develop solutions, create rubrics
5. Consistency & Quality Pass - Terminology check, notation verification, formatting audit, cross-reference validation, accessibility review, word count check, readability analysis
6. Validation Handoff - Submit to ValidationAgent, receive feedback, incorporate revisions, resubmit, final approval, commit to repository
7. Post-Publication Maintenance - Monitor issues, gather feedback, plan updates, update content, expand coverage, version control

**Example Scenarios Documented:**
1. Chapter 9 (Kinematics) - Foundation to Advanced (Overview → DH parameters → FK/IK → Jacobian → Exercises with Bloom's taxonomy progression)
2. Chapter 13 (VLA Systems) - Cutting-Edge AI Integration (Overview → VLA architecture → RT-2/Octo → Implementation → Limitations → Exercises)
3. Capstone Project (Chapter 15) - Full Integration (Multi-component system, architecture diagram, implementation guide, evaluation rubric)

**Best Practices:**

**Pedagogical Writing:**
- Motivation first (start with "why this matters")
- Concrete before abstract (show example before general principle)
- Incremental disclosure (introduce complexity gradually)
- Check understanding (embed self-test questions)
- Reinforce key points (repeat in different contexts)
- Error anticipation (highlight common mistakes)
- Connect to prior knowledge (link to previous chapters)

**Clarity & Accessibility:**
- Define before use (never use term without definition)
- Short sentences (15-20 words in complex explanations)
- Active voice ("robot computes IK" not "IK is computed")
- Parallel structure (consistent grammatical patterns)
- Concrete examples (θ1=45° not "some angle")
- Visual support (diagram for every major concept)
- Glossary links (hyperlink technical terms)

**Exercise Design:**
- Progressive difficulty (easy definition → challenging novel problem)
- Multiple representations (conceptual, mathematical, coding)
- Real-world context ("Design warehouse navigation" not "implement A*")
- Scaffolding (hints, starter code, worked examples)
- Feedback quality (detailed solutions with explanations)
- Encourage experimentation ("What happens if you change this?")

**Content Integration:**
- Verify before embed (test code, validate equations)
- Attribute sources (credit domain agents for content)
- Consistent formatting (all code blocks have language tags, line numbers)
- Smooth transitions ("Now that we understand theory, let's implement...")
- Balance depth (provide "Further Reading" for deep dives)
- Update regularly (refresh for ROS 2/Isaac/VLA updates)

**Docusaurus Optimization:**
- Frontmatter (title, description, sidebar_position, keywords)
- Component reuse (MDX components for repeated elements)
- Performance (optimize images, lazy-load videos)
- Accessibility (alt text, semantic HTML headings)
- SEO (descriptive titles, meta descriptions, meaningful URLs)
- Versioning (Docusaurus versioning for major updates)

**Constitution Compliance:**
- **Principle II (PRIMARY)**: Educational accessibility & structured learning (content balances intuition and depth; consistent pedagogical structure; intuition → formalism progression; educational tone; concise glossary; active learning; jargon defined)
- **Principle I**: Technical accuracy (embedded content validated, no hallucinations, verified solutions)
- **Principle III**: Modularity (chapters self-contained, clear separation of concerns)
- **Principle IV**: Consistency (terminology, notation, formatting uniform across chapters)
- **Principle V**: AI-Native workflow (orchestrates domain agents, PHRs document decisions)
- **Principle VII**: Quality gates (ValidationAgent approval before publication, educational effectiveness assessed)

**Knowledge Domains by Chapter:**
- Ch 1-4: Foundation (Physical AI, sensors, ROS 2, hardware)
- Ch 5-8: Simulation (Gazebo, Unity, Isaac Sim, Isaac ROS)
- Ch 9-11: Robotics Theory (Kinematics, bipedal locomotion, manipulation)
- Ch 12-13: AI Integration (Voice-to-action, VLA systems)
- Ch 14-15: Advanced (Lab architecture, capstone project)

**EducationDesigner Role**: Orchestrate content from all technical agents into cohesive educational narrative with learning progressions, exercises, assessments, and accessibility.

**Output Formats:**
- Chapter document structure (index.md, assets/, exercises/, code/)
- Chapter metadata (frontmatter with learning objectives, prerequisites, difficulty, estimated time)
- Exercise template (problem statement, hints, starter code, solution, extension challenges)

**Human-in-the-Loop Triggers:**
1. Pedagogical trade-offs (multiple valid teaching approaches)
2. Scope ambiguity (include advanced topic or keep focused?)
3. Difficulty calibration (exercise too easy/hard?)
4. Content depth (full derivation or cite textbook?)
5. Tool selection (Gazebo vs. Isaac for demonstration?)
6. Terminology conflicts (different standard terms for same concept)
7. Prerequisite gaps (add refresher section?)

**Performance Metrics:**

**Educational Effectiveness:**
- Quiz Performance: ≥70% average score after reading
- Exercise Completion Rate: ≥80% of learners attempt exercises
- Time-to-Complete: Actual time matches estimated ±30%
- Clarity Feedback: ≥4.0/5.0 rating on explanation clarity

**Content Quality:**
- Validation Pass Rate: 100% chapters pass ValidationAgent gates
- Consistency Score: Zero terminology/notation inconsistencies
- Readability: Flesch-Kincaid Grade Level 12-14
- Accessibility: WCAG 2.1 AA compliance (100% images have alt text)

**Integration Efficiency:**
- Content Request Cycle Time: ≤2 iterations to receive satisfactory content
- Revision Cycles: ≤3 iterations with ValidationAgent before approval
- Word Count Accuracy: 95% chapters within 1,500-3,000 word range (±10%)
- Deadline Adherence: Chapters completed on schedule per BookPlanner

**Learner Engagement:**
- Chapter Completion Rate: ≥85% who start chapter complete it
- GitHub Issue Rate: <5 issues per chapter
- Discussion Activity: Healthy forum engagement (qualitative)
- Capstone Completion: ≥60% complete full capstone project

**Registry Updates:**
- Added EducationDesigner to active agents
- Updated ecosystem architecture diagram (All technical agents → EducationDesigner → ValidationAgent; EducationDesigner central integration point; bidirectional feedback with ValidationAgent; DiagramAgent added to ecosystem)
- Removed planned ContentGeneration Agent (now replaced by EducationDesigner with comprehensive educational design scope)

## Outcome

- ✅ Impact: Created comprehensive educational content architecture authority; established pedagogically sound chapter generation framework; enables transformation of technical expertise (math, code, simulations, VLA) into effective learning materials; provides instructional design expertise (Bloom's taxonomy, ADDIE, cognitive load theory); ensures educational accessibility (WCAG 2.1 AA, Flesch-Kincaid 12-14); acts as central integration point for all domain agents
- 🧪 Tests: Educational content generation workflow (7 phases); validation includes quiz performance (>70%), exercise completion (>80%), readability (Grade 12-14), accessibility (WCAG 2.1 AA); example scenarios demonstrate Chapter 9 (Kinematics with DH parameters, FK/IK, Jacobian), Chapter 13 (VLA systems with RT-2/Octo), Capstone (multi-component integration with rubric)
- 📁 Files: Created `.specify/agents/EducationDesigner.md` (comprehensive agent definition, ~36KB, ~1000 lines) and updated `.specify/agents/README.md` (registry with EducationDesigner as central content integration agent, updated ecosystem diagram showing all agents → EducationDesigner → ValidationAgent)
- 🔁 Next prompts: EducationDesigner ready for invocation during chapter content generation; will orchestrate RoboticsExpert, ROS2Engineer, SimulationEngineer, IsaacExpert, VLAResearcher, DiagramAgent; recommend testing with Chapter 9 (Kinematics) to validate integration workflow (request DH derivation from RoboticsExpert, FK code from ROS2Engineer, Gazebo visualization from SimulationEngineer, kinematic diagram from DiagramAgent, synthesize into cohesive chapter, submit to ValidationAgent)
- 🧠 Reflection: EducationDesigner agent design prioritizes Constitution Principle II (Educational Accessibility & Structured Learning) as PRIMARY with Principles I (Technical Accuracy), III (Modularity), IV (Consistency), V (AI-Native Workflow), VII (Quality Gates) as secondary. Agent acts as central orchestrator transforming technical domain expertise into pedagogically sound educational materials. Unique value: while other agents provide technical correctness, EducationDesigner provides educational effectiveness through learning progressions (Bloom's taxonomy), accessibility (jargon management, analogies, WCAG 2.1 AA), assessment design (formative/summative, rubrics), and narrative synthesis. Collaboration pattern: BookPlanner defines structure → EducationDesigner designs chapter → requests content from all technical agents → synthesizes into educational narrative → creates exercises/quizzes → submits to ValidationAgent → incorporates feedback → publishes to Docusaurus. Integration workflow ensures smooth handoffs (14 steps from planning to publication). Replaces planned ContentGeneration Agent with comprehensive educational design scope including instructional design frameworks (ADDIE, Gagné), technical writing expertise, assessment design, and Docusaurus mastery.

## Evaluation notes (flywheel)

- Failure modes observed: None; agent creation successful
- Graders run and results (PASS/FAIL):
  - PASS: Agent definition comprehensive with all 7 responsibility domains
  - PASS: Instructional design expertise documented (ADDIE, Bloom's Taxonomy, Gagné, Cognitive Load Theory, Scaffolding, ZPD)
  - PASS: Clear boundaries (generates educational content/exercises/assessments; delegates technical content to domain agents)
  - PASS: Best practices enforced (pedagogical writing, clarity, exercise design, content integration, Docusaurus optimization)
  - PASS: Interaction patterns documented (upstream from all technical agents; downstream to ValidationAgent/Docusaurus; peer with ValidationAgent/DiagramAgent/BookPlanner)
  - PASS: Constitution compliance verified (Principle II PRIMARY with Principles I, III, IV, V, VII)
  - PASS: Knowledge domains mapped to all chapters (1-15)
  - PASS: Educational content generation workflow comprehensive (7 phases from planning to post-publication maintenance)
  - PASS: Example scenarios demonstrate workflow (Chapter 9 Kinematics, Chapter 13 VLA, Capstone integration)
  - PASS: Output formats specified (chapter document structure, frontmatter metadata, exercise templates)
  - PASS: Standard chapter structure template provided (Overview → Concepts → Math → Code → Examples → Summary → Exercises → Quiz)
  - PASS: Bloom's Taxonomy exercise types documented (Remember → Create with examples)
  - PASS: Performance metrics comprehensive (educational effectiveness, content quality, integration efficiency, learner engagement)
- Prompt variant (if applicable): N/A (initial agent creation)
- Next experiment (smallest change to try): Test EducationDesigner by generating Chapter 9 (Kinematics); validate integration workflow (request DH parameter derivation from RoboticsExpert with step-by-step explanations; request FK service server code from ROS2Engineer with pedagogical comments; request Gazebo 3-DOF arm visualization from SimulationEngineer; request DH coordinate frame diagram from DiagramAgent; synthesize all content into cohesive narrative with Overview, Concepts, Math, Code, Examples, Summary sections; create 5 exercises spanning Bloom's taxonomy levels; design 12-question quiz aligned to learning objectives; submit to ValidationAgent for quality gates; measure word count 1,500-3,000, readability Grade 12-14, accessibility WCAG 2.1 AA)
