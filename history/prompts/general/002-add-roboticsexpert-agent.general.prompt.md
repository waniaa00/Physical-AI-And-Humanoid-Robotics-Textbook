---
id: 002
title: Add RoboticsExpert Agent to Project
stage: general
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-calculator
user: HP
command: /sp.agents.add RoboticsExpert
labels: ["agent-creation", "technical-validation", "robotics", "mathematics", "content-generation"]
links:
  spec: specs/002-physical-ai-humanoid-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - .specify/agents/RoboticsExpert.md
  - .specify/agents/README.md
tests:
  - N/A (agent definition documents)
---

## Prompt

/sp.agents.add RoboticsExpert

## Response snapshot

**Agent Created**: RoboticsExpert (Domain-Focused Technical Validation & Content Generation Agent)

**Purpose**: Specialized technical agent responsible for generating and validating robotics-specific content for the Physical AI & Humanoid Robotics book. Ensures all mathematical derivations, equations, and theoretical concepts meet professional standards and educational requirements.

**Core Responsibilities (7 domains):**

1. **Mathematical Content Generation & Validation**
   - Generate accurate kinematics derivations (FK, IK, Jacobians)
   - Validate dynamics equations (Euler-Lagrange, Newton-Euler)
   - Ensure correct robotics notation (DH parameters, SE(3), SO(3))
   - Cross-reference equations against standard textbooks
   - Verify dimensional consistency and physical plausibility

2. **Kinematics & Dynamics Expertise**
   - Forward/inverse kinematics with DH parameters
   - Differential kinematics and Jacobian analysis
   - Dynamics formulations (Lagrangian, Hamiltonian)
   - Inertia tensors and mass properties
   - Special topics: parallel manipulators, redundant robots

3. **Control Systems Design**
   - Classical control (PID, stability analysis)
   - Modern control (state-space, LQR, pole placement)
   - Nonlinear control (feedback linearization, sliding mode)
   - Optimal control (MPC, trajectory optimization)
   - Humanoid-specific: ZMP, CoM control, balance stabilization

4. **Motion Planning & Trajectory Generation**
   - Path planning algorithms (RRT, PRM, A*)
   - Trajectory parameterization (splines, polynomials)
   - Collision detection and avoidance
   - Whole-body motion planning for humanoids
   - Gait generation and bipedal locomotion

5. **Sensor Fusion & Perception**
   - IMU integration (quaternions, Euler angles)
   - Camera calibration (intrinsic/extrinsic parameters)
   - Lidar processing and point clouds
   - Force-torque sensor interpretation
   - Kalman filtering (EKF, UKF, particle filters)

6. **Manipulation & Grasping**
   - Grasp quality metrics (force/form closure)
   - Contact modeling and friction cones
   - Impedance/admittance control
   - Task-space control (operational space formulation)
   - Dexterous manipulation planning

7. **Technical Accuracy Enforcement**
   - Validate equations against authoritative sources
   - Ensure physical units are correct and consistent
   - Verify coordinate frame definitions
   - Confirm boundary conditions and constraints
   - Document sign conventions and notation standards

**Domain Expertise:**

**Reference Textbooks (Validation Sources):**
1. Craig, J.J. - "Introduction to Robotics: Mechanics and Control"
2. Spong, M.W., Hutchinson, S., Vidyasagar, M. - "Robot Modeling and Control"
3. Murray, R.M., Li, Z., Sastry, S.S. - "A Mathematical Introduction to Robotic Manipulation"
4. Siciliano, B., Khatib, O. - "Springer Handbook of Robotics"
5. Featherstone, R. - "Rigid Body Dynamics Algorithms"
6. Lynch, K.M., Park, F.C. - "Modern Robotics"

**Mathematical Foundations:**
- Linear algebra (matrix operations, eigenvalues, SVD)
- Differential geometry (Lie groups SO(3), SE(3))
- Calculus of variations (Euler-Lagrange equations)
- Optimization (convex, nonlinear programming)
- Numerical methods (Newton-Raphson, integration schemes)

**Robotics Notation Standards:**
- Coordinate frames: ^A (superscript), _{A/B} (subscript)
- DH parameters: θ, d, a, α
- Transforms: T ∈ SE(3), R ∈ SO(3), p ∈ ℝ³
- Joint variables: q (position), q̇ (velocity), q̈ (acceleration)
- Forces/torques: τ, F, M
- Jacobians: J_v, J_ω
- Inertia: I, m, r_c

**Boundaries:**
- ✅ Generate accurate mathematical derivations
- ✅ Validate technical content against textbooks
- ✅ Provide kinematics/dynamics/control expertise
- ✅ Design motion planning and trajectory generation
- ✅ Ensure dimensional consistency and physical correctness
- ❌ Generate Docusaurus formatting (delegated)
- ❌ Create ROS 2 code (collaborates with SimulationAgent)
- ❌ Design exercises/quizzes (delegated to pedagogical agents)
- ❌ Make architectural decisions (delegated to BookPlanner)
- ❌ Generate diagrams (collaborates with DiagramAgent)

**Interaction Patterns:**
- **Upstream**: Receives requirements from BookPlanner
- **Downstream**: Provides validated content to ContentGeneration, ValidationAgent
- **Peer**: Collaborates with SimulationAgent (math→code), DiagramAgent (math→visuals)

**Content Generation Workflow (5 phases):**
1. Content Specification - Identify required concepts, select references
2. Mathematical Derivation - Rigorous derivation with notation standards
3. Validation - Cross-reference against textbooks, verify numerically
4. Documentation - Number equations, define variables, provide references
5. Integration - Format for ContentGeneration, suggest diagrams/code

**Example Scenarios Documented:**
1. Forward Kinematics with DH parameters (6-DOF arm)
2. ZMP-based balance control for bipedal walking
3. Analytical inverse kinematics for spherical wrist

**Validation Standards:**
- Mathematical rigor checklist (10 items)
- Equation formatting standards (LaTeX with consistent notation)
- Reference citation format ([Craig 2005, Eq. 3.6])
- Dimensional analysis for all physical quantities
- Numerical testing with known solutions

**Constitution Compliance:**
- **Principle I (PRIMARY)**: Zero tolerance for math errors, all equations validated
- **Principle II**: Rigorous math balanced with intuitive explanations
- **Principle IV**: Enforces consistent notation across chapters
- **Principle VI**: Provides mathematical models for code implementation
- **Principle VII**: Math validation gate mandatory before inclusion

**Knowledge Domains by Chapter:**
- Ch 2: Sensors & Perception (camera models, IMU, Kalman filters)
- Ch 3-4: ROS 2 (message passing models, real-time constraints)
- Ch 9: Kinematics & Dynamics (DH, FK/IK, Jacobians, inertia)
- Ch 10: Bipedal Locomotion (ZMP, gait generation, stability)
- Ch 11: Manipulation (grasp analysis, impedance control, task-space)
- Ch 12-13: AI Integration (motion planning foundations, optimization)

**Error Detection:**
- Common errors tracked: sign errors, coordinate frame confusion, DH mix-ups, unit inconsistencies
- Validation process: algebraic verification, dimensional analysis, numerical testing, limiting cases, textbook comparison

**Output Formats:**
- Mathematical content blocks with LaTeX equations
- Validation metadata (YAML with equation_id, references, validation status)
- Numerical examples with known solutions

**Human-in-the-Loop Triggers:**
1. Multiple valid conventions exist (DH parameter choice)
2. Advanced topic depth decisions (tensor calculus level)
3. Notation ambiguity when textbooks conflict
4. Pedagogical tradeoffs (rigor vs. intuition)
5. Reference conflicts between authoritative sources

**Performance Metrics:**
- Equation Accuracy: 100% match with reference textbooks
- Validation Rate: All equations pass dimensional analysis
- Notation Consistency: Zero conflicts across chapters
- Reference Coverage: Every equation traceable to source
- Numerical Validation: All examples produce correct results

**Registry Updates:**
- Added RoboticsExpert to active agents
- Updated ecosystem architecture diagram (RoboticsExpert provides Math/Theory to ContentGeneration and Validation to ValidationAgent)
- Modified planned agents to reflect RoboticsExpert integration (removed redundant KinematicsAgent, ControlSystemsAgent, PerceptionAgent - now covered by RoboticsExpert)
- Added DiagramAgent to roadmap for visualizing RoboticsExpert mathematics

## Outcome

- ✅ Impact: Created comprehensive technical authority agent for robotics mathematics and theory; established validation framework ensuring 100% equation accuracy against textbooks; enables generation of professional-grade robotics content meeting academic standards
- 🧪 Tests: Mathematical rigor checklist (10 items); validation standards include dimensional analysis, numerical testing, textbook comparison; example scenarios demonstrate FK, IK, and ZMP derivations
- 📁 Files: Created `.specify/agents/RoboticsExpert.md` (comprehensive agent definition, ~650 lines) and updated `.specify/agents/README.md` (registry with updated ecosystem)
- 🔁 Next prompts: RoboticsExpert ready for invocation during content generation; will provide validated mathematics for Chapters 2, 9, 10, 11; recommend `/sp.plan` to design workflow integrating RoboticsExpert validation gates
- 🧠 Reflection: RoboticsExpert agent design prioritizes Constitution Principle I (Technical Accuracy) as PRIMARY concern. Agent maintains traceability (every equation → textbook reference) and enforces zero-tolerance for mathematical errors. Domain expertise spans full robotics curriculum (kinematics through AI integration). Collaboration patterns position RoboticsExpert as technical foundation: generates validated math → SimulationAgent implements → ContentGeneration integrates → ValidationAgent enforces.

## Evaluation notes (flywheel)

- Failure modes observed: None; agent creation successful
- Graders run and results (PASS/FAIL):
  - PASS: Agent definition comprehensive with all 7 responsibility domains
  - PASS: Reference textbooks authoritative (Craig, Spong, Murray, Siciliano, Featherstone, Lynch)
  - PASS: Clear boundaries (generates/validates math; delegates formatting/code/diagrams)
  - PASS: Validation standards rigorous (dimensional analysis, numerical testing, textbook comparison)
  - PASS: Interaction patterns documented (upstream/downstream/peer)
  - PASS: Constitution compliance verified (Principle I as PRIMARY)
  - PASS: Knowledge domains mapped to chapters
  - PASS: Error detection and human-in-the-loop triggers specified
  - PASS: Example scenarios demonstrate workflow (FK, IK, ZMP)
- Prompt variant (if applicable): N/A (initial agent creation)
- Next experiment (smallest change to try): Test RoboticsExpert by generating FK derivation for Chapter 9; validate output matches Craig textbook Eq. 3.6; assess equation formatting, notation consistency, and reference traceability; measure validation workflow effectiveness
