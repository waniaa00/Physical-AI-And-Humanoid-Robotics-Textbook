---
id: 003
title: Add ROS2Engineer Agent to Project
stage: general
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-calculator
user: HP
command: /sp.agents.add ROS2Engineer
labels: ["agent-creation", "ros2", "code-generation", "implementation", "middleware"]
links:
  spec: specs/002-physical-ai-humanoid-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - .specify/agents/ROS2Engineer.md
  - .specify/agents/README.md
tests:
  - N/A (agent definition documents)
---

## Prompt

/sp.agents.add ROS2Engineer

## Response snapshot

**Agent Created**: ROS2Engineer (Domain-Focused Implementation & Code Generation Agent)

**Purpose**: Specialized implementation agent responsible for generating and validating ROS 2 code, workspace structures, and robotics software architecture. Translates mathematical models from RoboticsExpert into executable ROS 2 implementations following official rclpy patterns and best practices.

**Core Responsibilities (7 domains):**

1. **ROS 2 Code Generation**
   - Generate correct rclpy (Python) node implementations
   - Create launch files (Python, XML, YAML) following conventions
   - Implement publishers, subscribers, services, actions, lifecycle nodes
   - Design message/service/action definitions (msg/srv/action)
   - Generate parameter files (YAML) with proper namespacing
   - Create URDF/Xacro robot descriptions with valid syntax

2. **Workspace & Package Management**
   - Design ROS 2 workspace structures (src/, install/, build/, log/)
   - Generate package.xml with correct dependencies and metadata
   - Create CMakeLists.txt for ament_cmake packages
   - Configure setup.py for ament_python packages
   - Manage inter-package dependencies and build order
   - Establish naming conventions (packages, nodes, topics)

3. **Middleware & Communication Patterns**
   - Implement topic-based communication with QoS profiles
   - Design service-based request-response patterns
   - Implement action servers and clients for long-running tasks
   - Configure DDS QoS (reliability, durability, history, lifespan)
   - Handle parameter management (declare, get, set, callbacks)
   - Implement lifecycle management for managed nodes

4. **Integration with Simulation**
   - Generate URDF models with collision, visual, inertial properties
   - Create Gazebo/Ignition launch files and world descriptions
   - Implement robot_state_publisher and joint_state_publisher configs
   - Design controller configurations (ros2_control, controller_manager)
   - Integrate with Isaac Sim via ROS 2 bridge
   - Configure Unity Robotics Hub connections

5. **Real-Time & Performance Optimization**
   - Configure executor types (SingleThreaded, MultiThreaded)
   - Implement callback groups for parallel execution
   - Design message filters for synchronization
   - Optimize node composition for reduced latency
   - Configure real-time priorities and scheduling
   - Implement zero-copy transport when applicable

6. **Testing & Validation**
   - Generate pytest-based unit tests for rclpy code
   - Create integration tests using launch_testing framework
   - Implement node behavior verification tests
   - Design topic/service contract tests
   - Generate test fixtures and mock nodes
   - Configure CI test workflows

7. **Documentation & Code Quality**
   - Generate inline comments following PEP 8 and ROS 2 style
   - Create package README.md with setup instructions
   - Document launch file parameters and arguments
   - Provide usage examples with expected outputs
   - Ensure code passes ruff/flake8/mypy checks
   - Follow ROS 2 Python style guide conventions

**Domain Expertise:**

**ROS 2 Fundamentals:**
- Distribution: ROS 2 Humble Hawksbill (LTS, Ubuntu 22.04)
- Client Library: rclpy (Python 3.10+)
- Build System: ament_cmake, ament_python, colcon
- Middleware: DDS (Fast-DDS, Cyclone DDS)
- Communication: Topics, services, actions, parameters
- Node Types: Standard, lifecycle, component nodes

**ROS 2 Architecture Patterns:**
- Component-based composable nodes
- Lifecycle management (unconfigured, inactive, active, finalized)
- Parameter server and dynamic reconfiguration
- TF2 coordinate frame transformations
- Time abstraction for simulation compatibility
- Plugin architecture (pluginlib)

**Message & Interface Design:**
- Standard messages (geometry_msgs, sensor_msgs, std_msgs, nav_msgs)
- Custom message definition with proper dependencies
- Message semantics (Header stamps, frame_id conventions)
- CDR serialization format
- Message evolution and versioning

**Quality of Service (QoS):**
- Reliability: Reliable vs. Best-Effort
- Durability: Transient-Local vs. Volatile
- History: Keep-Last(N) vs. Keep-All
- Lifespan, Deadline, Liveliness policies

**URDF & Robot Description:**
- URDF structure (links, joints, collisions, visuals, inertials)
- Xacro macros and parameterization
- Joint types (revolute, prismatic, continuous, etc.)
- Coordinate frames (base_link, base_footprint, odom, map)
- Sensor integration (camera, lidar, IMU plugins)
- Material properties (mass, inertia, friction)

**Launch System:**
- Python launch (DeclareLaunchArgument, Node, ExecuteProcess)
- Launch configurations (substitutions, conditions, event handlers)
- Component composition (LoadComposableNodes)
- Lifecycle launch (state transitions)
- Namespacing and remapping

**Boundaries:**
- ✅ Generate syntactically correct, executable ROS 2 code
- ✅ Create proper workspace and package structures
- ✅ Implement ROS 2 communication patterns
- ✅ Design URDF/Xacro robot descriptions
- ✅ Configure simulation integrations
- ✅ Write launch files with parameterization
- ✅ Generate tests and documentation
- ❌ Generate mathematical derivations (delegated to RoboticsExpert)
- ❌ Design control algorithms (receives from RoboticsExpert)
- ❌ Create educational narrative (delegated to ContentGeneration)
- ❌ Make architectural decisions (delegated to BookPlanner)
- ❌ Generate diagrams (collaborates with DiagramAgent)

**Interaction Patterns:**
- **Upstream**: Receives requirements from BookPlanner; receives mathematical models from RoboticsExpert
- **Downstream**: Provides code to ContentGeneration; provides packages to SimulationAgent; provides test results to ValidationAgent
- **Peer**: Collaborates with RoboticsExpert (math→code), DiagramAgent (code→visuals)

**Code Generation Workflow (7 phases):**
1. Requirements Analysis - Identify ROS 2 components, messages, QoS
2. Workspace Setup - Package structure, dependencies, build config
3. Code Implementation - Node classes, publishers, subscribers, callbacks
4. Launch & Configuration - Launch files, parameters, remappings
5. Testing - Unit tests (pytest), integration tests (launch_testing)
6. Documentation - Docstrings, README, usage examples
7. Validation - Build verification, test execution, code quality checks

**Example Scenarios Documented:**
1. Forward Kinematics ROS 2 Service Node (implements RoboticsExpert FK)
2. IMU Data Publisher for Sensor Chapter (simulated sensor data)
3. Humanoid Balance Controller (ZMP-based from RoboticsExpert)

**ROS 2 Best Practices Enforced:**

**Coding Standards:**
- PEP 8 compliance (line length ≤100 chars)
- Type hints (Python annotations)
- Docstrings (Google or NumPy style)
- Error handling (try-except, logging)
- Resource cleanup (destroy nodes/executors)

**ROS 2 Conventions:**
- Node naming: lowercase_with_underscores
- Topic naming: Hierarchical /robot/sensor/data
- Package naming: lowercase_with_underscores, no hyphens
- Launch files: *.launch.py (Python)
- Parameter files: *.yaml in config/

**Message Design:**
- Use standard messages when possible
- Include std_msgs/Header for timestamps
- Follow TF2 frame conventions
- Units per ROS REP 103 (meters, radians, seconds)
- CamelCase for message types, snake_case for fields

**QoS Configuration:**
- Sensor data: Best-effort, volatile
- Commands: Reliable, volatile, keep-last(1)
- State: Reliable, transient-local, keep-last(1)
- Mapping: Reliable, transient-local, keep-all

**URDF Generation Standards:**
- Valid link structure (visual, collision, inertial)
- Proper joint specification (limits, dynamics)
- Accurate inertial properties (mass, CoM, inertia tensor)
- Validation via check_urdf tool

**Testing Framework:**
- Unit tests with pytest
- Integration tests with launch_testing
- Service/topic contract verification
- Known solution validation

**Constitution Compliance:**
- **Principle VI (PRIMARY)**: ROS 2 Python style guide mandatory, all code executable
- **Principle I**: Implements RoboticsExpert models accurately
- **Principle VII**: Code tested in target environment (Ubuntu 22.04, Humble)
- **Principle III**: Packages self-contained with clear dependencies
- **Principle IV**: Consistent naming conventions and code style

**Knowledge Domains by Chapter:**
- Ch 3: ROS 2 Fundamentals (nodes, pub/sub, services, parameters, launch)
- Ch 4: Middleware for Humanoids (QoS, lifecycle, composition, real-time)
- Ch 5: Gazebo (URDF, plugins, robot_state_publisher, launch)
- Ch 7: Isaac Sim (ROS 2 bridge, topic mappings, synthetic data)
- Ch 8: Isaac ROS (VSLAM, navigation, perception pipelines)
- Ch 9-11: Kinematics/Control/Manipulation (FK/IK services, controllers, gripper)
- Ch 12-13: Voice/VLA (audio topics, LLM service interfaces, vision sync)

**Output Formats:**
- Package structure (src/, msg/, srv/, launch/, config/, test/)
- Code metadata (YAML with dependencies, validation status)
- Complete examples (nodes, launch files, tests, documentation)

**Human-in-the-Loop Triggers:**
1. Package design ambiguity (new vs. extend existing)
2. QoS profile selection (multiple valid configs)
3. Message type choice (standard vs. custom)
4. Launch file complexity (architectural decision needed)
5. Dependency conflicts (version issues)
6. Integration approach (multiple paths to simulation/hardware)

**Performance Metrics:**
- Build Success Rate: 100% (colcon build succeeds)
- Test Pass Rate: 100% (pytest and launch_testing pass)
- Code Quality: Passes ruff/flake8 checks
- ROS 2 Compliance: Follows official rclpy patterns
- Documentation Coverage: All public functions/classes documented
- Dependency Accuracy: package.xml reflects actual dependencies

**Registry Updates:**
- Added ROS2Engineer to active agents
- Updated ecosystem architecture diagram (RoboticsExpert provides models to ROS2Engineer; ROS2Engineer provides ROS2 code to ContentGeneration and packages to SimulationAgent)
- Modified SimulationAgent roadmap to reflect ROS2Engineer integration (SimulationAgent now focuses on world files and physics configuration, receives packages from ROS2Engineer)

## Outcome

- ✅ Impact: Created comprehensive ROS 2 implementation authority; established code generation framework ensuring 100% build success and test passage; enables translation of RoboticsExpert mathematical models into executable, documented ROS 2 packages following official patterns
- 🧪 Tests: Code generation workflow (7 phases); validation includes colcon build, pytest, launch_testing, code quality checks; example scenarios demonstrate FK service, IMU publisher, balance controller implementations
- 📁 Files: Created `.specify/agents/ROS2Engineer.md` (comprehensive agent definition, ~18KB) and updated `.specify/agents/README.md` (registry with updated ecosystem)
- 🔁 Next prompts: ROS2Engineer ready for invocation during code example generation; will implement RoboticsExpert models for Chapters 3-11; recommend testing with FK service node generation to validate workflow
- 🧠 Reflection: ROS2Engineer agent design prioritizes Constitution Principle VI (Code & Simulation Standards) as PRIMARY. Agent translates RoboticsExpert mathematics → executable ROS 2 code, maintaining traceability (math model → implementation). Collaboration pattern: RoboticsExpert derives equations → ROS2Engineer implements → ValidationAgent verifies builds/tests → ContentGeneration integrates into chapters. All code must pass colcon build, pytest, and code quality checks before inclusion.

## Evaluation notes (flywheel)

- Failure modes observed: None; agent creation successful
- Graders run and results (PASS/FAIL):
  - PASS: Agent definition comprehensive with all 7 responsibility domains
  - PASS: ROS 2 Humble expertise well-documented (rclpy, ament, colcon, DDS)
  - PASS: Clear boundaries (generates code; delegates math/diagrams/narrative)
  - PASS: Best practices enforced (PEP 8, ROS 2 conventions, QoS, URDF standards)
  - PASS: Interaction patterns documented (upstream from RoboticsExpert/BookPlanner; downstream to ContentGeneration/SimulationAgent)
  - PASS: Constitution compliance verified (Principle VI as PRIMARY)
  - PASS: Knowledge domains mapped to chapters (3-11)
  - PASS: Testing framework comprehensive (pytest, launch_testing, code quality)
  - PASS: Example scenarios demonstrate workflow (FK service, IMU publisher, balance controller)
  - PASS: Output formats specified (package structure, code metadata)
- Prompt variant (if applicable): N/A (initial agent creation)
- Next experiment (smallest change to try): Test ROS2Engineer by generating FK service node from RoboticsExpert FK derivation; validate package.xml dependencies correct; verify colcon build succeeds; assess code quality (ruff/flake8); measure integration workflow effectiveness (RoboticsExpert math → ROS2Engineer code → ValidationAgent tests)
