# Skill: generate_exercises

**Version**: 1.0.0
**Created**: 2025-12-04
**Type**: Assessment Design
**Primary Agent**: EducationDesigner
**Supporting Agents**: RoboticsExpert, ROS2Engineer

## Purpose

Design exercises spanning all Bloom's taxonomy levels (Remember → Create) aligned to chapter learning objectives. Ensures active learning and skill practice (Constitution Principle II).

## Input Parameters

```yaml
learning_objectives:
  type: array
  required: true
  description: "SMART learning objectives from chapter outline"
  example: ["Compute FK using DH parameters", "Analyze Jacobian singularities"]

chapter_concepts:
  type: array
  required: true
  description: "Key concepts covered in chapter"
  example: ["DH parameters", "Transformation matrices", "Jacobian"]

difficulty_level:
  type: string
  required: true
  enum: ["beginner", "intermediate", "advanced"]

number_of_exercises:
  type: integer
  required: false
  default: 7
  constraints: "Between 5 and 10"

include_solutions:
  type: boolean
  required: false
  default: true
```

## Output Format

```markdown
# Chapter {N} Exercises

## Exercise 1: [Title] (Remember/Understand)
**Difficulty**: Easy
**Topics**: {topic_1}, {topic_2}
**Estimated Time**: 5-10 minutes

### Problem Statement
{clear_description_of_task}

### Hints
- {hint_1}
- {hint_2}

### Solution
{complete_solution_with_explanation}

---

## Exercise 2-7: [Progressive difficulty through Bloom's levels]
...
```

## Exercise Distribution by Bloom's Taxonomy

```yaml
Remember (1-2 exercises):
  - "Define {term} in your own words"
  - "List the steps to {process}"

Understand (1-2 exercises):
  - "Explain why {phenomenon} occurs"
  - "Describe the difference between {A} and {B}"

Apply (2-3 exercises):
  - "Compute {calculation} given {parameters}"
  - "Implement {algorithm} in Python"
  - "Modify code to {variation}"

Analyze (1-2 exercises):
  - "Compare {method_A} vs {method_B}, identify tradeoffs"
  - "Debug the following code/equation and explain error"

Evaluate (0-1 exercise):
  - "Critique {approach} for {application}, propose improvements"

Create (0-1 exercise):
  - "Design {system} integrating chapter concepts"
```

## Quality Requirements

- [ ] Progressive difficulty (easy → challenging)
- [ ] Clear problem statements
- [ ] Hints provided for challenging problems
- [ ] Complete solutions with explanations
- [ ] Rubrics for open-ended problems
- [ ] At least 2 exercises per learning objective
- [ ] Mix of conceptual, mathematical, and coding exercises

## Success Criteria

- Exercises span all Bloom's levels
- Every learning objective has ≥2 exercises
- Difficulty calibrated to chapter (beginner: 60% easy, 30% medium, 10% hard)
- Solutions are pedagogically explained (not just answers)
- Estimated time accurate (tested with sample learners)

## Estimated Time

1.5-2 hours per chapter (7-10 exercises with solutions)
