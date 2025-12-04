# Skill: validate_mathematics

**Version**: 1.0.0
**Created**: 2025-12-04
**Type**: Technical Validation
**Primary Agent**: RoboticsExpert
**Supporting Agents**: ValidationAgent

## Purpose

Verify mathematical equations, derivations, and notation against standard robotics textbooks (Craig, Spong, Murray) and constitution standards. Ensures zero tolerance for technical errors in mathematics (Constitution Principle I).

## Input Parameters

```yaml
equations:
  type: array
  required: true
  description: "LaTeX-formatted equations to validate"
  example: ["T = \\begin{bmatrix} R & p \\\\ 0 & 1 \\end{bmatrix}"]

derivation_steps:
  type: array
  required: false
  description: "Step-by-step derivation to verify"

notation_standard:
  type: string
  required: false
  default: "Craig"
  enum: ["Craig", "Spong", "Murray"]
  description: "Robotics textbook notation convention"

textbook_reference:
  type: string
  required: true
  description: "Specific textbook equation or section"
  example: "Craig, J.J. Introduction to Robotics, eq. 3.6, p. 45"

numerical_example:
  type: object
  required: false
  description: "Numerical test case to verify equation"
  properties:
    inputs: object
    expected_output: object
```

## Output Format

```yaml
validation_report:
  status: "PASS" | "FAIL"
  equations_checked: integer
  errors_found:
    - equation_id: string
      error_type: "notation" | "mathematical_error" | "missing_variable_definition"
      description: string
      suggested_fix: string
  notation_compliance:
    status: "PASS" | "FAIL"
    issues: array
  textbook_verification:
    reference_found: boolean
    equation_matches: boolean
    notes: string
  numerical_validation:
    test_passed: boolean
    expected: object
    actual: object
    error_margin: float
```

## Validation Criteria

1. **Notation Compliance** (Constitution Principle IV):
   - q for joint angles
   - T for SE(3) transformations
   - R for SO(3) rotations
   - θ (theta), d, a, α (alpha) for DH parameters

2. **Mathematical Correctness**:
   - Equations match textbook references exactly
   - All variables defined before first use
   - Units specified for physical quantities
   - Dimensional analysis correct

3. **Numerical Verification**:
   - Worked examples produce expected results
   - Error margin <0.001 for floating-point calculations
   - Edge cases handled correctly

## Usage Example

```yaml
RoboticsExpert.validate_mathematics({
  equations: [
    "$T_i = \\text{Rot}(z, \\theta_i) \\cdot \\text{Trans}(0, 0, d_i) \\cdot \\text{Trans}(a_i, 0, 0) \\cdot \\text{Rot}(x, \\alpha_i)$"
  ],
  textbook_reference: "Craig, Introduction to Robotics, eq. 3.6",
  notation_standard: "Craig",
  numerical_example: {
    inputs: {theta: 0.785, d: 0, a: 1.0, alpha: 0},
    expected_output: {T: [[0.707, -0.707, 0, 1.0], [0.707, 0.707, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]}
  }
})

→ Output:
{
  status: "PASS",
  equations_checked: 1,
  errors_found: [],
  notation_compliance: {status: "PASS"},
  textbook_verification: {reference_found: true, equation_matches: true},
  numerical_validation: {test_passed: true, error_margin: 0.0001}
}
```

## Success Criteria

- [ ] 100% equations match textbook references
- [ ] All variables defined before use
- [ ] Units specified for physical quantities
- [ ] Numerical examples validate theoretical results
- [ ] Notation follows constitution standards
- [ ] Zero mathematical errors

## Estimated Time

- Simple equation set (1-5 equations): 15-30 minutes
- Complex derivation (>5 equations with proofs): 45-60 minutes
- Full chapter review (10-20 equations): 1.5-2 hours
