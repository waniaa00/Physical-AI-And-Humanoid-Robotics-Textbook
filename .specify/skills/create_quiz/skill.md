# Skill: create_quiz

**Version**: 1.0.0
**Primary Agent**: EducationDesigner
**Purpose**: Generate 10-15 assessment questions (MCQ, T/F, short answer) aligned to learning objectives with >70% target pass rate.

## Inputs
- learning_objectives (array)
- chapter_sections (array)
- number_of_questions (int, default: 12)
- question_types (object: {mcq: 7, true_false: 3, short_answer: 2})

## Outputs
- Quiz with 10-15 questions
- Answer key with explanations
- Difficulty calibration (40% easy, 40% medium, 20% hard)
- Alignment matrix (questions → objectives)

## Success Criteria
- Every learning objective covered by ≥2 questions
- Distractors based on common misconceptions
- Explanations for all answers
- Target pass rate >70% achievable

## Estimated Time: 1-1.5 hours
