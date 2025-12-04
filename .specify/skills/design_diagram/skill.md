# Skill: design_diagram

**Version**: 1.0.0
**Primary Agent**: DiagramAgent
**Purpose**: Request technical diagrams and visualizations (Mermaid, conceptual, mathematical).

## Inputs
- diagram_type (string: conceptual | mathematical | system_architecture | flowchart | sequence)
- description (string)
- key_elements (array)
- notation (string, optional)
- output_format (string, default: mermaid)

## Outputs
- Mermaid diagram code or image
- Caption and alt text
- Legend (if applicable)

## Success Criteria
- Clearly communicates concept
- Readable labels
- Engineering conventions followed
- Caption provided
- Alt text for accessibility

## Estimated Time: 30-45 minutes per diagram
