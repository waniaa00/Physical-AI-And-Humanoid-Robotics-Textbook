# Content Author Guide (T068-T070)

This guide helps content authors add and maintain interest tags for the personalized content feature.

---

## Quick Start

### Adding Interest Tags to a Chapter

1. **Open your chapter file**:
   ```bash
   website/docs/module1/chapter1-introduction.md
   ```

2. **Add `interests` to frontmatter**:
   ```yaml
   ---
   sidebar_position: 1
   title: "Chapter 1: Introduction to Physical AI"
   interests: ["physical-ai", "ros2"]  # ← Add this line
   ---
   ```

3. **Save and test**:
   ```bash
   # Start development server
   npm start

   # Visit personalized content page
   http://localhost:3000/personalized-content
   ```

---

## Available Interest Categories

| Interest ID | Label | Icon | Description |
|-------------|-------|------|-------------|
| `physical-ai` | Physical AI | 🤖 | Embodied AI systems and robotics |
| `ros2` | ROS 2 | 🔧 | Robot Operating System 2 |
| `kinematics` | Kinematics | 📐 | Robot motion and positioning |
| `dynamics-control` | Dynamics & Control | ⚙️ | Control systems and dynamics |
| `sensors` | Sensors | 📡 | Sensor technology and perception |
| `humanoid-design` | Humanoid Design | 🚶 | Humanoid robot architecture |
| `simulation` | Simulation | 💻 | Virtual testing environments |
| `machine-learning` | Machine Learning | 🧠 | ML for robotics |

---

## Tagging Best Practices

### Rule 1: Use 1-3 Interests per Chapter

```yaml
# ✅ GOOD - Focused and specific
interests: ["kinematics", "ros2"]

# ❌ TOO FEW - Missing relevant tags
interests: ["physical-ai"]
# (Chapter about ROS2 kinematics should include both tags)

# ❌ TOO MANY - Overly broad
interests: ["physical-ai", "ros2", "kinematics", "dynamics-control", "sensors"]
# (User sees chapter for every interest - defeats personalization)
```

### Rule 2: Tag Primary Topics, Not Mentions

```yaml
# Chapter: "Introduction to ROS2"
# Content: Focuses on ROS2, mentions sensors briefly

# ✅ GOOD - Tags primary topic
interests: ["ros2"]

# ❌ BAD - Tags every mention
interests: ["ros2", "sensors", "physical-ai", "machine-learning"]
```

### Rule 3: Be Consistent Across Similar Chapters

```yaml
# Chapter 1: "ROS2 Basics"
interests: ["ros2"]

# Chapter 2: "ROS2 Advanced Topics"
interests: ["ros2"]  # ✅ Same tag

# Chapter 3: "ROS2 in Simulation"
interests: ["ros2", "simulation"]  # ✅ Adds simulation
```

---

## Examples by Chapter Type

### Theory Chapters

```yaml
---
title: "Chapter 3: Kinematics Fundamentals"
interests: ["kinematics"]
---
# Focus on single theoretical topic
```

### Tutorial Chapters

```yaml
---
title: "Chapter 5: Building a ROS2 Node"
interests: ["ros2"]
---
# Hands-on implementation
```

### Integration Chapters

```yaml
---
title: "Chapter 7: Sensor Integration with ROS2"
interests: ["ros2", "sensors"]
---
# Combines multiple topics
```

### Design Chapters

```yaml
---
title: "Chapter 6: Humanoid Robot Design"
interests: ["humanoid-design", "physical-ai"]
---
# Design and architecture focus
```

### Advanced Chapters

```yaml
---
title: "Chapter 8: ML for Robot Control"
interests: ["machine-learning", "dynamics-control"]
---
# Advanced cross-cutting topic
```

---

## Validation Checklist

After adding interest tags, verify:

- [ ] **Frontmatter Valid**
  ```bash
  # Check YAML syntax
  # No errors when running dev server
  npm start
  ```

- [ ] **Tags Match Available Interests**
  ```typescript
  // Check against website/src/config/interests.config.ts
  export const AVAILABLE_INTERESTS = [
    { id: 'physical-ai', ... },
    { id: 'ros2', ... },
    // ... etc
  ];
  ```

- [ ] **Chapter Appears in Personalized View**
  ```bash
  # 1. Visit http://localhost:3000/profile
  # 2. Select matching interest
  # 3. Visit http://localhost:3000/personalized-content
  # 4. Verify chapter appears
  ```

- [ ] **Interest Count Reasonable**
  ```yaml
  # Ideal: 1-2 interests
  # Maximum: 3 interests
  # Minimum: 1 interest
  ```

---

## Testing Your Changes

### Manual Testing

1. **Start development server**:
   ```bash
   cd website
   npm start
   ```

2. **Sign in**:
   ```
   http://localhost:3000/signin
   ```

3. **Set interests**:
   ```
   http://localhost:3000/profile
   Select interests matching your chapter
   ```

4. **View personalized content**:
   ```
   http://localhost:3000/personalized-content
   Verify your chapter appears
   ```

5. **Change interests**:
   ```
   Return to /profile
   Deselect interest
   Verify chapter disappears from personalized view
   ```

### Automated Testing

```bash
# Run interest mapper tests
npm test -- interestMapper.test.ts

# Run all personalization tests
npm test -- --testPathPatterns=personalization
```

---

## Common Mistakes and Fixes

### Mistake 1: Typo in Interest ID

```yaml
# ❌ WRONG - Typo in interest slug
interests: ["phsyical-ai"]  # Missing 'y'

# ✅ CORRECT
interests: ["physical-ai"]
```

**Fix**: Always copy interest IDs from `interests.config.ts`

### Mistake 2: Using Display Names Instead of IDs

```yaml
# ❌ WRONG - Using display label
interests: ["Physical AI"]

# ✅ CORRECT - Using slug/ID
interests: ["physical-ai"]
```

**Fix**: Use kebab-case slugs, not human-readable labels

### Mistake 3: Missing Array Brackets

```yaml
# ❌ WRONG - Not an array
interests: "physical-ai"

# ✅ CORRECT - Array format
interests: ["physical-ai"]
```

**Fix**: Always use array syntax `["interest1", "interest2"]`

### Mistake 4: Trailing Commas

```yaml
# ❌ WRONG - Trailing comma in YAML
interests: ["physical-ai", "ros2",]

# ✅ CORRECT - No trailing comma
interests: ["physical-ai", "ros2"]
```

**Fix**: Remove trailing commas in YAML arrays

---

## Adding a New Interest Category

If you need a new interest category not in the current list:

### Step 1: Propose the Interest

Contact the development team with:
- **Proposed Name**: "Computer Vision"
- **Slug**: "computer-vision"
- **Icon**: "👁️"
- **Description**: "Visual perception and image processing"
- **Affected Chapters**: List chapters that would use this tag

### Step 2: Developer Implementation

Developer will:
1. Add to `website/src/config/interests.config.ts`
2. Add backend ID mapping in `interestMapper.ts`
3. Update database (if needed)
4. Add to profile selector

### Step 3: Use the New Interest

Once approved and implemented:
```yaml
---
title: "Chapter X: Computer Vision"
interests: ["computer-vision"]
---
```

---

## Interest Tag Coverage Report

### Current Status (Example)

| Module | Total Chapters | Tagged | Untagged | Coverage |
|--------|---------------|--------|----------|----------|
| Module 1 | 8 | 8 | 0 | 100% ✅ |
| Module 2 | 6 | 4 | 2 | 67% ⚠️ |
| Module 3 | 4 | 2 | 2 | 50% ⚠️ |

**Goal**: 100% coverage across all modules

### Checking Coverage

```bash
# List all chapters
find website/docs -name "*.md" -type f

# Check which chapters have interest tags
grep -r "interests:" website/docs

# Find chapters without interests
grep -L "interests:" website/docs/**/*.md
```

---

## Maintenance Guidelines

### Regular Reviews

**Monthly**: Review interest tag accuracy
- Are tags still relevant?
- Are new tags needed?
- Are any chapters over-tagged?

**After Major Updates**: Update tags when chapter content changes significantly

### Feedback Loop

**Track User Behavior**:
- Which interests are most popular?
- Which chapters appear most in personalized views?
- Are users finding relevant content?

**Adjust Tags**: Based on usage data, refine tags to improve relevance

---

## Quick Reference

### Complete Example

```yaml
---
sidebar_position: 3
title: "Chapter 3: Forward and Inverse Kinematics"
description: "Learn about robot positioning and motion planning"
interests: ["kinematics", "ros2"]
---

# Chapter 3: Forward and Inverse Kinematics

Content goes here...
```

### Interest Tag Template

```yaml
---
title: "Chapter X: Your Title"
interests: ["interest1", "interest2"]
---
```

### Validation Command

```bash
# Validate all markdown files
npm run validate-content  # (if script exists)

# Or manually check
grep -r "interests:" website/docs
```

---

## Support

Questions or issues with interest tagging?

1. **Check Documentation**:
   - `specs/007-personalized-content/quickstart.md`
   - `specs/007-personalized-content/CONTENT-AUTHOR-GUIDE.md`

2. **Run Tests**:
   ```bash
   npm test -- --testPathPatterns=personalization
   ```

3. **Contact Development Team**:
   - Slack: #personalized-content
   - Email: dev-team@example.com

---

## Success Criteria Checklist

Before considering content preparation complete:

- [ ] **All chapters have interest tags** (100% coverage)
- [ ] **Tags are accurate and relevant** (reviewed by subject matter expert)
- [ ] **1-3 interests per chapter** (no over-tagging)
- [ ] **Consistent tagging** (similar chapters use similar tags)
- [ ] **Tested in personalized view** (chapters appear correctly)
- [ ] **Documentation reviewed** (authors understand tagging process)
- [ ] **Feedback collected** (users finding relevant content)

**Target**: 100% chapter coverage with accurate, helpful tags that improve user discovery.
