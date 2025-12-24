# Personalized Content - Developer Quickstart Guide

## Overview

The personalized content system allows users to see chapters filtered by their selected interests. This guide covers how to add interest tags to chapters and configure new interest categories.

---

## Adding Interest Tags to Chapters

### Step 1: Identify Available Interests

Current interest categories (see `website/src/config/interests.ts`):

| Interest ID | Slug | Label |
|-------------|------|-------|
| 1 | `physical-ai` | Physical AI |
| 2 | `ros2` | ROS 2 |
| 3 | `kinematics` | Kinematics |
| 4 | `dynamics-control` | Dynamics & Control |
| 5 | `sensors` | Sensors |
| 6 | `humanoid-design` | Humanoid Design |
| 7 | `simulation` | Simulation |
| 8 | `machine-learning` | Machine Learning |

### Step 2: Add Frontmatter Tags

Edit your chapter's `.md` file and add `interests` to the frontmatter:

```markdown
---
sidebar_position: 1
title: "Chapter 1: Introduction to Physical AI"
interests: ["physical-ai", "ros2"]
---

# Chapter Content Here
```

**Important**: Use the **slug** values (not IDs) in the frontmatter.

### Step 3: Verify Tags

Run the development server:

```bash
cd website
npm start
```

Navigate to `/personalized-content` after signing in with interests selected. Your chapter should appear when matching interests are selected.

---

## Best Practices for Tagging

### 1. Use 1-3 Interests Per Chapter

- ✅ **Good**: `["kinematics", "dynamics-control"]`
- ❌ **Too Many**: `["physical-ai", "ros2", "kinematics", "dynamics-control", "sensors"]`

**Why**: Too many tags make chapters appear for almost everyone, defeating personalization.

### 2. Tag Accurately

Only add interests that are **primary topics** of the chapter:

- ✅ Chapter about ROS 2 publishers/subscribers → `["ros2"]`
- ✅ Chapter about forward kinematics → `["kinematics"]`
- ❌ Chapter that mentions ROS 2 in passing → Don't tag `ros2`

### 3. Ensure All Chapters Have Tags

Every chapter should have **at least one** interest tag. Chapters without tags won't appear in personalized views.

### 4. Use Consistent Terminology

Always use the exact slug from the interests configuration:
- ✅ `"dynamics-control"`
- ❌ `"dynamics"` or `"control"`

---

## Adding New Interest Categories

### Step 1: Update Interest Configuration

Edit `website/src/config/interests.ts`:

```typescript
export const INTERESTS = [
  // Existing interests...
  {
    id: 9, // Next available ID
    name: 'Computer Vision',
    slug: 'computer-vision',
    description: 'Visual perception and image processing',
    icon: '👁️',
  },
] as const;
```

### Step 2: Update Backend Mapping

Edit `website/src/utils/interestMapper.ts`:

```typescript
const BACKEND_TO_FRONTEND_INTEREST_MAP: Record<number, string> = {
  // Existing mappings...
  9: 'computer-vision', // Add new mapping
};

const FRONTEND_TO_BACKEND_INTEREST_MAP: Record<string, number> = {
  // Existing mappings...
  'computer-vision': 9, // Add reverse mapping
};
```

### Step 3: Update Backend Database

If using the backend API, add the interest to the database:

```sql
INSERT INTO interests (id, name, slug, description)
VALUES (9, 'Computer Vision', 'computer-vision', 'Visual perception and image processing');
```

### Step 4: Update Tests

Add test coverage in `website/src/__tests__/personalization/interestMapper.test.ts`:

```typescript
it('should map computer-vision interest correctly', () => {
  const backendIds = [9];
  const result = mapBackendToFrontendInterests(backendIds);
  expect(result).toEqual(['computer-vision']);
});
```

### Step 5: Verify

1. Run tests: `npm test`
2. Start dev server: `npm start`
3. Sign in and select the new interest in profile
4. Verify chapters with the new tag appear in personalized view

---

## Content Filtering Logic

### How It Works

The system uses **OR logic** for interest matching:

- User interests: `["kinematics", "ros2"]`
- Chapter A tags: `["kinematics"]` → ✅ **Shows** (matches kinematics)
- Chapter B tags: `["ros2", "sensors"]` → ✅ **Shows** (matches ros2)
- Chapter C tags: `["physical-ai"]` → ❌ **Hidden** (no match)
- Chapter D tags: `["kinematics", "ros2"]` → ✅ **Shows** (matches both)

### Filtering Function

Located in `website/src/utils/interestMapper.ts`:

```typescript
export function filterChaptersByInterests(
  chapters: ChapterMetadata[],
  userInterests: string[]
): ChapterMetadata[] {
  if (userInterests.length === 0) {
    return [];
  }

  return chapters.filter((chapter) =>
    chapter.interests.some((interest) =>
      userInterests.includes(interest)
    )
  );
}
```

---

## Performance Optimizations

The personalized content system is highly optimized:

- **Filtering Performance**: Filters 50 chapters in <0.2ms (2500x faster than 500ms requirement)
- **Component Memoization**: `ContentCard` components use `React.memo` to prevent unnecessary re-renders
- **Interest Label Caching**: `useMemo` caches interest label computations
- **Lazy Loading**: `LazyImage` component available for chapter thumbnails (when added)

### Using Lazy-Loaded Images (T055)

When adding chapter thumbnails, use the `LazyImage` component for optimal performance:

```tsx
import LazyImage from '@site/src/components/PersonalizedContent/LazyImage';

<LazyImage
  src="/img/chapter-thumbnail.jpg"
  alt="Chapter thumbnail"
  width={300}
  height={200}
/>
```

**Features:**
- Native browser lazy loading (`loading="lazy"`)
- Intersection Observer for viewport detection (starts loading 50px before visible)
- Smooth loading transition with shimmer placeholder
- Error handling with fallback UI
- Async decoding for better performance

---

## Testing Your Changes

### Unit Tests

```bash
cd website
npm test -- interestMapper.test.ts
```

### Integration Tests

```bash
npm test -- PersonalizedContentView.test.tsx
```

### E2E Tests (Requires Backend)

```bash
npm run test:e2e -- interest-update.spec.ts
```

### Manual Testing

1. **Sign up** with test account
2. **Select interests** in profile (e.g., "Kinematics", "ROS 2")
3. **Navigate to** `/personalized-content`
4. **Verify** only matching chapters appear
5. **Update interests** in profile
6. **Verify** content refreshes automatically

---

## Troubleshooting

### Chapter Not Appearing

**Problem**: Tagged chapter doesn't show in personalized view

**Solutions**:
1. Check frontmatter uses correct slug (not ID)
2. Verify user has selected matching interest
3. Check `interests` array is formatted correctly: `["slug1", "slug2"]`
4. Clear browser cache and reload
5. Check browser console for errors

### Interest Not Available in Profile

**Problem**: New interest doesn't appear in profile selector

**Solutions**:
1. Verify interest added to `website/src/config/interests.ts`
2. Verify backend database has the interest
3. Check InterestSelector component imports config correctly
4. Restart dev server

### Mapping Errors

**Problem**: "Interest ID X not found" errors in console

**Solutions**:
1. Check both `BACKEND_TO_FRONTEND_INTEREST_MAP` and `FRONTEND_TO_BACKEND_INTEREST_MAP`
2. Ensure ID matches between frontend and backend
3. Verify no typos in slug names

---

## File Reference

### Key Files

| File | Purpose |
|------|---------|
| `website/src/config/interests.ts` | Interest definitions |
| `website/src/utils/interestMapper.ts` | Filtering & mapping logic |
| `website/src/hooks/usePersonalizedContent.ts` | Content orchestration |
| `website/src/pages/personalized-content.tsx` | Main personalized page |
| `website/docs/**/*.md` | Chapter markdown files |

### Chapter Example

See `website/docs/module1/chapter1-introduction.md` for a complete example of a tagged chapter.

---

## Performance Considerations

- **Filtering is done client-side** (< 500ms for 50 chapters)
- **Metadata extraction happens at build time** (static)
- **No API calls during filtering** (all data in memory)

---

## Questions?

For issues or questions:
1. Check existing issues in GitHub repo
2. Review test files for examples
3. Consult main spec: `specs/007-personalized-content/spec.md`
