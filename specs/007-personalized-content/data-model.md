# Data Model Documentation: Personalized Content Feature

## Overview
This document defines the data structures and mapping strategies for the personalized content feature (T013-T014).

---

## Core Data Models

### 1. Interest

**Purpose**: Represents a user-selectable interest category
**Location**: `website/src/types/personalization.ts:6-11`

```typescript
export interface Interest {
  id: string;              // Unique identifier (e.g., "physical-ai", "ros2")
  label: string;           // Human-readable display name (e.g., "Physical AI")
  description: string;     // Detailed description for UI
  icon?: string;           // Optional emoji or icon (e.g., "🤖")
}
```

#### Example Instance
```typescript
const physicalAI: Interest = {
  id: 'physical-ai',
  label: 'Physical AI',
  description: 'AI systems that interact with the physical world through sensors and actuators',
  icon: '🤖'
};
```

#### Available Interests
**Source**: `website/src/config/interests.config.ts:18-67`

| ID | Label | Icon | Description |
|---|---|---|---|
| `physical-ai` | Physical AI | 🤖 | AI systems interacting with physical world |
| `ros2` | ROS 2 Fundamentals | ⚙️ | Robot Operating System 2 middleware |
| `kinematics` | Kinematics | 🦾 | Forward/inverse kinematics, Jacobians |
| `dynamics-control` | Dynamics & Control | 🎛️ | Robot dynamics, PID controllers |
| `sensors` | Sensors & Perception | 👁️ | IMUs, cameras, LIDAR, sensor fusion |
| `humanoid-design` | Humanoid Design | 🧍 | Humanoid architecture and hardware |
| `simulation` | Simulation | 🎮 | Gazebo, virtual testing environments |
| `machine-learning` | Machine Learning | 🧠 | ML for robotics, reinforcement learning |

**Total**: 8 predefined interests

---

### 2. ChapterMetadata

**Purpose**: Represents metadata for a single book chapter
**Location**: `website/src/types/personalization.ts:13-22`

```typescript
export interface ChapterMetadata {
  id: string;              // Unique chapter identifier (e.g., "chapter3-kinematics")
  title: string;           // Display title (e.g., "Chapter 3: Kinematics")
  url: string;             // Docusaurus route (e.g., "/docs/module1/chapter3-kinematics")
  interests: string[];     // Associated interest IDs (e.g., ["kinematics", "ros2"])
  module?: string;         // Module grouping (e.g., "module1")
  order?: number;          // Sequence number within module
  description?: string;    // Brief chapter summary
}
```

#### Example Instance
```typescript
const kinematicsChapter: ChapterMetadata = {
  id: 'chapter3-kinematics',
  title: 'Chapter 3: Kinematics',
  url: '/docs/module1/chapter3-kinematics',
  interests: ['kinematics'],
  module: 'module1',
  order: 3,
  description: 'Forward and inverse kinematics, Jacobians, and robot motion planning'
};
```

#### Static Metadata (MVP)
**Source**: `website/src/utils/metadataExtractor.ts:25-102`
- **Total Chapters**: 8 (placeholder data)
- **Module 1 Chapters**: 8 chapters covering foundations through machine learning

**Example Chapters**:
- Chapter 1: Introduction to Physical AI → `['physical-ai', 'ros2']`
- Chapter 2: ROS 2 Fundamentals → `['ros2']`
- Chapter 3: Kinematics → `['kinematics']`
- Chapter 4: Dynamics and Control → `['dynamics-control']`
- Chapter 5: Sensors and Perception → `['sensors', 'ros2']`

#### Future: Docusaurus Integration
**Frontmatter Schema** (to be added in T025):
```yaml
---
sidebar_position: 3
title: "Chapter 3: Robot Kinematics"
interests: ["kinematics", "ros2"]
---
```

**Extraction** (post-MVP):
```typescript
import { useAllDocsData } from '@docusaurus/plugin-content-docs/client';

// Extract frontmatter from all .md files
// Transform to ChapterMetadata[] array
```

---

### 3. PersonalizedContentView

**Purpose**: Represents the computed personalized content for a user
**Location**: `website/src/types/personalization.ts:24-31`

```typescript
export interface PersonalizedContentView {
  userInterests: string[];           // User's selected interest IDs
  matchedChapters: ChapterMetadata[]; // Chapters matching user interests
  emptyState: boolean;                // True if no interests or no matches
  totalChapters: number;              // Total available chapters in book
  matchCount: number;                 // Number of matched chapters
}
```

#### Example Instance
```typescript
const personalizedView: PersonalizedContentView = {
  userInterests: ['kinematics', 'ros2'],
  matchedChapters: [
    // Chapter 1: interests include 'ros2'
    // Chapter 2: interests include 'ros2'
    // Chapter 3: interests include 'kinematics'
    // Chapter 5: interests include 'ros2'
  ], // 4 matched chapters
  emptyState: false,
  totalChapters: 8,
  matchCount: 4
};
```

#### Empty State Cases
```typescript
// Case 1: No interests selected
{
  userInterests: [],
  matchedChapters: [],
  emptyState: true,  // ← Shows InterestPrompt
  totalChapters: 8,
  matchCount: 0
}

// Case 2: Interests selected, but no matching chapters
{
  userInterests: ['fictional-interest'],
  matchedChapters: [],
  emptyState: true,  // ← Shows "no matches" message
  totalChapters: 8,
  matchCount: 0
}
```

---

### 4. UserProfile

**Purpose**: Represents user account and interest preferences
**Location**: `website/src/types/personalization.ts:33-40`

```typescript
export interface UserProfile {
  id: string;              // User UUID from backend
  email: string;           // User email address
  name?: string;           // Optional display name
  interests: string[];     // Array of frontend interest IDs
  createdAt?: Date;        // Account creation timestamp
  updatedAt?: Date;        // Last profile update timestamp
}
```

#### Example Instance
```typescript
const userProfile: UserProfile = {
  id: '00000000-0000-0000-0000-000000000001',
  email: 'john.doe@example.com',
  name: 'John Doe',
  interests: ['physical-ai', 'ros2', 'kinematics'],
  createdAt: new Date('2025-01-15'),
  updatedAt: new Date('2025-01-20')
};
```

---

## Interest-to-Content Mapping Strategy (T014)

### Mapping Overview

**Matching Logic**: Chapters are matched if **ANY chapter interest matches ANY user interest** (OR logic)

**Implementation**: `website/src/utils/interestMapper.ts:17-36`

```typescript
export function filterChaptersByInterests(
  chapters: ChapterMetadata[],
  userInterests: string[]
): ChapterMetadata[] {
  if (!userInterests || userInterests.length === 0) {
    return []; // No interests = no matches
  }

  return chapters.filter((chapter) => {
    if (!chapter.interests || chapter.interests.length === 0) {
      return false; // Chapter with no interests = no match
    }

    // Match if ANY chapter interest is in user interests
    return chapter.interests.some((chapterInterest) =>
      userInterests.includes(chapterInterest)
    );
  });
}
```

### Mapping Examples

#### Example 1: Single Interest Match
```typescript
User Interests: ['kinematics']

Chapter 3: { interests: ['kinematics'] }
→ MATCH ✓

Chapter 1: { interests: ['physical-ai', 'ros2'] }
→ NO MATCH ✗
```

#### Example 2: Multiple Interest Match
```typescript
User Interests: ['ros2', 'kinematics']

Chapter 1: { interests: ['physical-ai', 'ros2'] }
→ MATCH ✓ (matches 'ros2')

Chapter 3: { interests: ['kinematics'] }
→ MATCH ✓ (matches 'kinematics')

Chapter 4: { interests: ['dynamics-control'] }
→ NO MATCH ✗
```

#### Example 3: Overlapping Interests
```typescript
User Interests: ['ros2']

Chapter 1: { interests: ['physical-ai', 'ros2'] }
→ MATCH ✓

Chapter 2: { interests: ['ros2'] }
→ MATCH ✓

Chapter 5: { interests: ['sensors', 'ros2'] }
→ MATCH ✓

Result: 3 matched chapters
```

### Sorting Strategy

**Sorting Logic**: Preserve book order (module → chapter order)

**Implementation**: `website/src/utils/interestMapper.ts:70-86`

```typescript
export function sortChaptersByBookOrder(
  chapters: ChapterMetadata[]
): ChapterMetadata[] {
  return [...chapters].sort((a, b) => {
    // Sort by module first
    if (a.module && b.module && a.module !== b.module) {
      return a.module.localeCompare(b.module);
    }

    // Then by chapter order
    if (a.order !== undefined && b.order !== undefined) {
      return a.order - b.order;
    }

    // Fallback to URL
    return a.url.localeCompare(b.url);
  });
}
```

**Example**:
```typescript
Matched chapters (before sort):
- Chapter 5: Sensors (module1, order: 5)
- Chapter 1: Introduction (module1, order: 1)
- Chapter 3: Kinematics (module1, order: 3)

After sort:
- Chapter 1: Introduction (module1, order: 1)
- Chapter 3: Kinematics (module1, order: 3)
- Chapter 5: Sensors (module1, order: 5)
```

---

## Backend-Frontend Interest Mapping

### Backend Interest Schema
**Source**: Backend database (Neon PostgreSQL)
**Format**: Numeric IDs (1, 2, 3, ...)

```sql
-- backend/database/schema.sql (example)
CREATE TABLE interests (
  interest_id SERIAL PRIMARY KEY,
  interest_name VARCHAR(255),
  description TEXT
);

-- Sample data:
-- 1: "Physical AI"
-- 2: "ROS 2 Fundamentals"
-- 3: "Kinematics"
```

### Frontend Interest Schema
**Source**: `website/src/config/interests.config.ts`
**Format**: String IDs ('physical-ai', 'ros2', 'kinematics', ...)

### Bidirectional Mapping

#### Backend ID → Frontend ID
```typescript
// Mapping table (to be created in T013 implementation)
const BACKEND_TO_FRONTEND_INTEREST_MAP: Record<number, string> = {
  1: 'physical-ai',
  2: 'ros2',
  3: 'kinematics',
  4: 'dynamics-control',
  5: 'sensors',
  6: 'humanoid-design',
  7: 'simulation',
  8: 'machine-learning',
};

export function mapBackendToFrontendInterests(backendIds: number[]): string[] {
  return backendIds
    .map(id => BACKEND_TO_FRONTEND_INTEREST_MAP[id])
    .filter(id => id !== undefined);
}
```

#### Frontend ID → Backend ID
```typescript
const FRONTEND_TO_BACKEND_INTEREST_MAP: Record<string, number> = {
  'physical-ai': 1,
  'ros2': 2,
  'kinematics': 3,
  'dynamics-control': 4,
  'sensors': 5,
  'humanoid-design': 6,
  'simulation': 7,
  'machine-learning': 8,
};

export function mapFrontendToBackendInterests(frontendIds: string[]): number[] {
  return frontendIds
    .map(id => FRONTEND_TO_BACKEND_INTEREST_MAP[id])
    .filter(id => id !== undefined);
}
```

### Usage in Personalized Content

```typescript
import { useAuth } from '@site/src/hooks/useAuth';
import { usePersonalizedContent } from '@site/src/hooks/usePersonalizedContent';
import { mapBackendToFrontendInterests } from '@site/src/utils/interestMapper';

function PersonalizedContentPage() {
  const { user } = useAuth();

  // user.interests = [1, 2, 3] (backend IDs)
  const frontendInterests = mapBackendToFrontendInterests(user?.interests || []);
  // frontendInterests = ['physical-ai', 'ros2', 'kinematics']

  const { content } = usePersonalizedContent(frontendInterests);
  // content.matchedChapters contains filtered chapters
}
```

---

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│ 1. USER AUTHENTICATION                                          │
│    - User signs in → AuthContext.signIn()                       │
│    - Backend returns: { user_id, email, interests: [1, 2, 3] } │
│    - AuthContext stores user state                              │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│ 2. PERSONALIZED CONTENT PAGE LOAD                               │
│    - Page component uses useAuth() hook                         │
│    - Extracts user.interests (backend IDs: [1, 2, 3])          │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│ 3. INTEREST MAPPING                                             │
│    - mapBackendToFrontendInterests([1, 2, 3])                  │
│    - Returns: ['physical-ai', 'ros2', 'kinematics']            │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│ 4. CONTENT FILTERING                                            │
│    - usePersonalizedContent(frontendInterests)                  │
│    - Calls useContentMetadata() → extractAllChapterMetadata()  │
│    - Calls filterChaptersByInterests(chapters, interests)       │
│    - Calls sortChaptersByBookOrder(matchedChapters)            │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────────┐
│ 5. PERSONALIZED CONTENT VIEW                                    │
│    - Returns PersonalizedContentView object                     │
│    - matchedChapters: filtered and sorted chapters              │
│    - emptyState: true if no matches                             │
│    - Component renders UI based on view                         │
└─────────────────────────────────────────────────────────────────┘
```

---

## Validation Rules

### Interest Validation
**Function**: `validateInterestIds()` in both config and mapper utilities

```typescript
export function validateInterestIds(interestIds: string[]): boolean {
  if (!interestIds || interestIds.length === 0) {
    return true; // Empty array is valid
  }

  const validIds = AVAILABLE_INTERESTS.map((interest) => interest.id);
  return interestIds.every((id) => validIds.includes(id));
}
```

**Usage**:
```typescript
validateInterestIds(['physical-ai', 'ros2'])  // → true
validateInterestIds(['invalid-interest'])      // → false
validateInterestIds([])                        // → true (empty is valid)
```

### Chapter Metadata Validation
**Requirements**:
- `id` must be unique across all chapters
- `url` must start with `/docs/`
- `interests` array must contain valid interest IDs
- `module` should follow pattern `moduleN` (e.g., 'module1', 'module2')
- `order` should be unique within same module

---

## Summary

### Data Models Defined
1. ✅ **Interest** - User-selectable interest categories
2. ✅ **ChapterMetadata** - Book chapter metadata with interest tags
3. ✅ **PersonalizedContentView** - Computed personalized content view
4. ✅ **UserProfile** - User account and preferences

### Mapping Strategy Defined
1. ✅ **Interest-to-Content Mapping** - OR logic with sorting
2. ✅ **Backend-Frontend Interest Mapping** - Bidirectional ID mapping
3. ✅ **Filtering Logic** - `filterChaptersByInterests()` implementation
4. ✅ **Sorting Logic** - `sortChaptersByBookOrder()` implementation

### Implementation Status
- [x] Type definitions created (T001)
- [x] Interest configuration created (T002)
- [x] Filtering utilities created (T008)
- [x] Metadata extraction created (T009)
- [x] Hooks created (T010, T011)
- [x] Data model documented (T013)
- [x] Mapping strategy documented (T014)

### Next Steps
1. Create bidirectional mapping utility functions
2. Begin Phase 3: UI component implementation
3. Add frontmatter tags to all chapter files (T025)
