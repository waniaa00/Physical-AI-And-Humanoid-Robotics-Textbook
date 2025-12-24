# User Interests Feature Testing Guide

This document provides comprehensive test scenarios for the user interests feature integrated into the Humanoid Robotics documentation platform.

## Overview

The user interests feature allows users to:
- Select 2-5 interests during sign-up or sign-in
- Specify their background (student/professional)
- Choose their preferred language (English/Urdu)
- View and update their interests in a profile page
- Personalize their learning experience

## Prerequisites

1. **Backend Setup**:
   ```bash
   cd backend
   poetry install
   # Set environment variables
   export NEON_DATABASE_URL="your_neon_postgres_url"
   export COHERE_API_KEY="your_cohere_api_key"

   # Run migrations
   poetry run python backend/migrations/run_migrations.py

   # Start backend server
   poetry run uvicorn main:app --reload
   ```

2. **Frontend Setup**:
   ```bash
   cd website
   npm install
   npm start
   ```

3. **Verify API**:
   - Backend: `http://localhost:8000`
   - Frontend: `http://localhost:3000`
   - API Docs: `http://localhost:8000/docs`

4. **Database Seeded Data**:
   - 8 interest categories should be seeded:
     1. Software Engineering
     2. Mechanical Engineering
     3. Electrical Engineering
     4. Artificial Intelligence & Machine Learning
     5. Robotics Hardware
     6. Computer Vision
     7. Control Systems
     8. Human-Robot Interaction

## Test Scenarios

### Scenario 1: Sign Up with Interests (Happy Path)

**Location**: `/signup`

**Steps**:
1. Navigate to the sign-up page
2. **Step 1 - Account Details**:
   - Fill in: Name, Email, Password, Confirm Password
   - Click "Next: Select Interests →"
3. Verify progress indicator shows Step 1 completed, Step 2 active
4. **Step 2 - Interests**:
   - Select background: "Student"
   - Select language: "English"
   - Select exactly 3 interests (e.g., Software Engineering, AI & ML, Robotics Hardware)
5. Observe selection counter: "3 / 5 selected"
6. Click "Complete Sign Up"

**Expected Results**:
- ✅ Progress indicator animates correctly (Step 1 green checkmark, Step 2 active)
- ✅ Interest cards are displayed in a responsive grid
- ✅ Selected cards have blue background and checkmark
- ✅ Counter shows "3 / 5 selected"
- ✅ Validation text: "✓ You can select 2 more if you'd like"
- ✅ "Complete Sign Up" button is enabled
- ✅ User is redirected to `/docs/intro` after successful sign-up
- ✅ Database contains user profile with 3 interests

**Test Cases**:
- TC1.1: Multi-step wizard functions correctly
- TC1.2: Can navigate back to Step 1 and forward again
- TC1.3: Interests are persisted when navigating between steps

---

### Scenario 2: Sign Up with Minimum Interests (2)

**Location**: `/signup`

**Steps**:
1. Complete Step 1 (account details)
2. In Step 2, select background: "Professional"
3. Select exactly 2 interests
4. Click "Complete Sign Up"

**Expected Results**:
- ✅ Counter shows "2 / 5 selected"
- ✅ Validation text: "✓ You can select 3 more if you'd like"
- ✅ Button is enabled with 2 interests
- ✅ Sign-up completes successfully

**Test Cases**:
- TC2.1: Minimum 2 interests requirement is enforced
- TC2.2: Professional background is saved correctly

---

### Scenario 3: Sign Up with Maximum Interests (5)

**Location**: `/signup`

**Steps**:
1. Complete Step 1
2. In Step 2, select 5 interests
3. Try to select a 6th interest

**Expected Results**:
- ✅ Counter shows "5 / 5 selected"
- ✅ Validation text: "Maximum selections reached"
- ✅ 6th interest card is disabled (grayed out)
- ✅ Clicking disabled cards has no effect
- ✅ Can deselect an interest to select a different one
- ✅ Sign-up completes with all 5 interests

**Test Cases**:
- TC3.1: Cannot select more than 5 interests
- TC3.2: Deselection works when at maximum
- TC3.3: Can swap interests when at maximum

---

### Scenario 4: Sign Up - Try with Only 1 Interest (Validation)

**Location**: `/signup`

**Steps**:
1. Complete Step 1
2. In Step 2, select only 1 interest
3. Try to click "Complete Sign Up"

**Expected Results**:
- ✅ Counter shows "1 / 5 selected"
- ✅ Validation text (red): "Select at least 1 more interest"
- ✅ "Complete Sign Up" button is disabled (grayed out)
- ✅ Clicking disabled button has no effect
- ✅ After selecting 2nd interest, button becomes enabled

**Test Cases**:
- TC4.1: Button disabled with 0 or 1 interest
- TC4.2: Validation message is clear
- TC4.3: Button enables when 2nd interest selected

---

### Scenario 5: Sign Up - Back Button Navigation

**Location**: `/signup`

**Steps**:
1. Complete Step 1 with email: "test@example.com"
2. On Step 2, select 3 interests
3. Click "← Back"
4. Verify email field still contains "test@example.com"
5. Click "Next: Select Interests →"
6. Verify 3 interests are still selected

**Expected Results**:
- ✅ Back button navigates to Step 1
- ✅ Form data from Step 1 is preserved
- ✅ Selected interests from Step 2 are preserved
- ✅ Progress indicator updates correctly
- ✅ Can complete sign-up after navigating back/forward

**Test Cases**:
- TC5.1: State persistence across navigation
- TC5.2: No data loss when using back button

---

### Scenario 6: Sign In - Existing User with Interests

**Location**: `/signin`

**Steps**:
1. Sign in with credentials for a user who already has interests
2. Submit the form

**Expected Results**:
- ✅ User is authenticated
- ✅ System checks for existing interests (API call to `/interests/{user_id}`)
- ✅ User is redirected directly to `/docs/intro` (skips interests prompt)
- ✅ No interests selection UI is shown

**Test Cases**:
- TC6.1: Existing users bypass interests step
- TC6.2: API check happens automatically

---

### Scenario 7: Sign In - New User without Interests (Optional Prompt)

**Location**: `/signin`

**Steps**:
1. Sign in with credentials for a user who has no interests
2. Submit the form
3. Observe "One More Thing..." prompt
4. Click "Add My Interests"
5. Select 3 interests
6. Click "Save & Continue"

**Expected Results**:
- ✅ After authentication, prompt appears: "Help us personalize your learning experience"
- ✅ Two options: "Add My Interests" and "Skip for Now"
- ✅ Clicking "Add My Interests" shows interest selection UI
- ✅ Interests are saved successfully
- ✅ User is redirected to `/docs/intro`

**Test Cases**:
- TC7.1: Prompt appears for users without interests
- TC7.2: Interests can be added post-signin
- TC7.3: Flow completes successfully

---

### Scenario 8: Sign In - Skip Interests (Optional)

**Location**: `/signin`

**Steps**:
1. Sign in with a user who has no interests
2. On the "One More Thing..." prompt, click "Skip for Now"

**Expected Results**:
- ✅ User is redirected directly to `/docs/intro`
- ✅ No interests are saved
- ✅ User can add interests later via profile page

**Test Cases**:
- TC8.1: Skipping interests is allowed
- TC8.2: User is not blocked from accessing content

---

### Scenario 9: Profile Page - View Existing Interests

**Location**: `/profile`

**Steps**:
1. Sign in as a user with existing interests
2. Navigate to `/profile`

**Expected Results**:
- ✅ Profile page loads successfully
- ✅ Current interests are displayed as colored badges
- ✅ Background is shown: "🎓 Student" or "💼 Professional"
- ✅ Language preference: "🇬🇧 English" or "🇵🇰 Urdu (اردو)"
- ✅ Interest count is displayed: "Your Interests (3)"
- ✅ Last updated date is shown
- ✅ "Edit Profile" button is visible

**Test Cases**:
- TC9.1: Profile data loads correctly
- TC9.2: UI displays all profile fields
- TC9.3: Date formatting is correct

---

### Scenario 10: Profile Page - Edit Interests

**Location**: `/profile`

**Steps**:
1. Navigate to profile page
2. Click "Edit Profile"
3. Change background from "Student" to "Professional"
4. Deselect 1 interest and select a different one
5. Click "Save Changes"

**Expected Results**:
- ✅ Clicking "Edit Profile" switches to edit mode
- ✅ Current selections are pre-populated
- ✅ Can change background and language
- ✅ Can modify interest selections
- ✅ "Save Changes" button is enabled when valid (2-5 interests)
- ✅ Success message appears: "Profile updated successfully!"
- ✅ Profile view updates with new data
- ✅ Success message disappears after 3 seconds

**Test Cases**:
- TC10.1: Edit mode pre-populates current data
- TC10.2: Changes are saved correctly
- TC10.3: Success feedback is shown

---

### Scenario 11: Profile Page - Cancel Edit

**Location**: `/profile`

**Steps**:
1. Navigate to profile page (with existing interests)
2. Click "Edit Profile"
3. Change some selections
4. Click "Cancel"

**Expected Results**:
- ✅ Edit mode closes
- ✅ Returns to view mode
- ✅ Original profile data is restored (changes discarded)
- ✅ No API call is made

**Test Cases**:
- TC11.1: Cancel discards changes
- TC11.2: Original data is preserved

---

### Scenario 12: Profile Page - First Time User (No Interests)

**Location**: `/profile`

**Steps**:
1. Sign in as a user with no interests (who skipped during sign-in)
2. Navigate to `/profile`

**Expected Results**:
- ✅ Profile page loads
- ✅ Detects no existing interests (404 from API)
- ✅ Immediately shows interest selection UI (edit mode)
- ✅ No "Cancel" button (cannot cancel when no profile exists)
- ✅ User must select 2-5 interests to save
- ✅ After saving, profile view is shown

**Test Cases**:
- TC12.1: New users start in edit mode
- TC12.2: Must complete profile before viewing

---

### Scenario 13: API Error Handling - Backend Down

**Location**: `/signup` or `/profile`

**Steps**:
1. Stop the backend server
2. Try to select interests in sign-up or profile page
3. Try to submit the form

**Expected Results**:
- ✅ Interest categories fail to load
- ✅ Error message: "Failed to fetch interest categories"
- ✅ "Retry" button appears
- ✅ Clicking retry attempts to fetch again
- ✅ When submitting, clear error message: "Failed to save interests"
- ✅ UI remains functional, user can retry

**Test Cases**:
- TC13.1: Graceful degradation when API is down
- TC13.2: Retry mechanism works
- TC13.3: Error messages are user-friendly

---

### Scenario 14: API Validation - Invalid User ID

**Location**: Profile page or direct API call

**Steps**:
1. Make API request with invalid UUID format

```bash
curl -X POST "http://localhost:8000/interests/save" \
  -H "Content-Type: application/json" \
  -d '{
    "user_id": "not-a-uuid",
    "interest_ids": [1, 2, 3],
    "background": "student",
    "language_preference": "en"
  }'
```

**Expected Results**:
- ✅ 400 Bad Request
- ✅ Error: "INVALID_USER_ID"
- ✅ Message: "user_id must be a valid UUID"

**Test Cases**:
- TC14.1: UUID validation works
- TC14.2: Clear error messages returned

---

### Scenario 15: API Validation - Too Few Interests

**Steps**:
```bash
curl -X POST "http://localhost:8000/interests/save" \
  -H "Content-Type: application/json" \
  -d '{
    "user_id": "00000000-0000-0000-0000-000000000001",
    "interest_ids": [1],
    "background": "student",
    "language_preference": "en"
  }'
```

**Expected Results**:
- ✅ 400 Bad Request
- ✅ Error: "TOO_FEW_INTERESTS"
- ✅ Message: "You must select at least 2 different interests"

**Test Cases**:
- TC15.1: Minimum 2 interests enforced
- TC15.2: Validation error is descriptive

---

### Scenario 16: API Validation - Too Many Interests

**Steps**:
```bash
curl -X POST "http://localhost:8000/interests/save" \
  -H "Content-Type: application/json" \
  -d '{
    "user_id": "00000000-0000-0000-0000-000000000001",
    "interest_ids": [1, 2, 3, 4, 5, 6],
    "background": "student",
    "language_preference": "en"
  }'
```

**Expected Results**:
- ✅ 400 Bad Request
- ✅ Error: "TOO_MANY_INTERESTS"
- ✅ Message: "You can select at most 5 interests"

**Test Cases**:
- TC16.1: Maximum 5 interests enforced
- TC16.2: Validation prevents over-selection

---

### Scenario 17: Keyboard Navigation & Accessibility

**Location**: `/signup` (Step 2)

**Steps**:
1. Navigate to interests step
2. Use Tab key to navigate through interest cards
3. Press Enter or Space to select/deselect
4. Use screen reader (NVDA/JAWS) to read interface

**Expected Results**:
- ✅ All interest cards are keyboard-focusable
- ✅ Focus indicator is visible (blue ring)
- ✅ Enter/Space keys toggle selection
- ✅ ARIA attributes present:
  - `role="checkbox"`
  - `aria-checked="true/false"`
  - `aria-label` with description
  - `aria-disabled` when max reached
- ✅ Screen reader announces: "[Interest Name]: [Description]. Checkbox, [checked/not checked]"

**Test Cases**:
- TC17.1: Full keyboard accessibility
- TC17.2: ARIA labels are correct
- TC17.3: Screen reader compatibility

---

### Scenario 18: Responsive Design - Mobile View

**Location**: All pages with interests UI

**Steps**:
1. Resize browser to mobile size (375px width)
2. Test sign-up flow
3. Test profile page
4. Test interest selection

**Expected Results**:
- ✅ Sign-up: Single column layout
- ✅ Interest cards: Stack vertically (1 per row)
- ✅ Progress indicator: Compact with smaller gaps
- ✅ Action buttons: Full width, stacked vertically
- ✅ All text remains readable
- ✅ Touch targets are adequately sized (44x44px minimum)

**Test Cases**:
- TC18.1: Mobile viewport (375px) is usable
- TC18.2: Tablet viewport (768px) shows 2 columns
- TC18.3: Desktop (1024px+) shows 3+ columns

---

### Scenario 19: Dark Mode Compatibility

**Location**: All pages with interests UI

**Steps**:
1. Enable dark mode (browser/OS setting or Docusaurus theme toggle)
2. Navigate through sign-up, sign-in, and profile pages

**Expected Results**:
- ✅ Interest cards:
  - Background: Dark gray (#1e293b)
  - Border: Lighter gray (#334155)
  - Selected: Dark blue (#1e3a5f)
- ✅ Text remains readable (high contrast)
- ✅ Validation messages adapt to dark theme
- ✅ Buttons maintain visibility
- ✅ Progress indicator uses dark-friendly colors

**Test Cases**:
- TC19.1: All components support dark mode
- TC19.2: Contrast ratios meet WCAG standards
- TC19.3: No visual glitches in dark mode

---

### Scenario 20: Database Persistence & Update

**Location**: Backend + Database

**Steps**:
1. Create user profile with 3 interests via API
2. Query database directly:
   ```sql
   SELECT * FROM user_profiles WHERE user_id = '...';
   SELECT * FROM user_interests WHERE user_id = '...';
   ```
3. Update interests via API (change 1 interest)
4. Query database again

**Expected Results**:
- ✅ Initial save: 1 row in `user_profiles`, 3 rows in `user_interests`
- ✅ Update: `updated_at` timestamp changes
- ✅ Old interests are deleted, new ones inserted
- ✅ No duplicate interests exist
- ✅ Foreign key constraints are enforced
- ✅ Transactions ensure atomicity

**Test Cases**:
- TC20.1: Database schema is correct
- TC20.2: Updates are atomic
- TC20.3: Constraints prevent bad data

---

## API Testing

### Direct API Tests (using curl or Postman)

**Test 1: Get All Interest Categories**
```bash
curl "http://localhost:8000/interests/categories/all"
```

**Expected Response**:
```json
{
  "categories": [
    {
      "id": 1,
      "name": "Software Engineering",
      "slug": "software-engineering",
      "description": "Programming, algorithms, software architecture, and development practices"
    },
    ...
  ],
  "count": 8
}
```

**Test 2: Save User Interests (Valid)**
```bash
curl -X POST "http://localhost:8000/interests/save" \
  -H "Content-Type: application/json" \
  -d '{
    "user_id": "123e4567-e89b-12d3-a456-426614174000",
    "interest_ids": [1, 2, 4],
    "background": "student",
    "language_preference": "en"
  }'
```

**Expected Response** (201 Created):
```json
{
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "background": "student",
  "language_preference": "en",
  "interest_ids": [1, 2, 4],
  "created_at": "2025-01-15T10:30:00",
  "updated_at": "2025-01-15T10:30:00"
}
```

**Test 3: Get User Interests**
```bash
curl "http://localhost:8000/interests/123e4567-e89b-12d3-a456-426614174000"
```

**Expected Response** (200 OK):
```json
{
  "user_id": "123e4567-e89b-12d3-a456-426614174000",
  "background": "student",
  "language_preference": "en",
  "created_at": "2025-01-15T10:30:00",
  "updated_at": "2025-01-15T10:30:00",
  "interests": [
    {
      "id": 1,
      "name": "Software Engineering",
      "slug": "software-engineering",
      "description": "..."
    },
    ...
  ]
}
```

**Test 4: Get Non-Existent User**
```bash
curl "http://localhost:8000/interests/00000000-0000-0000-0000-000000000999"
```

**Expected Response** (404 Not Found):
```json
{
  "error": "USER_NOT_FOUND",
  "message": "No interests found for user 00000000-0000-0000-0000-000000000999"
}
```

---

## Integration Points

### Better-Auth Integration (Future)

When better-auth is fully integrated, verify:
1. User ID from auth context is used correctly
2. Protected routes require authentication
3. Token/session is passed to API calls
4. Profile page requires authenticated user

---

## Success Criteria

- ✅ All 20 scenarios pass
- ✅ API tests return expected responses
- ✅ Database constraints prevent invalid data
- ✅ No console errors or warnings
- ✅ Dark mode and responsive design work
- ✅ Accessibility standards met (WCAG 2.1 AA)
- ✅ Error handling is robust
- ✅ State management works across navigation

---

## Known Limitations

1. **Temporary User IDs**: Currently using hard-coded UUID (`00000000-0000-0000-0000-000000000001`) - replace with actual auth system
2. **No Authentication**: better-auth integration pending
3. **No Authorization**: All endpoints are currently public
4. **In-Memory Only**: InterestSelector fetches categories on every mount (no global state/caching)

---

## Next Steps

After successful testing:
1. Integrate with better-auth for real user authentication
2. Add authorization middleware to protect endpoints
3. Implement user context provider for global state
4. Add analytics tracking for interest selections
5. Create interest-based content recommendations (Phase 6: Personalization)
6. Implement A/B testing for interest selection UI variations
