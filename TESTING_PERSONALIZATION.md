# Testing Guide: Personalization Feature (Phase 6)

**Feature**: User Interest-Based Personalization
**Status**: Implementation Complete (T056-T066)
**Testing Phase**: T067-T068

This document provides comprehensive testing scenarios for the personalization feature that adapts chatbot responses based on user interests and background.

---

## Overview

The personalization feature enables the chatbot to tailor responses using:
- **Interest-specific analogies** (8 interest categories mapped to domain-specific examples)
- **Background-based tone** (student vs professional communication styles)
- **Language preference awareness** (English vs Urdu)
- **Visual indicators** (✨ Personalized badge in chat UI)

---

## Test Environment Setup

### Prerequisites
1. Backend running on `http://localhost:8000`
2. Frontend running on `http://localhost:3000`
3. PostgreSQL database with interests schema
4. User with ID: `00000000-0000-0000-0000-000000000001` (temp testing user)

### Database Verification
```sql
-- Verify user_profiles table
SELECT * FROM user_profiles WHERE user_id = '00000000-0000-0000-0000-000000000001';

-- Verify user_interests table
SELECT ui.*, ic.name, ic.slug
FROM user_interests ui
JOIN interest_categories ic ON ui.interest_id = ic.id
WHERE ui.user_id = '00000000-0000-0000-0000-000000000001';
```

---

## Test Scenarios

### Phase 1: Backend Personalization (T056-T064)

#### Test 1.1: PersonalizationService - Build Personalized Prompt
**Objective**: Verify that personalized prompts are correctly generated based on user interests.

**Steps**:
1. Create test user with interests: `software-engineering`, `artificial-intelligence-machine-learning`
2. Call `build_personalized_system_prompt(user_id, base_prompt)`
3. Verify response contains:
   - `is_personalized: True`
   - `user_interests: ["Software Engineering", "Artificial Intelligence & Machine Learning"]`
   - `background: "student"` or `"professional"`
   - `language_preference: "en"` or `"ur"`

**Expected Result**:
```python
{
  "personalized_prompt": "<base_prompt>\n\n## Personalization Context\n\nYou are communicating with a **student** who has expressed interest in:\n- Software Engineering: ...\n- Artificial Intelligence & Machine Learning: ...\n\n## Communication Style\n...",
  "user_interests": ["Software Engineering", "Artificial Intelligence & Machine Learning"],
  "is_personalized": True,
  "background": "student",
  "language_preference": "en"
}
```

**Validation Checklist**:
- [ ] Personalized prompt includes user interests section
- [ ] Correct tone/approach for background (student/professional)
- [ ] Interest-specific analogies and examples included
- [ ] Grounding reminder included
- [ ] Language preference noted (if Urdu)

---

#### Test 1.2: PersonalizationService - No Interests Fallback
**Objective**: Verify graceful fallback when user has no interests.

**Steps**:
1. Create test user with no interests saved
2. Call `build_personalized_system_prompt(user_id, base_prompt)`
3. Verify response contains:
   - `is_personalized: False`
   - `user_interests: []`
   - `personalized_prompt == base_prompt` (unchanged)

**Expected Result**:
```python
{
  "personalized_prompt": "<unchanged base prompt>",
  "user_interests": [],
  "is_personalized": False,
  "background": None,
  "language_preference": "en"
}
```

**Validation Checklist**:
- [ ] Returns base prompt without modification
- [ ] is_personalized flag is False
- [ ] No error thrown

---

#### Test 1.3: Agent Integration - Personalized Chat Request
**Objective**: Verify that chat requests with user_id trigger personalized agent creation.

**Steps**:
1. Send POST request to `/agent/chat`:
```json
{
  "message": "What is inverse kinematics?",
  "session_id": "test_session_001",
  "context_text": null,
  "user_id": "00000000-0000-0000-0000-000000000001"
}
```
2. Check backend logs for:
   - `"Personalized agent for user <user_id>"`
   - Personalization service called
   - User profile fetched

**Expected Result**:
- Agent created with personalized system prompt
- Response includes analogies/examples relevant to user's interests
- Backend logs confirm personalization was applied

**Validation Checklist**:
- [ ] Backend logs show "Personalized agent for user..."
- [ ] Response contains interest-specific analogies
- [ ] Tone matches user's background (student/professional)

---

#### Test 1.4: Agent Integration - No User ID Fallback
**Objective**: Verify that requests without user_id use base agent.

**Steps**:
1. Send POST request to `/agent/chat` without `user_id`:
```json
{
  "message": "What is inverse kinematics?",
  "session_id": "test_session_002",
  "context_text": null
}
```
2. Check backend logs

**Expected Result**:
- Base agent used (no personalization)
- Backend logs do NOT show personalization messages
- Response is generic (no user-specific analogies)

**Validation Checklist**:
- [ ] Base agent used
- [ ] No personalization service calls
- [ ] Generic response returned

---

### Phase 2: Frontend Integration (T065-T066)

#### Test 2.1: ChatContext - Send User ID with Requests
**Objective**: Verify that ChatContext includes user_id in API requests when user is signed in.

**Steps**:
1. Sign in as test user (user_id stored in localStorage)
2. Open ChatKit
3. Send a message: "Explain PID control"
4. Open browser DevTools → Network → Check request payload

**Expected Result**:
```json
{
  "message": "Explain PID control",
  "session_id": "...",
  "context_text": null,
  "user_id": "00000000-0000-0000-0000-000000000001"
}
```

**Validation Checklist**:
- [ ] Request includes `user_id` field
- [ ] user_id matches localStorage value
- [ ] Console logs show: `"User ID for personalization: <user_id>"`

---

#### Test 2.2: ChatContext - Guest User (No User ID)
**Objective**: Verify that guest users (not signed in) send requests without user_id.

**Steps**:
1. Clear localStorage: `localStorage.removeItem('user_id')`
2. Open ChatKit
3. Send a message
4. Check request payload in DevTools

**Expected Result**:
```json
{
  "message": "Explain PID control",
  "session_id": "...",
  "context_text": null,
  "user_id": null
}
```

**Validation Checklist**:
- [ ] Request includes `user_id: null` or omits field
- [ ] Console logs show: `"User ID for personalization: null"`
- [ ] No errors in console

---

#### Test 2.3: ChatWindow - Personalization Indicator (Active)
**Objective**: Verify that "✨ Personalized" badge appears when user has interests.

**Steps**:
1. Sign in as user with saved interests
2. Open ChatKit
3. Check chat window header

**Expected Result**:
- Badge appears: `✨ Personalized`
- Subtitle shows: `"Tailored for Software Engineering & AI +2"` (or similar)
- Hovering badge shows tooltip: `"Personalized for: Software Engineering, AI, ..."`

**Validation Checklist**:
- [ ] Badge is visible in header
- [ ] Badge has green styling
- [ ] Subtitle shows first 2 interests + count
- [ ] Tooltip shows all interests

**Visual Verification**:
```
┌─────────────────────────────────────┐
│ 🤖 Robotics Assistant ✨ Personalized │
│    Tailored for Software Eng. & AI +1│
└─────────────────────────────────────┘
```

---

#### Test 2.4: ChatWindow - No Personalization Indicator (Guest)
**Objective**: Verify that badge does NOT appear for guest users.

**Steps**:
1. Clear localStorage
2. Open ChatKit
3. Check chat window header

**Expected Result**:
- No badge visible
- Subtitle shows: `"Humanoid Robotics Guide"`

**Validation Checklist**:
- [ ] No ✨ badge
- [ ] Generic subtitle displayed
- [ ] No API call to `/interests/{user_id}`

---

#### Test 2.5: Sign-In/Sign-Up - User ID Storage
**Objective**: Verify that user_id is stored in localStorage after authentication.

**Steps**:
1. Go to `/signup`
2. Complete account creation
3. Select interests
4. Check localStorage in DevTools

**Expected Result**:
```javascript
localStorage.getItem('user_id') // "00000000-0000-0000-0000-000000000001"
localStorage.getItem('user_has_interests') // "true"
```

**Validation Checklist**:
- [ ] user_id stored after sign-up
- [ ] user_has_interests flag set to "true"
- [ ] Both values persist after page refresh

---

### Phase 3: End-to-End Personalization (T067-T068)

#### Test 3.1: Interest-Specific Analogies (Software Engineering)
**Objective**: Verify that responses contain software engineering analogies.

**Setup**:
- User interests: `software-engineering`
- Background: `student`

**Steps**:
1. Sign in as user
2. Ask: "What is inverse kinematics?"
3. Analyze response for analogies

**Expected Analogies** (should contain at least ONE):
- "similar to designing a software architecture"
- "like a class hierarchy in object-oriented programming"
- "analogous to API design patterns"
- Keywords: code, algorithm, function, class, API

**Validation Checklist**:
- [ ] Response contains software engineering analogy
- [ ] Explanation uses programming terminology
- [ ] Examples reference coding/development
- [ ] Response remains factually accurate

---

#### Test 3.2: Interest-Specific Analogies (Mechanical Engineering)
**Objective**: Verify that responses contain mechanical engineering analogies.

**Setup**:
- User interests: `mechanical-engineering`
- Background: `professional`

**Steps**:
1. Sign in as user
2. Ask: "Explain the ZMP (Zero Moment Point)"
3. Analyze response

**Expected Analogies** (should contain at least ONE):
- "like mechanical linkages in a machine"
- "similar to how gears transfer motion and force"
- "analogous to stress distribution in mechanical structures"
- Keywords: force, torque, mechanism, motion, dynamics

**Validation Checklist**:
- [ ] Response contains mechanical engineering analogy
- [ ] Professional tone (concise, technical)
- [ ] Examples reference mechanical systems
- [ ] Response remains factually accurate

---

#### Test 3.3: Background Tone - Student vs Professional
**Objective**: Verify that tone differs based on background setting.

**Test A - Student Background**:
- User: background=student, interests=control-systems
- Message: "What is PID control?"
- Expected tone: Educational, step-by-step, encouraging
- Expected style: "Let's break this down..." "Think of it like..."

**Test B - Professional Background**:
- User: background=professional, interests=control-systems
- Message: "What is PID control?"
- Expected tone: Concise, technical, practical
- Expected style: "PID control implements..." "Key trade-offs include..."

**Validation Checklist**:
- [ ] Student: Longer explanations with analogies
- [ ] Student: Encouragement and learning resources
- [ ] Professional: Concise with implementation focus
- [ ] Professional: Trade-offs and best practices mentioned

---

#### Test 3.4: Multiple Interests - Combined Analogies
**Objective**: Verify that multiple interests produce diverse analogies.

**Setup**:
- User interests: `software-engineering`, `control-systems`, `robotics-hardware`
- Background: `student`

**Steps**:
1. Ask: "How do I design a robot arm controller?"
2. Analyze response

**Expected Elements**:
- Software engineering: API design, code architecture
- Control systems: PID tuning, feedback loops
- Robotics hardware: actuator selection, sensor integration

**Validation Checklist**:
- [ ] Response draws from multiple interest domains
- [ ] Analogies span different fields
- [ ] Connections between interests explained
- [ ] Holistic answer addressing multiple perspectives

---

#### Test 3.5: Language Preference - Urdu User
**Objective**: Verify that Urdu preference is acknowledged (even though responses are in English).

**Setup**:
- User: language_preference=ur
- Interests: any

**Steps**:
1. Sign in as Urdu-preferring user
2. Send any message
3. Check personalized prompt (backend logs)

**Expected Prompt Section**:
```
## Language Preference

The user prefers content in Urdu. While you should respond in English
(as translation is handled separately), be mindful that the user may
benefit from simpler vocabulary and clearer explanations that translate well.
```

**Validation Checklist**:
- [ ] Personalized prompt includes language preference section
- [ ] Response uses simpler vocabulary
- [ ] Explanations are clear and translation-friendly

---

#### Test 3.6: Personalization Caching
**Objective**: Verify that personalization check is cached for performance.

**Steps**:
1. Sign in and open ChatKit (first time)
2. Check Network tab - API call to `/interests/{user_id}` should occur
3. Close and reopen ChatKit
4. Check Network tab - should use cached value (no API call)

**Expected Result**:
- First open: API call made
- Subsequent opens: No API call (cached in localStorage)

**Validation Checklist**:
- [ ] Initial load fetches interests
- [ ] Cache stored: `localStorage.getItem('user_has_interests') === 'true'`
- [ ] Subsequent loads skip API call
- [ ] Cache invalidated after profile update

---

#### Test 3.7: Profile Update Invalidates Personalization
**Objective**: Verify that updating interests refreshes personalization.

**Steps**:
1. Sign in with interests: `software-engineering`
2. Open ChatKit → Ask question → Verify software analogies
3. Go to `/profile` → Change interests to `mechanical-engineering`
4. Return to ChatKit → Ask same question
5. Verify response now uses mechanical analogies

**Validation Checklist**:
- [ ] First response uses software analogies
- [ ] After profile update, cache refreshed
- [ ] Second response uses mechanical analogies
- [ ] ChatKit shows updated interests in badge tooltip

---

### Phase 4: Error Handling & Edge Cases

#### Test 4.1: Invalid User ID
**Objective**: Verify graceful handling of invalid user ID.

**Steps**:
1. Manually set localStorage: `localStorage.setItem('user_id', 'invalid-uuid')`
2. Open ChatKit
3. Send message

**Expected Result**:
- Backend rejects invalid UUID (validation error)
- Frontend falls back to base agent
- No personalization applied
- User receives response (no crash)

**Validation Checklist**:
- [ ] Backend validates UUID format
- [ ] Frontend handles error gracefully
- [ ] Chat functionality still works
- [ ] Error logged but not shown to user

---

#### Test 4.2: User with Deleted Interests
**Objective**: Verify handling when user's interests are deleted from DB.

**Steps**:
1. Sign in with cached `user_has_interests=true`
2. Manually delete user interests from database:
   ```sql
   DELETE FROM user_interests WHERE user_id = '00000000-0000-0000-0000-000000000001';
   ```
3. Open ChatKit
4. Send message

**Expected Result**:
- API call to `/interests/{user_id}` returns 404
- Cache updated: `user_has_interests=false`
- Personalization badge disappears
- Base agent used for response

**Validation Checklist**:
- [ ] Badge removed on next load
- [ ] Cache updated to false
- [ ] No errors in console
- [ ] Base agent provides response

---

#### Test 4.3: Backend Personalization Service Failure
**Objective**: Verify fallback when PersonalizationService fails.

**Steps**:
1. Simulate error: Temporarily break interests service (e.g., database down)
2. Send chat request with user_id
3. Check response

**Expected Result**:
- Backend logs warning: `"Personalization failed, fallback to base agent"`
- Base agent used
- User receives response (no error to user)

**Validation Checklist**:
- [ ] Fallback to base agent on error
- [ ] Backend logs error
- [ ] User experience unaffected
- [ ] Response still returned

---

#### Test 4.4: Network Timeout - Interests Fetch
**Objective**: Verify handling when interests API call times out.

**Steps**:
1. Throttle network in DevTools to "Slow 3G"
2. Open ChatKit (triggers interests fetch)
3. Observe behavior

**Expected Result**:
- Request times out after reasonable period
- UI shows loading state briefly
- Falls back to non-personalized mode
- Chat still functional

**Validation Checklist**:
- [ ] Timeout handled gracefully
- [ ] Loading state shown during fetch
- [ ] Fallback to base mode on timeout
- [ ] User can still send messages

---

### Phase 5: Factual Accuracy Validation (T068)

#### Test 5.1: Verify Grounded Responses with Personalization
**Objective**: Ensure personalized responses remain factually accurate and grounded in source material.

**Test Case A - Software Engineering Interest**:
- User: interests=software-engineering
- Message: "What is the Denavit-Hartenberg convention?"
- Verify:
  1. Response includes software analogy (e.g., "like defining class interfaces")
  2. Core explanation is factually correct (matches source material)
  3. Analogy enhances understanding WITHOUT contradicting facts

**Test Case B - Mechanical Engineering Interest**:
- User: interests=mechanical-engineering
- Message: "Explain bipedal locomotion stability"
- Verify:
  1. Response includes mechanical analogy (e.g., "like balancing forces in structures")
  2. ZMP/stability concepts are accurate
  3. Mechanical context aids understanding without misinformation

**Validation Checklist**:
- [ ] Analogies are supplementary, not replacements for facts
- [ ] Core concepts match source material
- [ ] Citations still included when available
- [ ] Grounding instruction enforced: "always prioritize factual accuracy"

---

#### Test 5.2: Cross-Validate Personalized vs Base Responses
**Objective**: Ensure personalization doesn't change factual content.

**Steps**:
1. Create 2 test users:
   - User A: No interests (base agent)
   - User B: Interests = software-engineering
2. Ask both: "What is inverse kinematics?"
3. Compare responses

**Expected Result**:
- **Core facts identical**: Both explain inverse kinematics correctly
- **User B addition**: Includes software analogy
- **User B style**: More step-by-step (if student background)
- **Both grounded**: Citations from same sources

**Validation Checklist**:
- [ ] Factual content matches between responses
- [ ] User B has additional analogies/examples
- [ ] No contradictions introduced
- [ ] Both cite same sources (if available)

---

## Test Results Template

Use this template to record test results:

```markdown
### Test: [Test ID - Test Name]
**Date**: YYYY-MM-DD
**Tester**: [Name]
**Status**: ✅ Pass / ❌ Fail / ⚠️ Partial

**Environment**:
- Backend: [version/commit]
- Frontend: [version/commit]
- Database: PostgreSQL [version]

**Results**:
[Detailed observations]

**Issues Found**:
- [Issue 1]
- [Issue 2]

**Screenshots/Logs**:
[Attach relevant screenshots or log snippets]
```

---

## Critical Success Criteria

**Phase 6 is considered complete when**:

1. ✅ **Backend Personalization** (T056-T064):
   - [ ] PersonalizationService generates correct prompts
   - [ ] Interest-to-analogy mappings work
   - [ ] Background tones applied correctly
   - [ ] Agent integration includes user_id handling
   - [ ] Fallback to base agent on errors

2. ✅ **Frontend Integration** (T065-T066):
   - [ ] ChatContext sends user_id with requests
   - [ ] ChatWindow shows personalization indicator
   - [ ] Sign-in/sign-up store user_id in localStorage
   - [ ] Profile updates refresh personalization cache

3. ✅ **Testing & Validation** (T067-T068):
   - [ ] All 8 interest categories produce relevant analogies
   - [ ] Student vs professional tones are distinct
   - [ ] Multiple interests combine correctly
   - [ ] Personalized responses remain factually accurate
   - [ ] Error handling works for all edge cases

4. ✅ **User Experience**:
   - [ ] Personalization is transparent (badge visible)
   - [ ] Guest users unaffected (no errors)
   - [ ] Performance acceptable (< 2s for personalization check)
   - [ ] Cache reduces redundant API calls

---

## Known Limitations

1. **Temporary User IDs**: Uses hardcoded test user ID until better-auth is integrated
2. **Language Preference**: Urdu noted but responses still in English (translation separate)
3. **Analogy Selection**: Only first analogy/example from each interest used
4. **Cache Invalidation**: Manual (profile update) only; no TTL expiration

---

## Next Steps (Post-Phase 6)

After completing Phase 6 testing:

1. **Phase 7**: Integrate Translation + Personalization
   - T069-T074: Combine personalization with Urdu translation

2. **Phase 8**: Polish & Cross-Cutting
   - T075-T091: Security, performance, documentation

3. **Better-Auth Integration**:
   - Replace temp user IDs with real authentication
   - Add user session management
   - Implement proper user registration flow

---

**Testing Document Version**: 1.0
**Last Updated**: 2025-12-18
**Related Tasks**: T056-T068
**Related Files**:
- `backend/services/personalization_service.py`
- `backend/main.py` (agent integration)
- `website/src/contexts/ChatContext.tsx`
- `website/src/components/ChatKit/ChatWindow.tsx`
- `website/src/lib/user.ts`
