---
description: "Task breakdown for Urdu Translation, Text Summarization, and Personalization feature"
feature: urdu-translation-personalization
branch: 5-urdu-translation-personalization
created: 2025-12-18
---

# Tasks: Urdu Translation, Text Summarization, and Personalization

**Input**: Design documents from `/specs/5-urdu-translation-personalization/`
**Prerequisites**: plan.md, spec.md, data-model.md

**Tests**: Tests are NOT explicitly requested in the specification, so test tasks are excluded. Focus is on implementation and manual validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Based on plan.md:
- **Backend**: `backend/` (FastAPI Python application)
- **Frontend**: `website/src/` (React/Docusaurus application)
- **Database**: Neon PostgreSQL (remote, migrations in `backend/migrations/`)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for translation/personalization features

- [X] T001 Create backend directory structure for translation/summarization services in backend/services/
- [X] T002 [P] Install translation dependencies (googletrans or azure-translator) in backend/requirements.txt
- [X] T003 [P] Install or verify Cohere SDK for summarization in backend/requirements.txt
- [X] T004 [P] Install Neon PostgreSQL adapter (psycopg2-binary or asyncpg) in backend/requirements.txt
- [X] T005 Create environment variables template in backend/.env.example for translation API keys, Neon DB connection
- [X] T006 [P] Create frontend components directory structure in website/src/components/Translation/ and website/src/components/Summarization/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T007 Setup Neon PostgreSQL database connection in backend/database/connection.py
- [X] T008 Create database migration framework structure in backend/migrations/
- [X] T009 Create migration 001: user_profiles, interest_categories, user_interests tables per data-model.md in backend/migrations/001_create_user_profiles_and_interests.sql
- [X] T010 [P] Implement database repository pattern base class in backend/repositories/base_repository.py
- [X] T011 [P] Create Pydantic models for API requests/responses in backend/models/translation.py, backend/models/summarization.py, backend/models/interests.py
- [X] T012 [P] Setup error handling classes (TranslationError, SummarizationError, PersonalizationError) in backend/utils/errors.py
- [X] T013 [P] Create utility functions for code detection and preservation in backend/utils/text_processing.py
- [X] T014 Seed interest_categories table with 8 predefined categories per spec.md in backend/migrations/002_seed_interest_categories.sql
- [X] T015 Run database migrations and verify schema in Neon PostgreSQL

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Translate Content to Urdu (Priority: P1) 🎯 MVP

**Goal**: Enable users to translate selected book text and chatbot responses to Urdu with RTL formatting

**Independent Test**: Select any text passage in the book, click "Translate to Urdu", verify Urdu translation appears with RTL formatting and code/technical terms preserved

### Implementation for User Story 1

- [X] T016 [P] [US1] Create TranslationService class in backend/services/translation_service.py
- [X] T017 [US1] Implement translate_to_urdu() method with external API integration in backend/services/translation_service.py
- [X] T018 [US1] Implement code preservation logic (detect and skip code blocks) in backend/services/translation_service.py
- [X] T019 [US1] Add translation caching mechanism (optional, for performance) in backend/services/translation_service.py
- [X] T020 [US1] Create POST /translate endpoint in backend/main.py
- [X] T021 [US1] Add request validation (max 5000 words, required fields) in backend/main.py for /translate endpoint
- [X] T022 [US1] Add error handling for translation API failures in backend/main.py for /translate endpoint
- [X] T023 [P] [US1] Create TranslateButton component in website/src/components/Translation/TranslateButton.tsx
- [X] T024 [P] [US1] Create TranslatedText component with RTL styling in website/src/components/Translation/TranslatedText.tsx
- [X] T025 [US1] Integrate TranslateButton into book reading components in website/src/components/
- [X] T026 [US1] Add toggle functionality (Show Original / Show Urdu) in website/src/components/Translation/TranslateButton.tsx
- [X] T027 [US1] Add CSS for RTL text rendering in website/src/css/translation.css
- [X] T028 [US1] Test translation with sample book text and chatbot responses

**Checkpoint**: At this point, User Story 1 should be fully functional - users can translate text to Urdu independently

---

## Phase 4: User Story 2 - Summarize Selected Text (Priority: P1)

**Goal**: Enable users to summarize selected book sections for quick comprehension

**Independent Test**: Select a multi-paragraph section, click "Summarize", verify concise summary (20-30% of original length) appears

### Implementation for User Story 2

- [ ] T029 [P] [US2] Create SummarizationService class in backend/services/summarization_service.py
- [ ] T030 [US2] Implement summarize_text() method using Cohere LLM in backend/services/summarization_service.py
- [ ] T031 [US2] Add length validation (minimum 50 words, maximum 5000 words) in backend/services/summarization_service.py
- [ ] T032 [US2] Implement summary length control (target 20-30% of original) in backend/services/summarization_service.py
- [ ] T033 [US2] Create POST /summarize endpoint in backend/main.py
- [ ] T034 [US2] Add request validation and error handling in backend/main.py for /summarize endpoint
- [ ] T035 [P] [US2] Create SummarizeButton component in website/src/components/Summarization/SummarizeButton.tsx
- [ ] T036 [P] [US2] Create SummaryDisplay component in website/src/components/Summarization/SummaryDisplay.tsx
- [ ] T037 [US2] Integrate SummarizeButton into book reading components in website/src/components/
- [ ] T038 [US2] Add loading states and error handling for summarization in website/src/components/Summarization/SummarizeButton.tsx
- [ ] T039 [US2] Test summarization with various text lengths and content types

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently - translation and summarization features are complete

---

## Phase 5: User Story 3 - Declare and Manage Interests (Priority: P2)

**Goal**: Enable users to select and manage their background interests for personalization

**Independent Test**: Create account, select 2-5 interests during sign-up, verify interests are saved and displayed in profile settings

### Implementation for User Story 3

- [ ] T040 [P] [US3] Create UserInterestsRepository class in backend/repositories/user_interests_repository.py
- [ ] T041 [US3] Implement save_user_interests() method in backend/repositories/user_interests_repository.py
- [ ] T042 [US3] Implement get_user_interests() method in backend/repositories/user_interests_repository.py
- [ ] T043 [US3] Implement update_user_interests() method in backend/repositories/user_interests_repository.py
- [ ] T044 [P] [US3] Create InterestsService class in backend/services/interests_service.py
- [ ] T045 [US3] Implement validation for 2-5 interests selection in backend/services/interests_service.py
- [ ] T046 [US3] Create POST /interests/save endpoint in backend/main.py
- [ ] T047 [US3] Create GET /interests/{user_id} endpoint in backend/main.py
- [ ] T048 [US3] Add authentication middleware check for interest endpoints in backend/main.py
- [ ] T049 [P] [US3] Create InterestSelector component in website/src/components/Interests/InterestSelector.tsx
- [ ] T050 [P] [US3] Create InterestCard component for displaying individual interests in website/src/components/Interests/InterestCard.tsx
- [ ] T051 [US3] Integrate InterestSelector into sign-up flow in website/src/pages/ (or auth components)
- [ ] T052 [US3] Integrate InterestSelector into sign-in flow (optional step) in website/src/pages/ (or auth components)
- [ ] T053 [US3] Create user profile settings page with interest management in website/src/pages/profile.tsx
- [ ] T054 [US3] Add CSS styling for interest selection UI in website/src/css/interests.css
- [ ] T055 [US3] Test interest selection, saving, and retrieval with different user accounts

**Checkpoint**: At this point, users can declare and manage interests - foundation for personalization is ready

---

## Phase 6: User Story 4 - Receive Personalized Chatbot Responses (Priority: P2)

**Goal**: Deliver chatbot responses with examples and analogies relevant to user's declared interests

**Independent Test**: Ask chatbot the same question with different user interest profiles, verify responses adapt with relevant examples

### Implementation for User Story 4

- [ ] T056 [P] [US4] Create PersonalizationService class in backend/services/personalization_service.py
- [ ] T057 [US4] Implement build_personalized_system_prompt() method in backend/services/personalization_service.py
- [ ] T058 [US4] Create interest-to-analogy mapping logic in backend/services/personalization_service.py
- [ ] T059 [US4] Implement fact-checking mechanism (compare with original RAG response) in backend/services/personalization_service.py
- [ ] T060 [US4] Extend POST /agent/chat endpoint to accept user_id parameter in backend/main.py
- [ ] T061 [US4] Integrate PersonalizationService into agent workflow in backend/main.py
- [ ] T062 [US4] Fetch user interests before generating response in backend/main.py for /agent/chat
- [ ] T063 [US4] Inject personalized system prompt into Cohere agent in backend/main.py
- [ ] T064 [US4] Add fallback to generic response if user has no interests in backend/main.py
- [ ] T065 [P] [US4] Update ChatKit components to send user_id with chat requests in website/src/components/ChatKit/
- [ ] T066 [US4] Add UI indicator showing personalization is active in website/src/components/ChatKit/
- [ ] T067 [US4] Test personalized responses with different interest combinations (Software Engineering, Mechanical Engineering, Student, etc.)
- [ ] T068 [US4] Validate that personalized responses remain factually accurate against original content

**Checkpoint**: Personalization is now active - chatbot adapts responses based on user interests

---

## Phase 7: User Story 5 - Combine Translation with Personalization (Priority: P3)

**Goal**: Enable users to receive personalized chatbot responses in Urdu

**Independent Test**: Set user interests, ask question, receive personalized response, translate to Urdu, verify both personalization and translation work together

### Implementation for User Story 5

- [ ] T069 [US5] Extend POST /translate endpoint to handle personalized responses in backend/main.py
- [ ] T070 [US5] Test translation of personalized examples and analogies in backend/services/translation_service.py
- [ ] T071 [US5] Ensure technical terms in personalized responses are preserved during translation in backend/services/translation_service.py
- [ ] T072 [P] [US5] Update frontend to allow translation of personalized chatbot responses in website/src/components/ChatKit/
- [ ] T073 [US5] Test combined workflow: user interests → personalized response → Urdu translation
- [ ] T074 [US5] Verify that translated personalized responses maintain accuracy and relevance

**Checkpoint**: All user stories are now complete - translation, summarization, and personalization work independently and together

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories and production readiness

- [ ] T075 [P] Add comprehensive error logging for all services in backend/utils/logging.py
- [ ] T076 [P] Implement rate limiting for translation/summarization endpoints in backend/main.py
- [ ] T077 [P] Add performance monitoring (response time tracking) in backend/main.py
- [ ] T078 [P] Create API documentation for all endpoints in backend/docs/api.md
- [ ] T079 [P] Add user feedback mechanism for translation quality in website/src/components/Translation/
- [ ] T080 [P] Add user feedback mechanism for summarization quality in website/src/components/Summarization/
- [ ] T081 [P] Optimize database queries (add indexes for user_interests lookups) in backend/migrations/003_add_indexes.sql
- [ ] T082 [P] Setup connection pooling for Neon PostgreSQL in backend/database/connection.py
- [ ] T083 [P] Add input sanitization for all endpoints in backend/utils/validation.py
- [ ] T084 [P] Implement API key rotation mechanism in backend/utils/secrets.py
- [ ] T085 [P] Add frontend loading states and error boundaries in website/src/components/
- [ ] T086 Security audit: Ensure no sensitive data logged in backend/
- [ ] T087 Performance testing: Verify 5-second translation SLA for 1000-word passages
- [ ] T088 Performance testing: Verify system handles 100 concurrent translation requests
- [ ] T089 [P] Update user documentation in website/docs/ explaining translation, summarization, and personalization features
- [ ] T090 Code cleanup and refactoring across all services
- [ ] T091 Run full end-to-end validation of all 5 user stories

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Story 1 (Phase 3 - Translation)**: Depends on Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (Phase 4 - Summarization)**: Depends on Foundational (Phase 2) - Independent of US1
- **User Story 3 (Phase 5 - Interests)**: Depends on Foundational (Phase 2) - Independent of US1/US2
- **User Story 4 (Phase 6 - Personalization)**: Depends on Foundational (Phase 2) AND User Story 3 (requires interests)
- **User Story 5 (Phase 7 - Combined)**: Depends on User Story 1 (translation) AND User Story 4 (personalization)
- **Polish (Phase 8)**: Depends on all implemented user stories being complete

### User Story Dependencies

- **User Story 1 (P1 - Translation)**: Can start after Foundational - No dependencies on other stories
- **User Story 2 (P1 - Summarization)**: Can start after Foundational - No dependencies on other stories
- **User Story 3 (P2 - Interests)**: Can start after Foundational - No dependencies on other stories
- **User Story 4 (P2 - Personalization)**: REQUIRES User Story 3 (must have interests data) - Can start after US3 complete
- **User Story 5 (P3 - Combined)**: REQUIRES User Story 1 AND User Story 4 - Can start after both complete

### Within Each User Story

- Backend services before API endpoints
- API endpoints before frontend components
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

**Phase 1 (Setup)**: T002, T003, T004, T006 can run in parallel (different files)

**Phase 2 (Foundational)**: T010, T011, T012, T013 can run in parallel (different files)

**Phase 3 (US1 - Translation)**:
- T016, T023, T024 can run in parallel (backend service, frontend components are independent)
- After T017-T022 complete: T023-T027 can run in parallel (all frontend work)

**Phase 4 (US2 - Summarization)**:
- T029, T035, T036 can run in parallel (backend service, frontend components are independent)
- After T030-T034 complete: T035-T038 can run in parallel (all frontend work)

**Phase 5 (US3 - Interests)**:
- T040, T044, T049, T050 can run in parallel (repository, service, frontend components are independent)

**Phase 6 (US4 - Personalization)**:
- T056, T065 can run in parallel (backend service, frontend updates are independent initially)

**Phase 8 (Polish)**: T075, T076, T077, T078, T079, T080, T081, T082, T083, T084, T085, T089 can run in parallel (different files/concerns)

**User Stories in Parallel** (if team has capacity):
- Once Foundational (Phase 2) is complete:
  - US1 (Translation) can proceed in parallel with US2 (Summarization) and US3 (Interests)
  - US1 and US2 are completely independent
  - US3 must complete before US4 can start
  - US4 must complete before US5 can start

---

## Parallel Example: User Story 1 (Translation)

```bash
# After backend service is implemented, launch frontend components in parallel:
Task T023: "Create TranslateButton component in website/src/components/Translation/TranslateButton.tsx"
Task T024: "Create TranslatedText component with RTL styling in website/src/components/Translation/TranslatedText.tsx"

# These can be developed simultaneously by different developers or AI agents
```

## Parallel Example: Foundational Phase

```bash
# Launch all foundational infrastructure tasks in parallel:
Task T010: "Implement database repository pattern base class in backend/repositories/base_repository.py"
Task T011: "Create Pydantic models in backend/models/"
Task T012: "Setup error handling classes in backend/utils/errors.py"
Task T013: "Create utility functions for code detection in backend/utils/text_processing.py"
```

---

## Implementation Strategy

### MVP First (User Stories 1 + 2 Only - P1 Features)

1. ✅ Complete Phase 1: Setup (T001-T006)
2. ✅ Complete Phase 2: Foundational (T007-T015) - CRITICAL
3. ✅ Complete Phase 3: User Story 1 - Translation (T016-T028)
4. ✅ Complete Phase 4: User Story 2 - Summarization (T029-T039)
5. **STOP and VALIDATE**: Test translation and summarization independently
6. Deploy/demo - Users can now translate and summarize without authentication

**MVP Deliverable**: Translation and Summarization features working for all users (no auth required)

### Incremental Delivery (Add Personalization - P2 Features)

1. ✅ Complete MVP (Phases 1-4)
2. ✅ Complete Phase 5: User Story 3 - Interests (T040-T055)
3. ✅ Complete Phase 6: User Story 4 - Personalization (T056-T068)
4. **STOP and VALIDATE**: Test personalization with different interest profiles
5. Deploy/demo - Authenticated users now get personalized responses

**Incremental Deliverable**: Translation + Summarization + Personalization (requires auth)

### Complete Feature Set (Add Combined Features - P3)

1. ✅ Complete MVP + Incremental (Phases 1-6)
2. ✅ Complete Phase 7: User Story 5 - Combined (T069-T074)
3. ✅ Complete Phase 8: Polish (T075-T091)
4. **FINAL VALIDATION**: End-to-end testing of all user stories
5. Production deployment

**Full Deliverable**: All features complete - translation, summarization, personalization, and combinations

### Parallel Team Strategy

With multiple developers:

**Phase 1-2**: Team works together on Setup + Foundational (foundation for all stories)

**After Foundational Complete**:
- Developer A: User Story 1 (Translation) - T016-T028
- Developer B: User Story 2 (Summarization) - T029-T039
- Developer C: User Story 3 (Interests) - T040-T055

**After US3 Complete**:
- Developer D: User Story 4 (Personalization) - T056-T068 (depends on US3)

**After US1 + US4 Complete**:
- Developer E: User Story 5 (Combined) - T069-T074 (depends on US1 and US4)

**Final Phase**:
- All developers: Polish tasks (T075-T091) - can be distributed in parallel

---

## Notes

- **[P] tasks**: Different files, no dependencies - safe to run in parallel
- **[Story] label**: Maps task to specific user story for traceability
- **Each user story is independently testable**: Can validate US1 without US2, etc.
- **MVP scope**: User Stories 1 + 2 (P1) deliver immediate value without authentication
- **P2 features require auth**: User Stories 3 + 4 depend on existing better-auth system
- **Stop at checkpoints**: Validate each story independently before proceeding
- **Commit strategy**: Commit after each task or logical group
- **No test tasks included**: Specification does not explicitly request tests; validation is manual
- **Translation API selection**: T002 requires choosing between Google Translate or Azure Translator (decision in Phase 0/Setup)
- **Neon DB connection**: Ensure environment variables are set before T007

---

## Task Summary

**Total Tasks**: 91 tasks

**Task Count by User Story**:
- Phase 1 (Setup): 6 tasks
- Phase 2 (Foundational): 9 tasks
- Phase 3 (US1 - Translation): 13 tasks
- Phase 4 (US2 - Summarization): 11 tasks
- Phase 5 (US3 - Interests): 16 tasks
- Phase 6 (US4 - Personalization): 13 tasks
- Phase 7 (US5 - Combined): 6 tasks
- Phase 8 (Polish): 17 tasks

**Parallel Opportunities**: 35 tasks marked with [P] can run in parallel within their phases

**Independent Test Criteria**:
- US1: Select text → Translate → Verify Urdu with RTL
- US2: Select text → Summarize → Verify concise summary
- US3: Sign up → Select interests → Verify saved
- US4: Ask question → Verify personalized response
- US5: Get personalized response → Translate → Verify both work

**Suggested MVP Scope**: Phases 1-4 (Translation + Summarization) = 39 tasks
