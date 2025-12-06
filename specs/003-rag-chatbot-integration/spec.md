# Feature Specification: RAG-Powered Chatbot Integration for Humanoid Robotics Textbook

**Feature Branch**: `003-rag-chatbot-integration`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Integrate RAG + Chatbot into Humanoid Robotics Textbook - We are extending the already-published Physical AI & Humanoid Robotics textbook website by adding a full RAG-powered Chatbot layer embedded inside the existing book UI."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Interactive Chapter Q&A (Priority: P1)

As a student reading a chapter on bipedal locomotion, I want to ask questions about specific concepts (like ZMP or CoM control) and receive accurate answers sourced directly from the book content, so I can clarify confusing topics without leaving the reading context.

**Why this priority**: Core value proposition - enables interactive learning within the book interface. This is the foundational capability that all other features build upon. Without this, the chatbot has no purpose.

**Independent Test**: Can be fully tested by opening any chapter, asking a question about content on that page, and verifying the chatbot retrieves and cites the correct book passage in its answer. Delivers immediate value by answering student questions.

**Acceptance Scenarios**:

1. **Given** a student is reading Chapter 9 (Walking and Balance), **When** they open the chatbot and ask "What is Zero Moment Point?", **Then** the chatbot retrieves relevant passages from that chapter, provides an accurate definition with source citations, and indicates which page/section the answer came from.

2. **Given** a student asks a question not covered in the book, **When** the chatbot searches and finds no relevant content, **Then** it responds "I don't know based on the book" rather than generating an answer from external knowledge.

3. **Given** a student is viewing a specific chapter, **When** they ask a question, **Then** the chatbot prioritizes retrieving content from the current chapter before searching the entire book.

---

### User Story 2 - Personalized Chapter Content (Priority: P2)

As a student with a specific background (e.g., software engineer with no robotics hardware experience), I want to click "Personalize this chapter" and have the content re-explained in terms familiar to me (e.g., using software analogies for hardware concepts), so I can better understand material outside my expertise.

**Why this priority**: Significantly enhances learning outcomes for diverse audiences. Builds on the core chatbot (P1) but requires user profiles and onboarding data. High value but can launch after basic Q&A works.

**Independent Test**: Can be tested by creating user profiles with different backgrounds (hardware vs software, beginner vs experienced), clicking "Personalize" on a chapter, and verifying the regenerated content uses appropriate analogies and explanations for that user's background.

**Acceptance Scenarios**:

1. **Given** a user with "software background" and "beginner robotics" profile, **When** they click "Personalize this chapter" on a hardware-focused chapter (e.g., actuators), **Then** the system re-renders the chapter content with software analogies (e.g., "Servo motors are like API endpoints - they accept commands and execute specific actions") appropriate for their level.

2. **Given** a user with "hardware background" and "advanced robotics" profile, **When** they personalize a software-heavy chapter (e.g., ROS 2 programming), **Then** the content emphasizes hardware integration points and skips basic programming concepts.

3. **Given** a user clicks "Personalize this chapter", **When** the personalization process completes, **Then** the original chapter remains accessible via a "View Original" toggle so users can compare versions.

---

### User Story 3 - Text Selection Summarization (Priority: P3)

As a student reviewing a dense technical section, I want to highlight a complex paragraph, click "Summarize Selected Text", and receive a concise explanation in the chatbot, so I can quickly grasp difficult concepts without re-reading multiple times.

**Why this priority**: Convenient learning aid that enhances the core experience. Nice-to-have feature that depends on P1 chatbot working. Lower priority than personalization because it's more of a convenience feature.

**Independent Test**: Can be tested by selecting any paragraph in any chapter, triggering the summarize action, and verifying the chatbot provides a concise summary (2-3 sentences) that accurately captures the key points of the selected text.

**Acceptance Scenarios**:

1. **Given** a student selects a 200-word paragraph about Inverse Kinematics, **When** they click "Summarize Selected Text", **Then** the chatbot displays a 2-3 sentence summary focusing on the core concept and its practical application.

2. **Given** a student selects text spanning multiple concepts (e.g., a full section with equations and explanations), **When** they request a summary, **Then** the chatbot identifies and summarizes each key concept separately in bullet points.

3. **Given** a student selects less than 10 words, **When** they attempt to summarize, **Then** the system prompts "Please select at least one complete sentence for summarization."

---

### User Story 4 - Urdu Chapter Translation (Priority: P4)

As an Urdu-speaking student, I want to click "Translate in Urdu" at the start of a chapter and have the entire chapter content rendered in Urdu while preserving technical terms in English, so I can learn robotics concepts in my native language.

**Why this priority**: Expands accessibility to non-English speakers. High impact for target demographic but requires additional complexity (translation API, maintaining technical accuracy). Can be added after core features are stable.

**Independent Test**: Can be tested by clicking "Translate in Urdu" on any chapter and verifying (1) all general text is translated to Urdu, (2) technical terms remain in English or include English in parentheses, (3) formatting and structure are preserved.

**Acceptance Scenarios**:

1. **Given** a user clicks "Translate in Urdu" on Chapter 9, **When** the translation completes, **Then** all narrative text appears in Urdu, technical terms like "Zero Moment Point" remain in English (or show as "زیرو مومنٹ پوائنٹ (Zero Moment Point)"), and all diagrams/code blocks remain unchanged.

2. **Given** a translated chapter is displayed, **When** the user asks a question in the chatbot, **Then** the chatbot can respond in Urdu and retrieve from both the original English content and translated Urdu content.

3. **Given** a user has translated a chapter, **When** they click "View Original English", **Then** the chapter immediately switches back to English without requiring a page reload.

---

### User Story 5 - User Onboarding and Profile Management (Priority: P1)

As a new user signing up for the textbook, I want to complete an onboarding questionnaire about my background (hardware experience, software experience, robotics experience, programming level), so the system can personalize my learning experience and chatbot responses.

**Why this priority**: Essential foundation for personalization features. Must be implemented early because it affects data schema and user flow. P1 because it's required for P2 (personalization) and enhances P1 (chatbot can reference user background).

**Independent Test**: Can be tested by creating a new account, completing the onboarding form with various background combinations, and verifying the profile data is stored and accessible to personalization/chatbot features.

**Acceptance Scenarios**:

1. **Given** a user clicks "Sign Up", **When** they complete email/password creation, **Then** they are immediately presented with an onboarding form asking for: hardware background (None/Beginner/Intermediate/Advanced), software background (None/Beginner/Intermediate/Advanced), robotics experience (None/Hobbyist/Student/Professional), programming level (None/Beginner/Intermediate/Advanced).

2. **Given** a user completes the onboarding questionnaire, **When** they submit it, **Then** their profile is saved to the database and they are directed to the textbook homepage with a confirmation message "Profile created - your experience will be personalized."

3. **Given** a user skips the onboarding questionnaire, **When** they try to access personalization features, **Then** they are prompted to complete their profile first with a link to the onboarding form.

4. **Given** a logged-in user, **When** they access their profile settings, **Then** they can update their background information at any time and changes immediately affect future chatbot interactions.

---

### User Story 6 - Conversation History and Continuity (Priority: P2)

As a student returning to study after a break, I want to view my previous chat conversations organized by chapter/date and continue where I left off, so I can review what I've already learned and build on previous discussions.

**Why this priority**: Enhances long-term learning value. Important for user retention but not critical for MVP. Depends on P1 chatbot and P5 user profiles working.

**Independent Test**: Can be tested by having multiple chat sessions across different chapters, logging out, logging back in, and verifying all conversations are preserved, searchable, and associated with the correct chapters.

**Acceptance Scenarios**:

1. **Given** a user has had conversations in 3 different chapters, **When** they open the chatbot "History" tab, **Then** they see conversations grouped by chapter with timestamps, sorted by most recent first.

2. **Given** a user selects a previous conversation from history, **When** they click "Continue Conversation", **Then** the chatbot loads the full conversation context and allows them to ask follow-up questions that reference earlier messages.

3. **Given** a user is viewing their conversation history, **When** they search for a keyword (e.g., "inverse kinematics"), **Then** the system highlights all conversations containing that term with the relevant message excerpts shown.

---

### Edge Cases

- **What happens when the chatbot retrieval finds no relevant content?**
  The chatbot MUST respond: "I don't know based on the book. Could you rephrase your question or specify which chapter you're asking about?" It should NEVER hallucinate or use knowledge outside the retrieved book content.

- **How does the system handle concurrent chatbot requests during personalization?**
  If a user triggers "Personalize this chapter" while already having an active chat session, the chat interface displays a "Personalizing..." status and queues new messages until personalization completes (expected < 30 seconds). Messages sent during personalization are answered in the context of the newly personalized content.

- **What happens if a user selects text across multiple chapters (e.g., split-screen view)?**
  The system only allows text selection within a single chapter context. If selection spans multiple chapters, the "Summarize Selected Text" button is disabled with a tooltip: "Please select text from a single chapter."

- **How does translation handle mixed content (e.g., code blocks, equations)?**
  Code blocks and mathematical equations remain in their original form (no translation). Inline code terms (e.g., "ROS 2 node") are preserved in English. Comments within code blocks are translated to Urdu.

- **What happens if onboarding data is incomplete or contradictory?**
  If a user selects "Advanced Programming" but "No Software Background", the system flags this as potentially contradictory and asks a confirmation question: "You selected advanced programming with no software background - is this correct?" Defaults to the most conservative interpretation (beginner) if conflict remains unresolved.

- **How does the system handle chatbot responses when the user's internet connection is unstable?**
  Messages are queued locally with a "Sending..." indicator. If a response takes > 15 seconds, show "Connection slow - still processing...". If > 45 seconds, show "Connection timeout - your message has been saved and will be sent when connection improves." Retry automatically when connection restores.

- **What happens if a user tries to access chat history from a deleted account?**
  All conversation history is permanently deleted when an account is deleted (GDPR compliance). Before deletion, users receive a warning: "Deleting your account will permanently remove all chat history and personalized content. This cannot be undone."

## Requirements *(mandatory)*

### Functional Requirements

#### Core RAG & Chatbot

- **FR-001**: System MUST embed a chatbot interface on every textbook chapter page that can be opened, minimized, and closed without leaving the chapter.

- **FR-002**: System MUST retrieve relevant passages from the book content using vector similarity search (RAG) before generating any chatbot response.

- **FR-003**: Chatbot MUST cite sources for every answer, displaying the chapter, section title, and page reference for retrieved passages.

- **FR-004**: Chatbot MUST refuse to answer questions when no relevant content is found in the book, responding with "I don't know based on the book" rather than hallucinating.

- **FR-005**: System MUST maintain conversation context within a single chat session, allowing users to ask follow-up questions that reference earlier messages.

- **FR-006**: System MUST support multiple specialized agent types: RAG Retrieval Agent, Summarization Agent, Text Selection Agent, Urdu Translation Agent, Personalization Agent, and Chapter Assistant Agent.

#### User Authentication & Profiles

- **FR-007**: System MUST provide sign-up and login functionality that collects user credentials securely.

- **FR-008**: System MUST present new users with an onboarding questionnaire collecting: hardware background, software background, robotics experience, and programming level (each with options: None, Beginner, Intermediate, Advanced, or equivalent scale).

- **FR-009**: System MUST store user profile data persistently and associate it with their account for future personalization.

- **FR-010**: Users MUST be able to view and update their profile information at any time from account settings.

- **FR-011**: System MUST support guest access, allowing unauthenticated users to read chapters without chatbot/personalization features.

#### Personalization Features

- **FR-012**: System MUST display "Personalize this chapter" and "Translate in Urdu" buttons at the beginning of each chapter for logged-in users.

- **FR-013**: When a user clicks "Personalize this chapter", the system MUST regenerate chapter content using the user's profile data (background, experience level) to tailor explanations, examples, and terminology.

- **FR-014**: Personalized content MUST preserve the original chapter structure (headings, sections, diagrams) while adapting narrative text and explanations.

- **FR-015**: Users MUST be able to toggle between personalized and original chapter content with a "View Original" / "View Personalized" switch.

#### Translation Features

- **FR-016**: When a user clicks "Translate in Urdu", the system MUST translate all narrative text to Urdu while preserving technical terms in English or providing English transliterations.

- **FR-017**: Translation MUST preserve the chapter's formatting, structure, code blocks, equations, and diagrams without modification.

- **FR-018**: Users MUST be able to switch between translated and original content with a language toggle.

#### Text Selection & Summarization

- **FR-019**: System MUST allow users to select text within a chapter and trigger a "Summarize Selected Text" action that opens the chatbot with a summary.

- **FR-020**: Summarization MUST only use the selected text as context (not the entire chapter) and provide a concise 2-3 sentence summary.

- **FR-021**: Text selection must be limited to content within a single chapter (no cross-chapter selection).

#### Conversation History

- **FR-022**: System MUST store all chatbot conversations persistently, associated with the user's account and the specific chapter context.

- **FR-023**: Users MUST be able to view conversation history grouped by chapter and sorted by date/time.

- **FR-024**: Users MUST be able to continue previous conversations from history, loading the full context.

- **FR-025**: Users MUST be able to search their conversation history by keyword.

#### Backend API Requirements

- **FR-026**: System MUST expose API endpoints for: `/ingest-book` (ingesting book content into vector database), `/retrieve` (RAG retrieval), `/chat` (chatbot interaction), `/summarize` (text summarization), `/select-text` (handle text selection), `/history` (conversation history), `/translation/urdu` (Urdu translation).

- **FR-027**: System MUST expose API endpoints for: `/personalize-chapter` (generate personalized content) and user profile management (create, read, update profile).

- **FR-028**: All API endpoints requiring user context MUST authenticate requests and validate user session tokens.

#### Data Storage & Persistence

- **FR-029**: System MUST store all book content chunks in a vector database with embeddings for semantic search.

- **FR-030**: System MUST store user profiles, conversation history, and session data in a relational database.

- **FR-031**: Each conversation MUST be assigned a unique conversation ID linking messages to the user and chapter context.

- **FR-032**: System MUST support storing and retrieving retrieved passage sources (chapter, section, page) for citation in chatbot responses.

#### Agent Architecture

- **FR-033**: Each specialized agent (RAG, Summarization, Translation, Personalization, etc.) MUST have isolated tools and skills, preventing hallucination outside its designated scope.

- **FR-034**: All agents MUST call the retrieval tool FIRST before generating responses and are prohibited from using knowledge outside retrieved book content.

### Key Entities *(include if feature involves data)*

- **User Profile**: Represents a student account with authentication credentials, background information (hardware/software/robotics experience, programming level), preferences, and account metadata (created date, last login).

- **Conversation**: Represents a chat session between a user and the chatbot, associated with a specific chapter. Contains conversation ID, user ID, chapter reference, timestamp, and a list of messages.

- **Message**: Represents a single message in a conversation. Contains message ID, conversation ID, role (user or assistant), content, retrieved context (sources/citations), timestamp, and agent metadata (which agent generated the response, tool calls made).

- **Book Content Chunk**: Represents a semantically meaningful segment of book content stored in the vector database. Contains chunk ID, chapter reference, section title, page number, content text, embedding vector, and metadata (tags, keywords, entity types).

- **Chapter**: Represents a chapter in the textbook. Contains chapter ID, title, original content (markdown/HTML), personalized versions (per user), translated versions (Urdu), and section structure.

- **Retrieved Context**: Represents passages retrieved from the vector database for a specific chatbot query. Contains chunk references, similarity scores, source metadata (chapter, section, page), and ranking.

- **Personalization Request**: Represents a request to generate personalized chapter content. Contains user ID, chapter ID, user profile snapshot (background data at request time), and generated personalized content.

- **Translation Request**: Represents a request to translate chapter content to Urdu. Contains chapter ID, original content reference, translated content, and translation metadata (technical terms preserved, timestamp).

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can ask a question in the chatbot and receive an accurate, source-cited answer within 3 seconds for 95% of queries.

- **SC-002**: Chatbot maintains 98% accuracy in refusing to answer questions outside the book content (measured by zero hallucinated responses in test scenarios with out-of-scope questions).

- **SC-003**: Personalized chapter content is generated and displayed within 30 seconds of clicking "Personalize this chapter".

- **SC-004**: Urdu translation of a chapter completes within 45 seconds and preserves all technical terms in English or transliterated format.

- **SC-005**: Users can complete the onboarding questionnaire and create a profile in under 2 minutes.

- **SC-006**: Conversation history loads within 2 seconds and allows users to continue previous conversations without context loss.

- **SC-007**: Text summarization generates a concise summary (2-3 sentences) within 3 seconds of selection.

- **SC-008**: 90% of student questions receive relevant answers on the first attempt (measured by user satisfaction ratings or "Was this helpful?" feedback).

- **SC-009**: Personalized content demonstrates measurable improvement in comprehension: students with "software background" profiles can correctly explain hardware concepts using software analogies in post-reading quizzes.

- **SC-010**: System supports at least 100 concurrent users interacting with the chatbot without response time degradation beyond SC-001 threshold (3 seconds).

- **SC-011**: Zero data loss: All conversations and profile data persist correctly across sessions with 100% recovery rate after system restarts.

- **SC-012**: Chatbot correctly cites sources for 100% of answers, displaying accurate chapter/section/page references.

## Assumptions

1. **Existing Book Content**: The Physical AI & Humanoid Robotics textbook is already published as a Docusaurus website with structured markdown content organized by chapters and sections.

2. **Content Format**: Book content is available in a format suitable for chunking (markdown, HTML, or plain text) with clear section delineations and page/chapter metadata.

3. **Vector Database**: Qdrant Cloud Free Tier provides sufficient storage and performance for the book's content volume (estimated < 10,000 chunks at 512-1024 tokens each).

4. **LLM Access**: Gemini API (via OpenAI Agents SDK) is accessible with acceptable rate limits and response times for real-time chatbot interactions.

5. **Translation Quality**: Urdu translation via Gemini maintains technical accuracy and readability for the target audience (Urdu-speaking robotics students).

6. **User Base Scale**: Initial user base will be < 1,000 concurrent users, allowing free-tier or low-cost database hosting (Neon serverless Postgres).

7. **Authentication Requirements**: Standard email/password authentication is sufficient; OAuth/SSO integration can be added later if needed.

8. **Browser Compatibility**: Users access the textbook via modern web browsers (Chrome, Firefox, Safari, Edge) with JavaScript enabled. Mobile browsers are supported but with responsive design considerations.

9. **Network Connectivity**: Users have stable internet connections capable of streaming chatbot responses and loading chapter content without frequent timeouts.

10. **Content Updates**: Book content updates are infrequent (e.g., monthly or quarterly), allowing manual re-ingestion into the vector database. Automated content sync is not required for MVP.

11. **Personalization Scope**: Personalization focuses on explanatory text and examples, not structural changes (no adding/removing sections, just adapting narrative style).

12. **Agent Isolation**: OpenAI Agents SDK supports creating isolated subagents with distinct tool sets, preventing cross-agent hallucination.

13. **Data Privacy**: Storing user conversations and profile data complies with basic privacy expectations (GDPR-style data deletion on account removal). No specific legal compliance requirements beyond standard best practices.

14. **Performance Benchmarks**: Response time targets (SC-001: 3 seconds for chatbot, SC-003: 30 seconds for personalization) are achievable with current LLM API latencies and vector database query speeds.

15. **UI Integration**: ChatKit (chatkit-js) can be embedded in Docusaurus pages without major conflicts with existing CSS/JavaScript frameworks.

## Out of Scope

1. **Real-time Collaboration**: Multi-user collaborative features (e.g., sharing conversations between users, group study sessions) are not included in this specification.

2. **Voice/Audio Interactions**: Chatbot is text-only; voice input/output and audio transcription are excluded.

3. **Offline Mode**: All features require active internet connection; offline caching or progressive web app (PWA) capabilities are not supported.

4. **Automated Content Ingestion**: Book content updates must be manually triggered via `/ingest-book` endpoint; automated syncing from the Docusaurus repository is not implemented.

5. **Multi-language Translation Beyond Urdu**: Only Urdu translation is supported initially; additional languages (Arabic, Hindi, Spanish, etc.) are excluded.

6. **Advanced Analytics Dashboard**: No admin dashboard for tracking chatbot usage, user engagement metrics, or conversation analytics. Basic logging for debugging is included, but business intelligence features are out of scope.

7. **Content Editing via Chatbot**: Users cannot edit or annotate chapter content through the chatbot; all content modifications must go through the standard Docusaurus editing workflow.

8. **Peer-to-Peer Learning Features**: No forums, discussion boards, or student-to-student messaging within the textbook platform.

9. **Gamification**: No badges, points, leaderboards, or progress tracking features. Learning is self-paced without competitive elements.

10. **Integration with External Learning Management Systems (LMS)**: No SCORM compliance, LTI integration, or grade syncing with platforms like Canvas, Moodle, or Blackboard.

11. **Custom Agent Creation by Users**: Users cannot create their own specialized agents or train custom models; only predefined agents (RAG, Summarization, Translation, etc.) are available.

12. **Video/Image Analysis**: Chatbot cannot analyze diagrams, images, or video content within chapters; it only processes text.

13. **Code Execution Environment**: No live coding sandbox or Jupyter notebook integration for running code examples from the textbook.

14. **Accessibility Features Beyond Standard Compliance**: Basic screen reader support via semantic HTML is expected, but advanced accessibility features (e.g., dyslexia-friendly fonts, text-to-speech for chatbot responses) are not prioritized.

15. **A/B Testing Framework**: No built-in experimentation platform for testing different personalization strategies or chatbot response formats.

## Dependencies

1. **Existing Textbook Infrastructure**: Requires the current Docusaurus website codebase, deployment pipeline, and hosting environment to be accessible and modifiable.

2. **Qdrant Cloud Account**: Active Qdrant Cloud account with API credentials for vector database operations.

3. **Neon Database Account**: Active Neon serverless Postgres account with database created and connection credentials available.

4. **Gemini API Access**: Valid API key for Google Gemini (via OpenAI Agents SDK) with sufficient quota for expected usage volume.

5. **Better-Auth Integration**: Better-Auth library and configuration for handling user authentication flows.

6. **ChatKit Library**: ChatKit (chatkit-js) npm package or equivalent frontend chat UI component.

7. **OpenAI Agents SDK**: Installed and configured SDK for creating specialized subagents and managing agent tools.

8. **FastAPI Framework**: Python FastAPI framework for building backend API endpoints.

9. **Existing RAG Code**: Access to the provided agent.py, retrieval.py, and ingestion pipeline code that must be integrated into the new system.

10. **SSL/HTTPS Configuration**: Secure HTTPS hosting for both frontend (Docusaurus) and backend (FastAPI) to protect authentication tokens and user data.

11. **CORS Configuration**: Backend API must correctly handle cross-origin requests from the Docusaurus frontend domain.

12. **Environment Variables**: Secure storage for API keys, database credentials, and JWT secrets (e.g., .env files, secret management service).

## Risks

1. **LLM Hallucination**: Despite enforcing retrieval-first design, LLMs may still hallucinate when context is ambiguous. Mitigation: Strict prompt engineering, post-processing to verify citations, and user feedback mechanisms to flag incorrect answers.

2. **Translation Accuracy**: Urdu translation may produce technically inaccurate or awkward phrasing for specialized robotics terminology. Mitigation: Human review of critical chapters, maintain glossary of verified Urdu translations for key terms, allow users to report translation issues.

3. **Performance Degradation at Scale**: Concurrent chatbot requests may overwhelm backend API or vector database, causing timeouts. Mitigation: Load testing before launch, implement request queuing, use caching for frequently asked questions, consider rate limiting per user.

4. **Data Privacy Violations**: Storing conversation history and profiles creates privacy obligations. Risk of data breaches or non-compliance. Mitigation: Encrypt sensitive data at rest, implement secure session management, provide clear privacy policy, support account deletion with full data purge.

5. **Poor Personalization Quality**: User background data may be insufficient to generate meaningfully personalized content, leading to minimal perceived value. Mitigation: Iterate on personalization prompts based on user feedback, A/B test different personalization strategies post-launch, allow users to rate personalized content usefulness.

6. **ChatKit Integration Conflicts**: Embedding ChatKit in Docusaurus may cause CSS/JavaScript conflicts or layout issues. Mitigation: Test on multiple browsers and devices, use CSS scoping or shadow DOM to isolate ChatKit styles, provide fallback UI if ChatKit fails to load.

7. **Inaccurate Source Citations**: Chatbot may cite incorrect chapters/sections if retrieval metadata is malformed. Mitigation: Validate ingestion pipeline outputs, manually verify citations for a sample of chapters, allow users to report citation errors.

8. **User Onboarding Abandonment**: Lengthy onboarding questionnaire may cause users to abandon signup. Mitigation: Make onboarding optional (allow skipping with defaults), keep questionnaire to < 5 questions, visually indicate progress during onboarding.

9. **Context Window Overflow**: Long conversations may exceed LLM context window limits, breaking conversation continuity. Mitigation: Implement conversation summarization after N messages, truncate oldest messages while preserving recent context, notify users when context is summarized.

10. **Dependency on Third-Party APIs**: Reliance on Gemini API, Qdrant Cloud, and Neon creates availability risks. Mitigation: Implement graceful degradation (e.g., show cached responses if API is down), monitor API status, have fallback plans for critical failures.
