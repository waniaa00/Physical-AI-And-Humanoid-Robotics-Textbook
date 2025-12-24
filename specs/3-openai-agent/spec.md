# Feature Specification: OpenAI Agents SDK RAG Agent

**Feature Branch**: `3-openai-agent`
**Created**: 2025-12-15
**Status**: Draft
**Input**: User description: "Build an AI agent using OpenAI Agents SDK with retrieval-augmented answering. Target purpose: Create an AI agent that uses the OpenAI Agents SDK to answer user questions by invoking the retrieval pipeline and grounding responses strictly in book content. Focus: Agent orchestration, tool calling, and controlled response generation using retrieved context from the vector database. Success criteria: Agent successfully calls retrieval tool for every user query, responses are grounded only in retrieved book chunks, agent returns answers with cited source metadata, supports answering based on full-book context, supports answering based only on user-selected text"

## User Scenarios & Testing

### User Story 1 - Full-Book Question Answering (Priority: P1)

A user asks a question about humanoid robotics (e.g., "How does inverse kinematics work for humanoid robots?") and the agent searches the entire book content, retrieves relevant passages, and provides a comprehensive answer grounded strictly in the retrieved content with source citations.

**Why this priority**: Core RAG agent functionality - connects retrieval pipeline to intelligent response generation. This is the minimum viable feature that delivers end-to-end question answering capability.

**Independent Test**: Submit question "What is inverse kinematics?" and verify agent (1) calls retrieval tool, (2) returns answer based on retrieved chunks, (3) includes citations with URLs and chapter references.

**Acceptance Scenarios**:

1. **Given** user asks "How does inverse kinematics work?", **When** agent processes query, **Then** agent calls retrieval tool with the query, receives relevant chunks, and generates answer grounded in retrieved content
2. **Given** agent receives question, **When** generating response, **Then** answer includes inline citations referencing specific book sections (e.g., "[Chapter 3: Kinematics]")
3. **Given** agent generates answer, **When** response is complete, **Then** agent includes source metadata listing all referenced URLs and chapter titles

---

### User Story 2 - Context-Constrained Answering (Priority: P2)

A user selects specific text from the book and asks a question about it (e.g., selects a paragraph about ZMP and asks "Can you explain this in simpler terms?"). The agent answers based only on the provided text context, not searching the full book.

**Why this priority**: Enables targeted explanations and clarifications about specific passages. Important for learning scenarios where users want to understand particular sections deeply.

**Independent Test**: Provide selected text about a specific topic and ask follow-up question. Verify agent answers using only the provided context without calling retrieval tool.

**Acceptance Scenarios**:

1. **Given** user provides selected text and asks question, **When** agent processes request, **Then** agent does NOT call retrieval tool and instead uses only the provided context
2. **Given** user asks "Explain this concept" with provided text, **When** agent generates answer, **Then** response references and explains the specific content from the provided text
3. **Given** user provides text about Topic A but asks about Topic B, **When** agent responds, **Then** agent clarifies it can only answer based on provided context or suggests broadening to full-book search

---

### User Story 3 - Multi-Turn Conversational Context (Priority: P2)

A user engages in multi-turn conversation with the agent, asking follow-up questions that reference previous exchanges. The agent maintains conversation context while continuing to ground responses in book content through retrieval.

**Why this priority**: Natural conversation flow improves user experience. Users often ask clarifying questions or dive deeper into topics, requiring the agent to maintain context across turns.

**Independent Test**: Start conversation with initial question, then ask 2-3 follow-up questions using pronouns ("it", "this", "that"). Verify agent understands references and maintains coherent conversation.

**Acceptance Scenarios**:

1. **Given** user asks initial question and receives answer, **When** user asks follow-up with pronoun reference ("Can you explain that further?"), **Then** agent understands "that" refers to previous answer topic and retrieves additional relevant content
2. **Given** multi-turn conversation, **When** agent generates responses, **Then** each answer maintains context from previous turns while grounding in retrieved book content
3. **Given** conversation context becomes unclear, **When** agent cannot resolve reference, **Then** agent asks clarification question before retrieving content

---

### User Story 4 - Citation and Source Transparency (Priority: P1)

For every answer provided, the agent clearly cites sources by including chapter titles, section names, and URLs so users can verify information and explore topics further in the original book.

**Why this priority**: Critical for trust and verifiability. Users must be able to trace answers back to source material, especially for technical educational content.

**Independent Test**: Ask any question and verify response includes: (1) inline citations in answer text, (2) structured source list at end with URLs, (3) chapter/section identifiers.

**Acceptance Scenarios**:

1. **Given** agent generates answer using multiple retrieved chunks, **When** response is formatted, **Then** answer includes inline citations like "[1]" or "[Chapter 3]" linked to specific sources
2. **Given** agent completes response, **When** formatting source list, **Then** list includes: chunk URL, chapter title, relevance score or ranking
3. **Given** agent answers question, **When** no relevant content found in retrieval, **Then** agent explicitly states "I don't have information about this in the book" rather than generating unsourced content

---

### User Story 5 - Retrieval Quality Validation (Priority: P3)

System administrators can validate that the agent consistently calls the retrieval tool and grounds responses appropriately through test queries with known expected behaviors.

**Why this priority**: Important for quality assurance but not essential for end-user functionality. Ensures agent maintains expected behavior over time.

**Independent Test**: Run test suite with 10 predefined questions. Verify each triggers retrieval, generates grounded response, and includes citations.

**Acceptance Scenarios**:

1. **Given** test suite of 10 questions, **When** agent processes each, **Then** 100% of responses include retrieval tool calls (no hallucinated answers)
2. **Given** test questions with known answers in book, **When** validating responses, **Then** agent answers align with actual book content (no contradictions)
3. **Given** test question about content not in book, **When** agent responds, **Then** agent states content is not available rather than generating answer

---

### Edge Cases

- What happens when retrieval returns no results (query topic not covered in book)?
- How does agent handle ambiguous questions that could refer to multiple topics?
- What happens when user provides very long selected text (>5000 tokens)?
- How does agent behave with off-topic questions unrelated to humanoid robotics?
- What happens when retrieval fails due to API errors?
- How does agent handle questions requiring comparison across multiple book sections?
- What happens when conversation context exceeds token limits?
- How does agent handle requests to "summarize the entire book" or very broad queries?

## Requirements

### Functional Requirements

- **FR-001**: System MUST use OpenAI Agents SDK to orchestrate agent interactions
- **FR-002**: Agent MUST define retrieval tool that calls backend `/search` endpoint
- **FR-003**: Agent MUST call retrieval tool for every user question in full-book mode
- **FR-004**: Agent MUST NOT call retrieval tool when user provides selected text context
- **FR-005**: Agent MUST generate responses grounded strictly in retrieved chunks (no hallucination)
- **FR-006**: Agent MUST include inline citations in generated responses (e.g., [Chapter 3], [1])
- **FR-007**: Agent MUST provide structured source list at end of response with URLs and chapter titles
- **FR-008**: Agent MUST support two modes: full-book search and context-constrained answering
- **FR-009**: Agent MUST maintain conversation history across multiple turns
- **FR-010**: Agent MUST handle retrieval failures gracefully with appropriate error messages
- **FR-011**: System MUST provide agent endpoint accepting user queries and optional context
- **FR-012**: Agent MUST refuse to answer questions when no relevant content is retrieved (threshold: all scores <0.4)

### Key Entities

- **Agent**: OpenAI Agents SDK agent instance configured with retrieval tool, system instructions for grounding, and response formatting guidelines. Orchestrates tool calling and response generation.

- **Retrieval Tool**: Function tool definition that wraps backend `/search` endpoint. Accepts query parameter, returns list of relevant chunks with text, scores, and metadata.

- **User Query**: Natural language question from user about humanoid robotics concepts. Contains query text and optional mode flag (full-book vs context-constrained).

- **Retrieved Context**: Collection of book chunks returned from retrieval tool. Contains chunk text, similarity scores, metadata (URL, chapter, chunk_index). Used as grounding for response generation.

- **Agent Response**: Generated answer to user query. Includes main answer text with inline citations, structured source list, and conversation context for multi-turn interactions.

- **Conversation Thread**: Sequence of user queries and agent responses in a session. Maintains context across multiple turns for coherent follow-up questions.

- **Selected Text Context**: User-provided book passage for context-constrained answering. Replaces retrieval tool call when present.

## Success Criteria

### Measurable Outcomes

- **SC-001**: Agent calls retrieval tool for 100% of full-book queries (validated through logging)
- **SC-002**: Agent responses include citations for 100% of answers (inline and source list)
- **SC-003**: Test suite of 10 questions achieves 100% grounded responses (no hallucinations detected by human review)
- **SC-004**: Agent correctly distinguishes between full-book and context-constrained modes with 100% accuracy
- **SC-005**: Multi-turn conversations maintain context for at least 5 consecutive turns without losing coherence
- **SC-006**: Agent refuses to answer (states "not in book") for 100% of queries with all retrieval scores <0.4
- **SC-007**: Agent response latency stays under 10 seconds for typical questions (single retrieval call)
- **SC-008**: Users can trace every factual claim in agent response to specific source URLs (100% verifiability)

## Assumptions

- OpenAI Agents SDK is available and compatible with Python 3.13+
- Backend retrieval API (`/search` endpoint) is operational and returns results in expected format
- OpenAI API access is available with sufficient rate limits for conversational usage
- Users will primarily ask questions in English matching book content language
- "Grounded response" means response content derives strictly from retrieved chunks without adding external knowledge
- Inline citations can use simple bracketed references ([Chapter X]) rather than formal citation formats
- Selected text context will typically be under 2000 words (within reasonable context window)
- Conversation history will be maintained in-memory for session duration (no persistent storage required initially)
- Acceptable latency for multi-turn conversations includes retrieval time (10s total is acceptable)
- Agent will use OpenAI GPT-4 or later model with sufficient context window for retrieved chunks

## Dependencies

- OpenAI Agents SDK must be installed and configured
- OpenAI API access with valid API key
- Backend retrieval API (feature 2-rag-retrieval) must be deployed and accessible
- Environment variables for OpenAI API key and retrieval endpoint URL

## Scope Boundaries

### In Scope
- Agent creation using OpenAI Agents SDK
- Retrieval tool definition wrapping `/search` endpoint
- System instructions for response grounding and citation formatting
- Full-book search mode with automatic retrieval
- Context-constrained mode using provided text
- Multi-turn conversation handling
- Inline citations and source list formatting
- Graceful error handling for retrieval failures
- Refusal to answer when content not in book

### Out of Scope
- Voice/audio interaction (text-only)
- Image or diagram understanding (text content only)
- Code execution or interactive examples
- Real-time book content updates (uses static vector database)
- Multi-language support beyond English
- User authentication or session persistence
- Advanced citation formats (APA, MLA, etc.)
- Answer quality rating or feedback collection
- Integration with external knowledge sources beyond the book
- Automated fact-checking or answer validation
- Summary generation for entire book chapters
- Question suggestion or query auto-completion
