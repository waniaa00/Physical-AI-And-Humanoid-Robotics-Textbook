---
id: 6
title: "RAG Chatbot + BetterAuth Personalization Planning"
stage: "plan"
date: "2025-12-10"
model: "Claude Sonnet 4.5"
feature: "rag-chatbot-personalization"
branch: "004-rag-chatbot-personalization"
user: "HP"
command: "/sp.plan"
labels: ["planning", "architecture", "chatbot", "personalization"]
links:
  spec: "specs/004-rag-chatbot-personalization/spec.md"
  plan: "specs/004-rag-chatbot-personalization/plan.md"
  ticket: null
  adr: null
files:
  - "specs/004-rag-chatbot-personalization/plan.md"
tests: []
---

# RAG Chatbot + BetterAuth Personalization Planning

## Prompt Text

### Title

RAG Chatbot + BetterAuth + ChatKit UI + UV Backend

## Phase 0 — Project reset

- confirm single repository (docusaurus existing)
- backend folder created: /backend
- initialize UV python project:
    - uv init
    - python 3.12
- install FastAPI + Agents + Qdrant + Cohere + BetterAuth + Neon

## Phase 1 — FastAPI Backend Scaffolding
### steps

- create /backend/src
- create /backend/src/main.py
- add routes:
- /auth/signup
- /auth/signin
- /profile
- /chat
- /summarize
- /select-text
- /personalize/:chapter
- /translate/:chapter
- /history
- mount BetterAuth
- mount JSON logging

### success criteria

- FastAPI runs locally with UV
- BetterAuth available
- no business logic yet

## Phase 2 — Database Models + Neon Integration
### steps

- install asyncpg
- configure NEON_URL
- create db folder:
`/backend/src/db`
- create models:
- users
- sessions
- chat_history
- user_preferences
- add async session factory
- add migrations (simple raw SQL ok)

## Phase 3 — Embeddings + Qdrant
### steps

- add qdrant client
- create collection:
    - name: humanoid_robotics_textbook
    - dimension: 1024
    - distance: cosine
- connect using QDRANT_URL/QDRANT_KEY
- test health

### tool functions

- `embed_text()`
- `upsert_chunks()`
- `search_chunks()`

### success criteria

- test insert + search locally

## Phase 4 — Ingestion
### steps

- reuse reference ingestion
- only replace sitemap
- use Cohere:
    - model="embed-english-v3.0"
- chunk markdown pages
- upsert into qdrant

### success criteria
- all chapters indexed

## Phase 5 — Gemini + Agents + Tool Calling
### steps

- install openai-agents-python
- create providers:
```provider = AsyncOpenAI(
  api_key=gemini_api_key,
  base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)
model = OpenAIChatCompletionsModel(
  model="gemini-2.0-flash",
  openai_client=provider
)```


- from agents import Agent
- define agents + tools:
    - retrieve()
    - summarize_selection()
    - translate_urdu()
    - personalize_chapter()
    - track_history()

## Phase 6 — Subagents Definition
### subagents

- rag_retrieval_agent
- summarization_agent
- selection_agent
- translation_agent
- personalization_agent
- history_agent

### skills bound per tool:

- retrieve → qdrant
- summarize → gemini
- translate → gemini
- personalize → gemini
- track → neon

## Phase 7 — Chat Route Logic
### steps

- POST /chat
- tool=predict()
- RAG flow:
    - retrieve()
    - answer only from retrieved
- fallback:
    - "I don't know"
- store history entry

## Phase 8 — Personalization Flow
### steps

- on /personalize/:chapter
- retrieve original chapter text
- call personalization_agent
- store personalized copy in DB
- return modified content

## Phase 9 — Urdu Translation Flow
### steps

- /translate/:chapter
- call translation_agent
- return Urdu text
- mark RTL

## Phase 10 — Summarize Selection Flow
### steps

- user highlight → frontend passes text
- POST /select-text
- call summarization_agent
- return short summary
- display inside chat
-
## Phase 11 — Chat History
### steps

- /history
- read neon table
- return full history
- ChatKit loads history tab view

## Phase 12 — ChatKit JS UI
### steps

- import chatkit-js
- floating chat
- bottom-right position
- add:
    - Messages
    - Source URLs
    - History tab
- attach session token from BetterAuth

### success criteria

- chat box on every page

## Phase 13 — Docusaurus integration
### steps

- inject script globally using layout file
- add chapter buttons:
    - Personalize Chapter
    - Translate To Urdu

- DOM replace chapter content

## Phase 14 — Tool-calling enforcement
### ensure:

- retrieval uses only retrieve()
- all generation tasks use subagents
- direct chat cannot bypass tools
- "unknown → I don't know" enforced
-
## Phase 15 — Deployment
### Backend

- Railway / Vercel functions
- env:

    - QDRANT_URL
    - QDRANT_KEY
    - COHERE_KEY
    - NEON_URL
    - GEMINI_API_KEY
    - BETTERAUTH_SECRET

### Frontend

- Vercel on Docusaurus

## Response Text

Created a comprehensive implementation plan with 15 phases:

1. Project Reset - Set up UV Python project
2. FastAPI Backend Scaffolding - Create basic structure and routes
3. Database Models + Neon Integration - Set up database layer
4. Embeddings + Qdrant - Configure vector store
5. Ingestion - Index content from sitemap
6. Gemini + Agents + Tool Calling - Integrate AI and tool framework
7. Subagents Definition - Create specialized agents
8. Chat Route Logic - Implement main chat functionality
9. Personalization Flow - Implement content personalization
10. Urdu Translation Flow - Implement translation with RTL support
11. Summarize Selection Flow - Implement text selection summarization
12. Chat History - Implement history persistence
13. ChatKit JS UI - Create floating chat interface
14. Docusaurus Integration - Integrate with existing book
15. Deployment - Deploy to production

Each phase includes specific tasks, objectives, and success criteria. The plan covers all technical requirements including FastAPI, UV, BetterAuth, Qdrant, Cohere, Gemini, Neon, and ChatKit integration.

## Outcome

Successfully created a detailed implementation plan that aligns with all requirements from the user's input. The plan is structured in phases with clear tasks and success criteria for each phase. It covers the complete implementation from project setup through deployment, including all technical components and integration points.