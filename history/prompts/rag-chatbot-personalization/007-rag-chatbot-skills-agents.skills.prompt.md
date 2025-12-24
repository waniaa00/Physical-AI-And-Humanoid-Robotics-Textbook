---
id: 7
title: "RAG Chatbot Skills and Agents Implementation"
stage: "tasks"
date: "2025-12-10"
model: "Claude Sonnet 4.5"
feature: "rag-chatbot-personalization"
branch: "004-rag-chatbot-personalization"
user: "HP"
command: "/sp.skills"
labels: ["skills", "agents", "implementation", "chatbot"]
links:
  spec: "specs/004-rag-chatbot-personalization/spec.md"
  plan: "specs/004-rag-chatbot-personalization/plan.md"
  ticket: null
  adr: null
files:
  - "backend/src/skills/__init__.py"
  - "backend/src/skills/retrieve.py"
  - "backend/src/skills/summarize_selection.py"
  - "backend/src/skills/translate_urdu.py"
  - "backend/src/skills/personalize_chapter.py"
  - "backend/src/skills/track_history.py"
  - "backend/src/skills/main.py"
  - "backend/src/agents/__init__.py"
  - "backend/src/agents/rag_retrieval_agent.py"
  - "backend/src/agents/summarization_agent.py"
  - "backend/src/agents/translation_agent.py"
  - "backend/src/agents/personalization_agent.py"
  - "backend/src/agents/history_agent.py"
  - "backend/src/agents/main.py"
tests: []
---

# RAG Chatbot Skills and Agents Implementation

## Prompt Text

## Global Agent Context
```
provider: AsyncOpenAI(
  api_key = GEMINI_API_KEY,
  base_url = "https://generativelanguage.googleapis.com/v1beta/openai/"
)

model: OpenAIChatCompletionsModel(
  model = "gemini-2.0-flash",
  openai_client = provider
)
```

## Skills
### retrieve
#### scope

Query Qdrant collection (humanoid_robotics_textbook) using Cohere embedding and return top-k chunks.

### input
```
{
  "query": "string",
  "limit": 5
}
```

### output
```
{
  "chunks": [
    { "text": "..." , "url": "..." }
  ]
}
```

### implementation

- cohere.embed → qdrant.search
- return ranked chunks
- never generate text here

### agent

`rag_retrieval_agent`

## summarize_selection
### purpose

Summarize highlighted text from page.

### input
```
{
  "selection": "string"
}
```

### output
```
{
  "summary": "string"
}
```

### agent

`summarization_agent`

### implementation

- gemini chat
- short form
- no external hallucination allowed
- return summary only

## translate_urdu
### purpose

Convert chapter text to Urdu and mark RTL.

### input
```
{
  "text": "string"
}
```

### output
```
{
  "urdu": "string",
  "rtl": true
}
```

### agent

`translation_agent`

### implementation
- gemini
- formal Urdu
- preserve code block
- return plain text

## personalize_chapter
### purpose

Tailor chapter according to hardware/software profile.

### input
```
{
  "chapter_id": "string",
  "user_profile": {
      "hardware": "string",
      "software": "string"
  }
}
```

### output
```
{
  "personalized": "string"
}
```
### agent

`personalization_agent`

### implementation

- gemini
- detect missing profile → request update
- return modified content

## track_history
### purpose

Store message pair in Neon DB + return updated history

### input
```{
  "user_id": "string",
  "message": "string",
  "response": "string"
}```

### output
```{
  "history": [
    { "message": "...", "response": "..." }
  ]
}```

### agent

`history_agent`

### implementation

- neon insert
- neon select recent messages

## Skill Bind

## Response Text

Successfully implemented all required skills and agents for the RAG Chatbot:

1. Created individual skill modules for each required functionality:
   - retrieve: Queries Qdrant collection using Cohere embeddings
   - summarize_selection: Summarizes highlighted text using Gemini
   - translate_urdu: Translates text to Urdu with RTL support
   - personalize_chapter: Adapts content based on user profile
   - track_history: Manages chat history in Neon DB

2. Created individual agent modules that wrap each skill:
   - rag_retrieval_agent
   - summarization_agent
   - translation_agent
   - personalization_agent
   - history_agent

3. Implemented proper error handling with decorators
4. Created main modules to combine all skills and agents
5. Used proper async/await patterns throughout
6. Integrated with required services (Qdrant, Cohere, Gemini, Neon)

## Outcome

All required skills and agents have been implemented according to specifications. The implementation follows the proper architecture with skills handling specific functionality and agents orchestrating the skills. Each skill has proper error handling and returns the expected output format.