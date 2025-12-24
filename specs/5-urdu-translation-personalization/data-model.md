# Data Model: Urdu Translation, Summarization, and Personalization

**Feature**: `5-urdu-translation-personalization`
**Date**: 2025-12-18
**Status**: Design

## Overview

This document defines the data structures, database schema, and API contracts for the translation, summarization, and personalization features. All data transformations (translation, summarization) are ephemeral and not persisted. Only user interest preferences are stored in the database.

## Database Schema

### Neon PostgreSQL Tables

#### 1. user_profiles

Stores user profile information including background level.

```sql
CREATE TABLE user_profiles (
    user_id UUID PRIMARY KEY,
    background VARCHAR(20) NOT NULL CHECK (background IN ('student', 'professional')),
    language_preference VARCHAR(10) DEFAULT 'en',
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT fk_user_profiles_user_id FOREIGN KEY (user_id) REFERENCES users(id) ON DELETE CASCADE
);

CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
```

**Notes**:
- `user_id` references better-auth's `users` table
- `background` indicates user's experience level (affects personalization tone)
- `language_preference` for future multi-language support (currently unused)

#### 2. interest_categories

Predefined list of interest categories users can select from.

```sql
CREATE TABLE interest_categories (
    id SERIAL PRIMARY KEY,
    name VARCHAR(100) UNIQUE NOT NULL,
    slug VARCHAR(100) UNIQUE NOT NULL,
    description TEXT,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

-- Seed data for 8 predefined categories
INSERT INTO interest_categories (name, slug, description) VALUES
('Robotics', 'robotics', 'General robotics and automation'),
('AI/ML', 'ai_ml', 'Artificial Intelligence and Machine Learning'),
('Software Engineering', 'software_engineering', 'Software development and architecture'),
('Mechanical Engineering', 'mechanical_engineering', 'Mechanical systems and design'),
('Electrical Engineering', 'electrical_engineering', 'Electronics and circuits'),
('Mathematics', 'mathematics', 'Mathematical foundations and theory'),
('Physics', 'physics', 'Physics principles and applications'),
('Computer Vision', 'computer_vision', 'Vision systems and image processing');
```

#### 3. user_interests

Junction table linking users to their selected interests (2-5 per user).

```sql
CREATE TABLE user_interests (
    id SERIAL PRIMARY KEY,
    user_id UUID NOT NULL,
    interest_id INTEGER NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT fk_user_interests_user_id FOREIGN KEY (user_id) REFERENCES user_profiles(user_id) ON DELETE CASCADE,
    CONSTRAINT fk_user_interests_interest_id FOREIGN KEY (interest_id) REFERENCES interest_categories(id) ON DELETE CASCADE,
    CONSTRAINT uq_user_interests_unique UNIQUE (user_id, interest_id)
);

CREATE INDEX idx_user_interests_user_id ON user_interests(user_id);
CREATE INDEX idx_user_interests_interest_id ON user_interests(interest_id);
```

**Constraints**:
- Each user must have between 2 and 5 interests
- No duplicate interests for a user (enforced by UNIQUE constraint)
- Constraint validation handled at application layer (not database CHECK constraint due to complexity)

### Database Migrations

**Migration 001: Initial Schema**
```sql
-- 001_create_user_profiles_and_interests.sql
BEGIN;

CREATE TABLE IF NOT EXISTS user_profiles (
    user_id UUID PRIMARY KEY,
    background VARCHAR(20) NOT NULL CHECK (background IN ('student', 'professional')),
    language_preference VARCHAR(10) DEFAULT 'en',
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS interest_categories (
    id SERIAL PRIMARY KEY,
    name VARCHAR(100) UNIQUE NOT NULL,
    slug VARCHAR(100) UNIQUE NOT NULL,
    description TEXT,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP
);

CREATE TABLE IF NOT EXISTS user_interests (
    id SERIAL PRIMARY KEY,
    user_id UUID NOT NULL,
    interest_id INTEGER NOT NULL,
    created_at TIMESTAMP NOT NULL DEFAULT CURRENT_TIMESTAMP,
    CONSTRAINT fk_user_interests_user_id FOREIGN KEY (user_id) REFERENCES user_profiles(user_id) ON DELETE CASCADE,
    CONSTRAINT fk_user_interests_interest_id FOREIGN KEY (interest_id) REFERENCES interest_categories(id) ON DELETE CASCADE,
    CONSTRAINT uq_user_interests_unique UNIQUE (user_id, interest_id)
);

CREATE INDEX idx_user_profiles_user_id ON user_profiles(user_id);
CREATE INDEX idx_user_interests_user_id ON user_interests(user_id);
CREATE INDEX idx_user_interests_interest_id ON user_interests(interest_id);

INSERT INTO interest_categories (name, slug, description) VALUES
('Robotics', 'robotics', 'General robotics and automation'),
('AI/ML', 'ai_ml', 'Artificial Intelligence and Machine Learning'),
('Software Engineering', 'software_engineering', 'Software development and architecture'),
('Mechanical Engineering', 'mechanical_engineering', 'Mechanical systems and design'),
('Electrical Engineering', 'electrical_engineering', 'Electronics and circuits'),
('Mathematics', 'mathematics', 'Mathematical foundations and theory'),
('Physics', 'physics', 'Physics principles and applications'),
('Computer Vision', 'computer_vision', 'Vision systems and image processing');

COMMIT;
```

## API Data Models

### Translation Models

**TranslationRequest** (Pydantic):
```python
from pydantic import BaseModel, Field, field_validator

class TranslationRequest(BaseModel):
    text: str = Field(..., min_length=1, max_length=10000, description="Text to translate")
    target_lang: str = Field(default="ur", pattern="^(ur|en)$", description="Target language code")
    preserve_code: bool = Field(default=True, description="Preserve code blocks during translation")
    user_id: Optional[str] = Field(default=None, description="Optional user ID for logging")

    @field_validator('text')
    @classmethod
    def validate_text_length(cls, v):
        word_count = len(v.split())
        if word_count > 5000:
            raise ValueError("Text exceeds maximum 5000 words")
        return v
```

**TranslationResponse**:
```python
class TranslationResponse(BaseModel):
    translated_text: str = Field(..., description="Translated text with preserved formatting")
    original_text: str = Field(..., description="Original input text")
    detected_lang: str = Field(..., description="Detected source language code")
    target_lang: str = Field(..., description="Target language code")
    word_count: int = Field(..., description="Word count of original text")
    translation_time_ms: int = Field(..., description="Translation processing time in milliseconds")
    preserved_elements: Dict[str, int] = Field(default={}, description="Count of preserved code blocks and terms")
```

### Summarization Models

**SummarizationRequest**:
```python
from enum import Enum

class SummaryLength(str, Enum):
    SHORT = "short"      # ~20% of original
    MEDIUM = "medium"    # ~30% of original
    LONG = "long"        # ~40% of original

class SummaryFocus(str, Enum):
    CONCEPTS = "concepts"  # Focus on conceptual explanations
    CODE = "code"          # Include code snippets in summary
    BOTH = "both"          # Balance concepts and code

class SummarizationRequest(BaseModel):
    text: str = Field(..., min_length=50, max_length=10000, description="Text to summarize")
    target_length: SummaryLength = Field(default=SummaryLength.SHORT, description="Desired summary length")
    focus: SummaryFocus = Field(default=SummaryFocus.CONCEPTS, description="Summary focus area")
    user_id: Optional[str] = Field(default=None, description="Optional user ID for logging")

    @field_validator('text')
    @classmethod
    def validate_text_length(cls, v):
        word_count = len(v.split())
        if word_count < 50:
            raise ValueError("Text is too short to summarize (minimum 50 words)")
        if word_count > 5000:
            raise ValueError("Text exceeds maximum 5000 words")
        return v
```

**SummarizationResponse**:
```python
class SummarizationResponse(BaseModel):
    summary: str = Field(..., description="Generated summary")
    original_word_count: int = Field(..., description="Word count of original text")
    summary_word_count: int = Field(..., description="Word count of summary")
    compression_ratio: float = Field(..., description="Ratio of summary length to original (0-1)")
    processing_time_ms: int = Field(..., description="Summarization processing time in milliseconds")
    key_points: Optional[List[str]] = Field(default=None, description="Extracted key points (optional)")
```

### Interest Management Models

**InterestSelectionRequest**:
```python
class InterestSelectionRequest(BaseModel):
    user_id: str = Field(..., description="User UUID from better-auth")
    interest_ids: List[int] = Field(..., min_length=2, max_length=5, description="List of interest category IDs (2-5)")
    background: str = Field(..., pattern="^(student|professional)$", description="User background level")

    @field_validator('interest_ids')
    @classmethod
    def validate_unique_interests(cls, v):
        if len(v) != len(set(v)):
            raise ValueError("Duplicate interests are not allowed")
        return v
```

**InterestSelectionResponse**:
```python
class InterestCategory(BaseModel):
    id: int
    name: str
    slug: str

class InterestSelectionResponse(BaseModel):
    success: bool
    user_id: str
    interests: List[InterestCategory]
    background: str
    updated_at: datetime
```

**GetUserInterestsResponse**:
```python
class GetUserInterestsResponse(BaseModel):
    user_id: str
    interests: List[InterestCategory]
    background: str
    updated_at: datetime
```

### Personalized Chat Models

**PersonalizedChatRequest** (extends existing ChatRequest):
```python
class PersonalizedChatRequest(BaseModel):
    message: str = Field(..., min_length=1, max_length=2000)
    session_id: str = Field(..., description="Chat session UUID")
    user_id: Optional[str] = Field(default=None, description="User UUID for personalization")
    context_text: Optional[str] = Field(default=None, description="Selected text for context-constrained mode")
```

**PersonalizedChatResponse** (extends existing ChatResponse):
```python
class PersonalizedChatResponse(BaseModel):
    response: str = Field(..., description="Generated chat response")
    session_id: str
    status: str = Field(..., pattern="^(success|guardrail_triggered|error)$")
    personalized: bool = Field(default=False, description="Whether personalization was applied")
    interests_used: Optional[List[str]] = Field(default=None, description="Interest slugs used for personalization")
```

## Non-Persisted Data Structures

### Translation Cache (In-Memory)

**Purpose**: Cache translation results for 1 hour to reduce API calls for repeated text

**Structure** (Redis or Python dict):
```python
translation_cache: Dict[str, CachedTranslation] = {}

class CachedTranslation:
    text_hash: str          # SHA256 hash of original text
    translated_text: str    # Cached translation
    target_lang: str        # Language code
    cached_at: datetime     # Cache timestamp
    expires_at: datetime    # Expiration timestamp (1 hour)
```

**Cache Key Format**: `translation:{target_lang}:{text_hash}`

### Personalization Context (Runtime Only)

**Purpose**: Inject user interests into agent system prompt

**Structure**:
```python
class PersonalizationContext:
    user_id: str
    interests: List[str]           # Interest slugs
    background: str                # student or professional
    system_prompt_suffix: str      # Generated prompt addition
```

**Example System Prompt Injection**:
```
Base System Prompt: "You are a helpful AI assistant for a humanoid robotics textbook..."

+ Personalization Suffix:
"The user has expertise in Software Engineering and AI/ML. When explaining robotics concepts:
- Use software architecture analogies (microservices, pub-sub patterns, APIs)
- Relate control systems to machine learning pipelines
- Include code examples in Python/ROS 2
- Provide step-by-step explanations suitable for a student-level audience"
```

## Data Flow Diagrams

### Translation Data Flow

```
┌───────────┐
│  User     │
│  Selects  │
│  Text     │
└─────┬─────┘
      │
      v
┌─────────────────────┐
│  Frontend           │
│  - Extract text     │
│  - Validate length  │
└─────┬───────────────┘
      │ POST /translate
      │ {text, target_lang, preserve_code}
      v
┌─────────────────────┐
│  Backend            │
│  - Validate input   │
│  - Check cache      │
└─────┬───────────────┘
      │
      v
┌─────────────────────┐
│  Cache Hit?         │
└─────┬───────────────┘
      │ No
      v
┌─────────────────────┐
│  Translation        │
│  Service            │
│  - Preprocess       │
│  - Call API         │
│  - Postprocess      │
└─────┬───────────────┘
      │
      v
┌─────────────────────┐
│  Cache Result       │
│  (1 hour TTL)       │
└─────┬───────────────┘
      │
      v
┌─────────────────────┐
│  Return Response    │
│  {translated_text,  │
│   original_text...} │
└─────────────────────┘
```

### Personalization Data Flow

```
┌───────────┐
│  User     │
│  Sends    │
│  Message  │
└─────┬─────┘
      │
      v
┌─────────────────────┐
│  Frontend           │
│  - Include user_id  │
└─────┬───────────────┘
      │ POST /agent/chat
      │ {message, user_id, session_id}
      v
┌─────────────────────┐
│  Backend            │
│  - Validate input   │
└─────┬───────────────┘
      │
      v
┌─────────────────────┐
│  Fetch User         │
│  Interests          │
│  (PostgreSQL)       │
└─────┬───────────────┘
      │ {interests: [software_eng, ai_ml], background: student}
      v
┌─────────────────────┐
│  Build              │
│  Personalization    │
│  Context            │
└─────┬───────────────┘
      │
      v
┌─────────────────────┐
│  Inject into        │
│  Agent System       │
│  Prompt             │
└─────┬───────────────┘
      │
      v
┌─────────────────────┐
│  RAG Retrieval      │
│  (Unchanged)        │
└─────┬───────────────┘
      │
      v
┌─────────────────────┐
│  Agent Response     │
│  Generation         │
│  (Personalized)     │
└─────┬───────────────┘
      │
      v
┌─────────────────────┐
│  Return Response    │
│  {response,         │
│   personalized:true}│
└─────────────────────┘
```

## Validation Rules

### Translation Validation

1. **Input Text**:
   - Min length: 1 character
   - Max length: 10,000 characters
   - Max word count: 5,000 words
   - Cannot be empty or whitespace-only

2. **Target Language**:
   - Must be "ur" (Urdu) or "en" (English)
   - Currently only Urdu translation supported

3. **Code Preservation**:
   - Detect code blocks (```code```, `inline`, indented blocks)
   - Extract before translation
   - Reinsert after translation

### Summarization Validation

1. **Input Text**:
   - Min length: 50 words
   - Max length: 5,000 words
   - Must contain at least 2 sentences

2. **Target Length**:
   - SHORT: 15-25% of original
   - MEDIUM: 25-35% of original
   - LONG: 35-45% of original

3. **Focus**:
   - CONCEPTS: Exclude code from summary
   - CODE: Include code snippets with brief explanations
   - BOTH: Balance concepts and code

### Interest Selection Validation

1. **Number of Interests**:
   - Min: 2 interests
   - Max: 5 interests
   - Must be unique (no duplicates)

2. **Interest IDs**:
   - Must exist in interest_categories table
   - Must be active (not deleted)

3. **Background**:
   - Must be "student" or "professional"
   - Cannot be empty

## Error Handling

### Database Errors

```python
class DatabaseError(Exception):
    """Base exception for database errors"""
    pass

class UserNotFoundError(DatabaseError):
    """User profile not found in database"""
    pass

class InterestLimitError(DatabaseError):
    """User has invalid number of interests (not 2-5)"""
    pass

class DuplicateInterestError(DatabaseError):
    """Attempt to add duplicate interest for user"""
    pass
```

### API Errors

```python
class TranslationAPIError(Exception):
    """Translation API call failed"""
    status_code: int
    error_message: str

class SummarizationError(Exception):
    """LLM summarization failed"""
    error_type: str  # "timeout", "rate_limit", "invalid_response"
    error_message: str
```

### Error Responses

```python
class ErrorResponse(BaseModel):
    error: str = Field(..., description="Error type")
    message: str = Field(..., description="Human-readable error message")
    details: Optional[Dict[str, Any]] = Field(default=None, description="Additional error details")
    timestamp: datetime = Field(default_factory=datetime.utcnow)
```

**Example Error Responses**:

```json
{
  "error": "VALIDATION_ERROR",
  "message": "Text exceeds maximum 5000 words",
  "details": {"word_count": 6543, "max_allowed": 5000},
  "timestamp": "2025-12-18T10:30:00Z"
}
```

```json
{
  "error": "TRANSLATION_API_ERROR",
  "message": "Translation service temporarily unavailable",
  "details": {"api_status": 503, "retry_after": 60},
  "timestamp": "2025-12-18T10:30:00Z"
}
```

## Performance Considerations

### Database Queries

**Optimized Interest Fetch**:
```sql
SELECT ic.id, ic.name, ic.slug
FROM user_interests ui
JOIN interest_categories ic ON ui.interest_id = ic.id
WHERE ui.user_id = $1;
```

**Query Performance**: <5ms (indexed on user_id)

### Connection Pooling

```python
# asyncpg connection pool configuration
pool = await asyncpg.create_pool(
    dsn=NEON_DATABASE_URL,
    min_size=5,
    max_size=20,
    max_queries=50000,
    max_inactive_connection_lifetime=300
)
```

### Caching Strategy

- **Translation Cache**: In-memory dict (or Redis) with 1-hour TTL
- **Interest Cache**: Cache user interests in session (clear on update)
- **Summarization**: No caching (ephemeral, context-dependent)

## Security Considerations

### Input Sanitization

- Sanitize all user inputs to prevent SQL injection (use parameterized queries)
- Validate all Pydantic models before processing
- Limit request body size (10 KB for translation/summarization)

### Authentication

- Verify user_id matches authenticated user from better-auth
- Allow unauthenticated translation/summarization (P1 features)
- Require authentication for interest management and personalization (P2 features)

### API Key Protection

- Store all API keys in environment variables
- Never log API keys or sensitive data
- Rotate API keys quarterly

## Appendix: SQL Queries

### Common Queries

**Get User Interests**:
```sql
SELECT ic.id, ic.name, ic.slug
FROM user_interests ui
JOIN interest_categories ic ON ui.interest_id = ic.id
WHERE ui.user_id = $1;
```

**Save User Interests** (transaction):
```sql
BEGIN;

-- Delete existing interests
DELETE FROM user_interests WHERE user_id = $1;

-- Insert new interests
INSERT INTO user_interests (user_id, interest_id)
VALUES
  ($1, $2),
  ($1, $3),
  ($1, $4);

-- Validate count (2-5)
SELECT COUNT(*) AS count FROM user_interests WHERE user_id = $1;
-- Application layer verifies count is between 2 and 5

COMMIT;
```

**Get All Interest Categories**:
```sql
SELECT id, name, slug, description
FROM interest_categories
ORDER BY name;
```

**Create User Profile**:
```sql
INSERT INTO user_profiles (user_id, background, language_preference)
VALUES ($1, $2, 'en')
ON CONFLICT (user_id) DO NOTHING;
```
