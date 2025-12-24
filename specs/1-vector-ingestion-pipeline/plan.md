# Implementation Plan: Vector Ingestion Pipeline

## Technical Context

**Feature**: Vector Ingestion Pipeline
**Target Website**: https://physical-ai-and-humanoid-robotics-t-two.vercel.app/
**Sitemap URL**: https://physical-ai-and-humanoid-robotics-t-two.vercel.app/sitemap.xml
**Collection Name**: humanoid-robotics-embeddings
**Implementation Scope**: Single file backend (main.py) with all required functions


### Architecture Overview
- **Backend**: Python with FastAPI framework
- **Web Crawling**: aiohttp for async requests, BeautifulSoup for HTML parsing
- **Embeddings**: Cohere API for vector generation
- **Vector Storage**: Qdrant Cloud for vector database operations
- **API Layer**: FastAPI for ingestion and verification endpoints

### Technology Stack
- Python 3.13+
- FastAPI: Web framework for API endpoints
- aiohttp: Async HTTP client for web crawling
- BeautifulSoup4: HTML parsing for content extraction
- Cohere: Embedding generation service
- Qdrant Client: Vector database operations
- uvicorn: ASGI server

### Data Flow
1. Get target website URLs from sitemap.xml (primary) or crawl website using `get_all_urls()` and `get_urls_from_sitemap()`
2. Extract readable content from each URL using `extract_text_from_url()`
3. Chunk extracted text using `chunk_text()`
4. Generate embeddings using `embed()`
5. Store in Qdrant using `save_chunk_to_qdrant()`
6. Expose via FastAPI endpoints

### Key Unknowns (NEEDS CLARIFICATION)
- Qdrant Cloud cluster URL format
- Cohere embedding model selection and configuration
- Optimal chunk size for embeddings
- Rate limiting strategies for API calls
- Error handling specifics for production deployment

## Constitution Check

### Principle I. Technical Accuracy & Scientific Rigor
- All code examples must be correct, executable, and validated
- No hardcoded secrets or credentials
- Proper error handling and validation

### Principle III. Modularity & Scalability
- Single file implementation (main.py) as requested
- Modular function design for maintainability
- Clear separation of concerns

### Principle VI. Code & Simulation Standards
- All code snippets must be syntactically correct and executable
- Code must include comments explaining non-obvious logic
- No hardcoded secrets, tokens, or credentials

### Principle VII. Quality Gates & Validation
- All code tested in target environment before publication
- No unverifiable claims about capabilities
- Build and deployment validation required

### Principle VIII. RAG Chatbot & Personalization Standards
- Follow exact agent.py pattern with Cohere embeddings
- Implementation must support RAG bot requirements

### Principle X. Security & Privacy
- Secure API communication and token management
- Input validation and sanitization

## Phase 0: Research & Resolution of Unknowns

### Research Tasks

#### 1. Qdrant Cloud Configuration
**Task**: Research Qdrant Cloud cluster setup and authentication
**Focus**: URL format, API key management, collection creation

#### 2. Cohere Embedding Models
**Task**: Research optimal Cohere embedding model for text content
**Focus**: Model selection, input types, vector dimensions

#### 3. Text Chunking Strategy
**Task**: Research optimal chunk size for embedding generation
**Focus**: Size limits, overlap strategies, semantic boundaries

#### 4. Rate Limiting Implementation
**Task**: Research API rate limiting strategies
**Focus**: Cohere API limits, Qdrant write limits, retry mechanisms

## Phase 1: Design & Architecture

### Data Model
- **BookContent**: URL, title, clean text content
- **TextChunk**: Text segment with metadata, chunk index
- **VectorEmbedding**: Vector values, associated text, metadata
- **IngestionLog**: Timestamp, status, error information

### API Contracts
- `POST /ingest`: Trigger ingestion pipeline
  - Request: None (uses configured website URL)
  - Response: {status: "success", message: string, processed_urls: number}
- `GET /verify`: Verify stored embeddings
  - Request: None
  - Response: {status: "success", collection_info: object, sample_points: number}

### Error Handling Strategy
- Graceful degradation when URLs fail to fetch
- Retry mechanisms for transient failures
- Proper logging for debugging
- API responses with appropriate HTTP status codes

## Phase 2: Implementation Approach

### File Structure
```
backend/
├── main.py              # Single file implementation
├── pyproject.toml       # Project dependencies
├── requirements.txt     # Generated from pyproject.toml
├── README.md           # Setup and usage instructions
└── .env.example        # Environment variable examples
```

### Implementation Sequence
1. Set up project structure and dependencies
2. Implement core functions: `get_all_urls`, `extract_text_from_url`
3. Implement text processing: `chunk_text`, `embed`
4. Implement Qdrant integration: `create_collection`, `save_chunk_to_qdrant`
5. Add FastAPI endpoints: `/ingest`, `/verify`
6. Create main execution function
7. Add configuration and error handling
8. Test and validate implementation

### Success Criteria
- Successfully crawl all pages from target website
- Extract readable content without HTML markup
- Properly chunk text with appropriate size limits
- Generate embeddings using Cohere API
- Store vectors and metadata in Qdrant collection
- FastAPI endpoints respond correctly
- Complete pipeline execution without errors