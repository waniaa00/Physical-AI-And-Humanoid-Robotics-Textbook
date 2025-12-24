# Claude Code Agent Context: Physical AI & Humanoid Robotics Book

## New Capabilities: Vector Ingestion Pipeline

### Web Crawling & Content Extraction
- **aiohttp**: Async HTTP client for efficient web crawling and URL fetching
  - Use `async with aiohttp.ClientSession()` for async requests
  - Handle responses with proper error checking
  - Implement retry mechanisms for transient failures

- **BeautifulSoup4**: HTML parsing for content extraction
  - Parse HTML content with `BeautifulSoup(content, 'html.parser')`
  - Extract readable text with `soup.get_text()`
  - Remove script and style elements with `soup(["script", "style"])`
  - Find links with `soup.find_all('a', href=True)`

### Embedding Generation
- **Cohere**: Embedding generation service
  - Initialize client with `cohere.Client(api_key)`
  - Generate embeddings with `client.embed(texts, model="embed-english-v3.0")`
  - Use input_type="search_document" for retrieval tasks
  - Handle API rate limits with backoff strategies

### Vector Database Operations
- **Qdrant Client**: Vector database operations
  - Connect with `QdrantClient(url=URL, api_key=KEY)`
  - Create collections with `create_collection(name, vectors_config)`
  - Store embeddings with `upsert(collection_name, points)`
  - Query with `scroll()` and `search()` methods

### API Framework
- **FastAPI**: Web framework for API endpoints
  - Define endpoints with `@app.get()` and `@app.post()` decorators
  - Use Pydantic models for request/response validation
  - Handle errors with HTTPException
  - Run with uvicorn for ASGI server functionality

### Implementation Pattern for Vector Ingestion
- **Core Functions**:
  - `get_all_urls(base_url)`: Crawl website and return all valid URLs
  - `extract_text_from_url(url)`: Extract readable content from a URL
  - `chunk_text(text, chunk_size=500)`: Split text into processable chunks
  - `embed(texts)`: Generate embeddings using Cohere
  - `create_collection(name)`: Create Qdrant collection for storage
  - `save_chunk_to_qdrant(chunk, embedding, metadata, collection)`: Store in vector DB

- **Pipeline Execution**:
  - Crawl → Extract → Chunk → Embed → Store → Verify
  - Implement proper error handling and logging
  - Use async/await for efficient processing
  - Include retry mechanisms for external service calls

### Configuration Requirements
- Environment variables: COHERE_API_KEY, QDRANT_API_KEY, QDRANT_URL
- Collection name: "humanoid-robotics-embeddings"
- Target website: "https://physical-ai-and-humanoid-robotics-t-two.vercel.app/"

### API Endpoints
- `POST /ingest`: Trigger the complete ingestion pipeline
- `GET /verify`: Verify stored embeddings in Qdrant
- Return structured JSON responses with status and details

## Integration with RAG System
- Follow exact agent.py pattern with Cohere embeddings and Qdrant storage
- Support for retrieval-augmented generation chatbot functionality
- Proper metadata tagging for downstream retrieval applications
- Consistent with constitution principles for RAG implementation