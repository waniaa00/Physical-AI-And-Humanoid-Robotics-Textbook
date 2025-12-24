# Research Findings: Vector Ingestion Pipeline

## Decision: Qdrant Cloud Configuration
**Rationale**: Qdrant Cloud uses standard URL format with API key authentication
**Details**:
- URL format: https://your-cluster-name.region-name.cloud.qdrant.io
- API key passed as header: X-Qdrant-API-Key
- Collection creation requires vector size and distance metric specification

## Decision: Cohere Embedding Model Selection
**Rationale**: Cohere embed-english-v3.0 model provides optimal balance of quality and cost for English text content
**Details**:
- Model: embed-english-v3.0
- Input type: search_document for retrieval tasks
- Vector dimensions: 1024 (default)
- Text chunks up to 512 tokens can be processed

## Decision: Text Chunking Strategy
**Rationale**: 500-character chunks provide optimal balance between context preservation and embedding quality
**Details**:
- Chunk size: 500 characters
- Ensures semantic coherence while fitting within embedding model limits
- Allows for proper context during retrieval

## Decision: Website URL Discovery Strategy
**Rationale**: Sitemap.xml provides a comprehensive list of URLs, with crawling as fallback
**Details**:
- Primary approach: Fetch sitemap.xml from target website
- XML parsing with BeautifulSoup using 'xml' parser
- Fallback to web crawling if sitemap is unavailable
- Use lxml as XML parser backend for BeautifulSoup

## Decision: Rate Limiting Implementation
**Rationale**: Implement exponential backoff with retry mechanisms to handle API rate limits gracefully
**Details**:
- Initial retry after 1 second with exponential backoff
- Max retry attempts: 3
- Handle 429 status codes specifically for rate limiting
- Include jitter to prevent thundering herd problems

## Decision: Error Handling Strategy
**Rationale**: Comprehensive error handling ensures pipeline reliability and provides actionable logs
**Details**:
- URL fetch errors: Continue processing other URLs, log error
- Embedding generation errors: Skip chunk, log error
- Qdrant storage errors: Retry with backoff, log if persistent
- All errors logged with appropriate severity levels