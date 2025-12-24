# Data Model: Vector Ingestion Pipeline

## Entity: BookContent
**Description**: Represents the extracted text content from a book page
**Fields**:
- url (string): The source URL of the content
- title (string): The page title extracted from HTML
- text_content (string): Clean, readable text content without HTML markup
- source_type (string): How the URL was discovered ("sitemap" or "crawled")
- created_at (datetime): Timestamp when content was extracted
- updated_at (datetime): Timestamp when content was last updated

**Validation Rules**:
- url must be a valid URL format
- text_content must not be empty
- source_type must be one of: "sitemap", "crawled"
- created_at must be in ISO 8601 format

## Entity: TextChunk
**Description**: Represents a segment of text that has been chunked for embedding generation
**Fields**:
- id (string): Unique identifier for the chunk
- content (string): The text content of the chunk
- chunk_index (integer): Position of chunk in the original document
- source_url (string): Reference to the original URL
- source_title (string): Reference to the original page title
- created_at (datetime): Timestamp when chunk was created

**Validation Rules**:
- content must not exceed 500 characters
- chunk_index must be non-negative
- source_url must reference a valid BookContent entity

## Entity: VectorEmbedding
**Description**: Represents the vector representation of a text chunk
**Fields**:
- id (string): Unique identifier for the embedding
- vector (array[float]): The embedding vector values (1024 dimensions)
- chunk_id (string): Reference to the source TextChunk
- text_content (string): The original text that was embedded
- metadata (object): Additional metadata including source URL and chunk index
- created_at (datetime): Timestamp when embedding was generated

**Validation Rules**:
- vector must have exactly 1024 dimensions
- chunk_id must reference a valid TextChunk entity
- text_content must match the referenced TextChunk content

## Entity: IngestionLog
**Description**: Represents records of ingestion activities
**Fields**:
- id (string): Unique identifier for the log entry
- operation (string): Type of operation (crawl, extract, embed, store)
- status (string): Status of the operation (success, failure, in_progress)
- source_url (string): URL being processed (if applicable)
- error_message (string): Error details if operation failed
- timestamp (datetime): When the operation was logged
- duration_ms (integer): Duration of the operation in milliseconds

**Validation Rules**:
- operation must be one of: crawl, extract, embed, store
- status must be one of: success, failure, in_progress
- timestamp must be in ISO 8601 format