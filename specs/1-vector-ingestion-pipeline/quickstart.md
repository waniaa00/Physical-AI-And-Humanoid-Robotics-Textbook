# Quickstart Guide: Vector Ingestion Pipeline

## Prerequisites
- Python 3.13+
- UV package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

### 1. Clone and Navigate to Backend
```bash
cd backend
```

### 2. Install Dependencies
```bash
uv pip install -r requirements.txt
```

### 3. Configure Environment Variables
Create a `.env` file based on `.env.example`:
```bash
cp .env.example .env
```

Edit `.env` with your API keys:
```env
COHERE_API_KEY=your-cohere-api-key-here
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_URL=your-qdrant-cluster-url-here
```

## Usage

### Run the Ingestion Pipeline
```bash
python main.py
```

This will:
1. Get URLs from sitemap.xml (https://physical-ai-and-humanoid-robotics-t-two.vercel.app/sitemap.xml) as primary source, with website crawling as fallback
2. Extract readable content from all pages
3. Chunk the text into appropriate sizes
4. Generate embeddings using Cohere
5. Store embeddings in Qdrant collection "humanoid-robotics-embeddings"

### Run as FastAPI Server
```bash
python main.py --run-server
```

Then use the API endpoints:

#### POST /ingest
Trigger the ingestion pipeline:
```bash
curl -X POST http://localhost:8000/ingest
```

#### GET /verify
Verify stored embeddings:
```bash
curl http://localhost:8000/verify
```

## Configuration Options
- Adjust chunk size by modifying the `chunk_size` parameter in `chunk_text()` function
- Change the target website by updating the `website_url` variable in the main function
- Modify the collection name by changing the `COLLECTION_NAME` constant

## Troubleshooting
- If you get rate limit errors, implement backoff strategies in the embedding function
- If URLs are not being crawled correctly, verify the target website structure
- If embeddings are not storing, check your Qdrant configuration and credentials