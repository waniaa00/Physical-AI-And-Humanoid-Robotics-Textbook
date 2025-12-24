# Vector Ingestion Pipeline

This project implements a vector ingestion pipeline that crawls a deployed website, extracts readable content, chunks the text, generates embeddings using Cohere, and stores them in Qdrant Cloud vector database.

## Features

- Crawls deployed website URLs (with sitemap.xml support as primary source)
- Extracts readable sections from HTML, removing markup and navigation elements
- Chunks extracted text and generates embeddings using Cohere models
- Stores embeddings and metadata in Qdrant Cloud vector DB
- Exposes ingestion and verification endpoints via FastAPI

## Dependencies

- Python 3.13+
- aiohttp
- beautifulsoup4
- qdrant-client
- cohere
- fastapi
- uvicorn
- python-dotenv

## Setup

1. Install dependencies:
```bash
cd backend
uv pip install -r requirements.txt
```

2. Set up environment variables:
```bash
export COHERE_API_KEY="your-cohere-api-key"
export QDRANT_API_KEY="your-qdrant-api-key"
export QDRANT_URL="your-qdrant-cluster-url"
```

## Usage

### Run the main pipeline:
```bash
cd backend
python main.py
```

### Run as a FastAPI server:
```bash
cd backend
python main.py --run-server
```

Then visit:
- `POST /ingest` - to trigger the ingestion pipeline
- `GET /verify` - to verify data has been stored in Qdrant

## Functions

- `get_all_urls()` - Gets URLs from sitemap.xml (primary) or crawls the website as fallback
- `get_urls_from_sitemap()` - Extracts URLs from sitemap.xml file
- `extract_text_from_url()` - Extracts readable text from a URL
- `chunk_text()` - Chunks text into smaller pieces
- `embed()` - Generates embeddings using Cohere
- `create_collection()` - Creates a collection in Qdrant
- `save_chunk_to_qdrant()` - Saves a chunk with its embedding to Qdrant