# Quickstart Guide: RAG Retrieval Pipeline

**Feature**: 2-rag-retrieval
**Date**: 2025-12-15
**Prerequisites**: Vector ingestion pipeline completed (feature 1), Cohere API key, Qdrant Cloud access

## Overview

This quickstart guide helps you set up and use the RAG retrieval pipeline for semantic search over humanoid robotics book content.

**What you'll be able to do**:
- Search book content using natural language queries
- Filter results by book section/chapter
- Validate retrieval quality with test suite
- Monitor service health

## Prerequisites

### Required
- ✅ Python 3.13+ installed
- ✅ UV package manager (or pip)
- ✅ Vector ingestion completed (collection "humanoid-robotics-embeddings" exists with 400+ vectors)
- ✅ Cohere API key ([get one here](https://cohere.com/))
- ✅ Qdrant Cloud cluster URL and API key

### Verify Prerequisites

```bash
# Check Python version
python --version  # Should be 3.13+

# Check collection exists (from previous ingestion)
# Your Qdrant dashboard should show:
# - Collection: humanoid-robotics-embeddings
# - Vectors: 400+
# - Vector size: 1024
# - Distance: Cosine
```

## Setup

### 1. Navigate to Backend Directory

```bash
cd backend
```

### 2. Configure Environment Variables

Create or update `.env` file:

```bash
cp .env.example .env
```

Edit `.env` with your API credentials:

```env
COHERE_API_KEY=your-cohere-api-key-here
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_URL=https://your-cluster-id.qdrant.tech
```

⚠️ **Important**: If you previously used `CO_API_KEY`, update to `COHERE_API_KEY` for consistency.

### 3. Install Dependencies

**Using UV** (recommended):
```bash
uv pip install -r requirements.txt
```

**Using pip**:
```bash
pip install -r requirements.txt
```

**Note**: No new dependencies are needed if you already ran the ingestion pipeline.

### 4. Start the Server

```bash
python main.py --run-server
```

**Expected output**:
```
INFO:     Started server process [12345]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
```

### 5. Verify Installation

Open your browser and navigate to:
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

You should see three new endpoints:
- `POST /search` - Semantic search
- `POST /test-retrieval` - Quality validation
- `GET /health` - Health check

## Basic Usage

### Health Check

Verify the service is working:

```bash
curl http://localhost:8000/health
```

**Expected response**:
```json
{
  "status": "healthy",
  "qdrant_connected": true,
  "collection_exists": true,
  "collection_vectors_count": 403,
  "cohere_available": true
}
```

### Simple Search Query

Search for content about inverse kinematics:

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is inverse kinematics?",
    "top_k": 5
  }'
```

**Expected response**:
```json
{
  "query": "What is inverse kinematics?",
  "results": [
    {
      "text": "Inverse kinematics (IK) is the process of determining joint angles that achieve a desired end effector position...",
      "score": 0.89,
      "metadata": {
        "url": "https://physical-ai-and-humanoid-robotics-t-two.vercel.app/docs/module1/chapter3-kinematics",
        "chunk_index": 5,
        "source": "website"
      }
    },
    // ... 4 more results
  ],
  "total_results": 5,
  "latency_ms": 567,
  "filters_applied": null,
  "message": null
}
```

### Search with Filters

Search only within Module 1:

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How does ROS2 work?",
    "top_k": 3,
    "filters": {
      "url_contains": "module1"
    }
  }'
```

### Adjust Result Count

Get top 10 results:

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "bipedal locomotion techniques",
    "top_k": 10
  }'
```

## Validation & Testing

### Run Full Test Suite

Test retrieval quality with all 20 predefined queries:

```bash
curl -X POST http://localhost:8000/test-retrieval \
  -H "Content-Type: application/json" \
  -d '{
    "query_ids": null,
    "top_k": 5
  }'
```

**Expected response**:
```json
{
  "total_queries": 20,
  "successful_queries": 17,
  "success_rate": 0.85,
  "meets_target": true,
  "results": [
    {
      "query_id": 1,
      "query_text": "What is inverse kinematics?",
      "expected_chapter": "chapter3-kinematics",
      "found_in_top_k": true,
      "top_result_url": "https://...chapter3-kinematics",
      "top_result_score": 0.89
    },
    // ... 19 more results
  ],
  "avg_latency_ms": 523
}
```

**Success Criteria**: `success_rate >= 0.85` (85% or higher)

### Run Specific Test Queries

Test only queries 1, 2, and 11:

```bash
curl -X POST http://localhost:8000/test-retrieval \
  -H "Content-Type: application/json" \
  -d '{
    "query_ids": [1, 2, 11],
    "top_k": 5
  }'
```

## Common Use Cases

### Use Case 1: Finding Chapter Content

**Goal**: Find all content about bipedal locomotion

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How do bipedal robots maintain balance while walking?",
    "top_k": 5
  }'
```

**Result**: Returns chunks from chapter9-bipedal-locomotion

### Use Case 2: Topic-Specific Search

**Goal**: Learn about VLA models

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are Vision-Language-Action models and how do they work?",
    "top_k": 7
  }'
```

**Result**: Returns chunks from chapter13-vla-models

### Use Case 3: Section-Filtered Search

**Goal**: Find simulation-related content only

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How to simulate robots?",
    "top_k": 5,
    "filters": {
      "url_contains": "module2"
    }
  }'
```

**Result**: Returns chunks only from Module 2 (Simulation chapters)

### Use Case 4: Quality Monitoring

**Goal**: Verify retrieval quality after re-ingestion

```bash
# Re-run test suite
curl -X POST http://localhost:8000/test-retrieval \
  -H "Content-Type: application/json" \
  -d '{"query_ids": null, "top_k": 5}'

# Check success rate
# Should be >= 0.85 (85%)
```

## Using Swagger UI (Interactive Testing)

### 1. Open Swagger UI

Navigate to: http://localhost:8000/docs

### 2. Try the `/search` Endpoint

1. Click on `POST /search`
2. Click "Try it out"
3. Edit the request body:
   ```json
   {
     "query": "What is inverse kinematics?",
     "top_k": 5
   }
   ```
4. Click "Execute"
5. View response below

### 3. Test Different Queries

**Example queries to try**:
- "How does ROS2 message passing work?"
- "Explain SLAM for robot navigation"
- "What are the main components of a humanoid robot?"
- "How do VLA models work for robotics?"
- "What control strategies are used for stable walking?"

### 4. Test Filtering

```json
{
  "query": "humanoid robot sensors",
  "top_k": 5,
  "filters": {
    "url_contains": "perception"
  }
}
```

## Configuration Options

### Environment Variables

| Variable | Description | Example | Required |
|----------|-------------|---------|----------|
| `COHERE_API_KEY` | Cohere API key for embeddings | `abc123...` | Yes |
| `QDRANT_API_KEY` | Qdrant Cloud API key | `xyz789...` | Yes |
| `QDRANT_URL` | Qdrant cluster URL | `https://cluster.qdrant.tech` | Yes |

### Request Parameters

#### SearchRequest

| Parameter | Type | Range | Default | Description |
|-----------|------|-------|---------|-------------|
| `query` | string | 1-1000 chars | required | Natural language query |
| `top_k` | integer | 1-20 | 5 | Number of results |
| `filters` | object | optional | null | Metadata filters |

#### Filters

| Filter Key | Type | Description | Example |
|------------|------|-------------|---------|
| `url_contains` | string | Substring match in URL | `"module1"`, `"locomotion"` |
| `url_exact` | string | Exact URL match | `"https://...chapter3"` |
| `source` | string | Source type | `"website"` |
| `chunk_index` | integer | Specific chunk index | `5` |

### Response Fields

| Field | Type | Description |
|-------|------|-------------|
| `query` | string | Original query text |
| `results` | array | List of RetrievedChunk objects |
| `total_results` | integer | Number of results returned |
| `latency_ms` | integer | Query processing time (ms) |
| `filters_applied` | object | Filters that were applied (if any) |
| `message` | string | Optional message (e.g., "No results found") |

#### RetrievedChunk

| Field | Type | Description |
|-------|------|-------------|
| `text` | string | Chunk text content |
| `score` | float | Similarity score (0.0-1.0, higher is better) |
| `metadata.url` | string | Source URL |
| `metadata.chunk_index` | integer | Chunk position in source |
| `metadata.source` | string | Source type |

## Troubleshooting

### Error: "Embedding service unavailable" (503)

**Cause**: Cohere API is down or API key is invalid

**Solutions**:
1. Verify `COHERE_API_KEY` in `.env` is correct
2. Check Cohere API status: https://status.cohere.com/
3. Verify API key has not expired
4. Check internet connectivity

### Error: "Search service unavailable" (503)

**Cause**: Qdrant connection failed

**Solutions**:
1. Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
2. Check Qdrant dashboard: https://cloud.qdrant.io/
3. Verify collection "humanoid-robotics-embeddings" exists
4. Check firewall allows outbound HTTPS to Qdrant

### Error: "Query cannot be empty" (400)

**Cause**: Empty or whitespace-only query

**Solution**: Provide a non-empty query string

### Error: "Input should be less than or equal to 20" (400)

**Cause**: `top_k` parameter out of range

**Solution**: Use `top_k` between 1 and 20

### No Results Returned

**Possible Causes**:
1. Query topic not covered in book content
2. Filters too restrictive (e.g., `url_contains` doesn't match any URLs)
3. Collection is empty (run ingestion first)

**Solutions**:
1. Try broader query (e.g., "robots" instead of "quantum robots")
2. Remove or relax filters
3. Verify collection has vectors: `curl http://localhost:8000/health`

### Low Relevance Scores (<0.5)

**Cause**: Query semantics don't match book content well

**Solutions**:
1. Rephrase query using technical terminology (e.g., "inverse kinematics" instead of "robot arm math")
2. Try more specific queries
3. Check test suite results to validate retrieval quality

### Slow Queries (>2 seconds)

**Possible Causes**:
1. Network latency to Cohere or Qdrant
2. Qdrant cluster overloaded
3. Large top_k value (e.g., 20)

**Solutions**:
1. Check network connectivity
2. Reduce `top_k` to 5-10
3. Upgrade Qdrant plan if consistently slow

## Performance Expectations

### Latency

| Scenario | Expected Latency | Acceptable Range |
|----------|------------------|------------------|
| Single query | 300-800ms | <2000ms (2s) |
| 10 concurrent queries | 500-1200ms avg | <3000ms (3s) avg |
| Health check | 100-300ms | <500ms |

### Accuracy

| Metric | Target | Acceptable |
|--------|--------|-----------|
| Test suite success rate | 85%+ | ≥80% |
| Relevant results in top-5 | 85%+ | ≥80% |
| Relevance score (good match) | >0.7 | >0.5 |

## Next Steps

### 1. Integration with Chatbot

Use the `/search` endpoint to fetch relevant context for RAG chatbot:

```python
# Pseudocode for chatbot integration
def answer_question(user_query: str) -> str:
    # Step 1: Retrieve relevant chunks
    chunks = requests.post(
        "http://localhost:8000/search",
        json={"query": user_query, "top_k": 5}
    ).json()["results"]

    # Step 2: Format context for LLM
    context = "\n\n".join([chunk["text"] for chunk in chunks])

    # Step 3: Generate answer using LLM
    answer = llm.generate(
        prompt=f"Context: {context}\n\nQuestion: {user_query}\n\nAnswer:"
    )

    return answer
```

### 2. Custom Test Queries

Extend the test suite with domain-specific queries:

```python
# Add to TEST_QUERIES in main.py
{"id": 21, "query": "Your custom query here",
 "expected": "expected-chapter-pattern", "category": "Custom"}
```

### 3. Advanced Filtering

Combine multiple filters:

```bash
curl -X POST http://localhost:8000/search \
  -H "Content-Type: application/json" \
  -d '{
    "query": "robot control",
    "top_k": 5,
    "filters": {
      "url_contains": "module1",
      "source": "website"
    }
  }'
```

### 4. Monitoring

Track retrieval quality over time:

```bash
# Run daily test suite
curl -X POST http://localhost:8000/test-retrieval \
  -H "Content-Type: application/json" \
  -d '{"query_ids": null, "top_k": 5}' \
  | jq '.success_rate'

# Log results for trend analysis
```

## Additional Resources

- **API Documentation**: http://localhost:8000/docs
- **OpenAPI Spec**: http://localhost:8000/openapi.json
- **Feature Spec**: [spec.md](./spec.md)
- **Implementation Plan**: [plan.md](./plan.md)
- **Data Models**: [data-model.md](./data-model.md)
- **API Contract**: [contracts/api-contract.yaml](./contracts/api-contract.yaml)

## Support

For issues or questions:
1. Check troubleshooting section above
2. Review error logs in terminal
3. Verify health check endpoint status
4. Consult feature specification and plan documents
