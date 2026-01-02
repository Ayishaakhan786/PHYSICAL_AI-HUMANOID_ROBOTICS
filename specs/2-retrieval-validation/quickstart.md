# Quickstart: Retrieval Pipeline Validation

This guide explains how to validate the RAG retrieval pipeline using the `retrieve.py` script.

## Prerequisites

- Python 3.11+
- Existing Qdrant Cloud collection with embedded content
- Cohere API key for query embedding generation

## Setup

The validation script uses the same configuration as the ingestion pipeline:

```bash
# Ensure .env is configured (from backend/ directory)
cd backend
cp .env.example .env  # If needed
# Edit .env with your API keys if not already done
```

Required environment variables:
- `QDRANT_URL`: Qdrant Cloud endpoint
- `QDRANT_API_KEY`: Qdrant authentication key
- `COHERE_API_KEY`: Cohere API key
- `COLLECTION_NAME`: (optional) Defaults to `book-rag-embeddings`

## Running Validation

### Basic Usage

Run all 5 predefined test queries:

```bash
cd D:\CODING\Python\Q4-Hackathons\PHYSICALAI_HUMANOID_ROBOTICS
python retrieve.py
```

### Custom Query

Test a specific search query:

```bash
python retrieve.py --query "humanoid robot control systems"
```

### Verbose Output

See detailed results for each query:

```bash
python retrieve.py --verbose
```

### JSON Output

Save results to a JSON file:

```bash
python retrieve.py --output validation_results.json
```

## Output Format

The script produces two outputs:

### Console Output

```
=== Retrieval Validation Results ===
Total Queries: 5
Passed: 4
Failed: 1
Average Latency: 1245ms
p95 Latency: 2100ms

Results:
1. humanoid robot kinematics - PASS (3 results, 0.89ms)
2. Isaac Sim GPU simulation - PASS (3 results, 0.72ms)
3. vision language action models - PASS (3 results, 0.65ms)
4. domain randomization sim-to-real - FAIL (0 results)
5. human interaction humanoid robots - PASS (3 results, 0.78ms)
```

### JSON Output (--output)

```json
{
  "total_queries": 5,
  "passed": 4,
  "failed": 1,
  "avg_latency_ms": 1245,
  "p95_latency_ms": 2100,
  "results": [
    {
      "query": "humanoid robot kinematics",
      "passed": true,
      "result_count": 3,
      "min_score": 0.72,
      "latency_ms": 89
    }
  ]
}
```

## Validation Criteria

Each test query is validated against:

| Criterion | Threshold | Description |
|-----------|-----------|-------------|
| Result Count | >= 3 | At least 3 chunks returned |
| Relevance Score | >= 0.5 | All results above similarity threshold |
| Metadata Completeness | 100% | All chunks have source_url, section_path, heading |
| Latency | < 3000ms | Query completes within 3 seconds (p95) |

## Test Queries

The script includes 5 predefined test queries based on book content:

1. **"humanoid robot kinematics"** - Tests kinematics/motion content
2. **"Isaac Sim GPU simulation"** - Tests module-4-isaac content
3. **"vision language action models"** - Tests module-5-vla content
4. **"domain randomization sim-to-real"** - Tests sim-to-real content
5. **"human interaction humanoid robots"** - Tests module-6-humanoid content

## Troubleshooting

### "Connection failed" error

- Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct in `.env`
- Check network connectivity to Qdrant Cloud

### "Cohere API error"

- Verify `COHERE_API_KEY` is valid
- Check API rate limits (Free Tier: 5 calls/minute)

### "Collection not found"

- Verify `COLLECTION_NAME` matches existing collection
- Ensure collection was created during ingestion phase

## Integration with Agent Development

Use validation results to:

1. **Verify pipeline health** before agent integration
2. **Identify coverage gaps** - topics with no/missing results
3. **Measure performance** - ensure latency meets agent requirements
4. **Document retrieval quality** - include in technical documentation

## Next Steps

After validation passes:
- Proceed with agent integration
- Add domain-specific test queries as needed
- Monitor retrieval quality in production
