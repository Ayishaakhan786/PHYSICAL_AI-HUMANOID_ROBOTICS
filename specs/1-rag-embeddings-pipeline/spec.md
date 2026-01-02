# Feature Specification: RAG Pipeline - URL Ingestion, Embeddings, and Vector Storage

**Feature Branch**: `1-rag-embeddings-pipeline`
**Created**: 2025-12-31
**Status**: Draft
**Input**: User description: "Spec-1: Website URL ingestion, embedding generation, and vector storage for RAG chatbot"

## User Scenarios & Testing *(mandatory)*

<!--
  User stories are prioritized by importance. Each story delivers independent value.
  P1 = Must-have (core functionality)
  P2 = Should-have (important enhancements)
  P3 = Nice-to-have (future improvements)
-->

### User Story 1 - Pipeline Execution and Content Ingestion (Priority: P1)

As a developer building a RAG chatbot, I want to ingest all content from my deployed Docusaurus book website so that I can search and retrieve relevant information.

**Why this priority**: This is the foundational capability - without ingesting content, nothing else in the RAG pipeline works. This delivers immediate value by enabling knowledge capture from existing documentation.

**Independent Test**: Can be tested by running the ingestion script against a sample URL and verifying that extracted text is saved to a local file with proper structure.

**Acceptance Scenarios**:

1. **Given** a list of valid public URLs, **When** the ingestion pipeline runs, **Then** all accessible pages must be crawled and their content extracted successfully.
2. **Given** a URL that returns a 404 error, **When** the pipeline attempts to crawl it, **Then** the error must be logged and the pipeline must continue processing remaining URLs.
3. **Given** a URL that requires authentication, **When** the pipeline attempts to crawl it, **Then** the pipeline must gracefully handle the access denial and log the failure.
4. **Given** the book website structure with multiple sections, **When** content is extracted, **Then** each chunk must contain its source URL and section hierarchy as metadata.

---

### User Story 2 - Content Chunking with Metadata (Priority: P1)

As a developer, I want content to be split into manageable chunks with rich metadata so that retrieval returns focused, relevant passages.

**Why this priority**: Chunk quality directly impacts retrieval relevance. Well-chunked content with metadata enables precise filtering and prevents unrelated content from appearing in search results.

**Independent Test**: Can be tested by running the chunking module on sample content and verifying that chunks have predictable boundaries, appropriate size limits, and complete metadata.

**Acceptance Scenarios**:

1. **Given** extracted page content, **When** chunking is applied, **Then** each chunk must not exceed the maximum token limit for the embedding model.
2. **Given** a page with multiple headings, **When** content is chunked, **Then** chunk boundaries should align with semantic sections where possible.
3. **Given** any chunk created, **When** metadata is attached, **Then** it must include: source URL, section path, heading context, and chunk sequence number.
4. **Given** the same source content, **When** chunking runs multiple times, **Then** chunks must be identical (deterministic output).

---

### User Story 3 - Embedding Generation with Cohere (Priority: P1)

As a developer, I want to generate consistent, high-quality vector embeddings for all content chunks so that semantic similarity search can accurately retrieve relevant information.

**Why this priority**: Embeddings are the core of semantic search. Consistent, high-quality embeddings directly determine retrieval accuracy and user satisfaction with the RAG chatbot.

**Independent Test**: Can be tested by generating embeddings for known similar and dissimilar text pairs and verifying that similar texts have higher cosine similarity scores.

**Acceptance Scenarios**:

1. **Given** a text chunk, **When** embedding generation is called, **Then** the output must be a fixed-dimension vector (matching the Cohere model specification).
2. **Given** the same text input, **When** embedding generation runs multiple times, **Then** the output vectors must be identical (deterministic).
3. **Given** multiple chunks processed in batch, **When** embeddings are generated, **Then** the batch must complete within a reasonable timeout and handle partial failures gracefully.
4. **Given** API rate limits or quota issues, **When** embedding generation fails, **Then** the pipeline must implement appropriate retry logic with exponential backoff.

---

### User Story 4 - Vector Storage in Qdrant Cloud (Priority: P1)

As a developer, I want to store all embeddings in a Qdrant Cloud collection with proper indexing so that similarity search returns accurate results quickly.

**Why this priority**: Without persistent storage, embeddings cannot be used for retrieval. Qdrant Cloud provides the vector database backend that enables the RAG chatbot to perform similarity searches.

**Independent Test**: Can be tested by upserting known vectors and then performing similarity searches to verify the correct results are returned.

**Acceptance Scenarios**:

1. **Given** a Qdrant Cloud connection, **When** the collection does not exist, **Then** the pipeline must create it with appropriate vector configuration and indexing.
2. **Given** a Qdrant Cloud connection, **When** the collection already exists, **Then** the pipeline must verify the schema matches requirements without recreating it (idempotent).
3. **Given** embedding vectors with payloads, **When** they are upserted, **Then** each point must be stored with a unique identifier and searchable metadata fields.
4. **Given** stored vectors, **When** a similarity search is performed, **Then** results must be returned with both the vector similarity score and the original payload.

---

### User Story 5 - Pipeline Reproducibility and Idempotency (Priority: P2)

As a developer, I want the pipeline to be reproducible and idempotent so that running it multiple times produces consistent results without duplicating or corrupting data.

**Why this priority**: Production systems need reliable, repeatable processes. Idempotency ensures that partial failures can be recovered without data duplication or loss.

**Independent Test**: Can be tested by running the full pipeline twice and verifying that the second run produces identical results without duplicating vectors.

**Acceptance Scenarios**:

1. **Given** a completed pipeline run, **When** the pipeline runs again with the same inputs, **Then** no duplicate vectors should be created in Qdrant.
2. **Given** a pipeline that fails midway through processing, **When** it is restarted, **Then** it must resume from the last successfully processed item.
3. **Given** new or updated content on the website, **When** the pipeline runs, **Then** only changed chunks should be re-embedded and upserted.
4. **Given** the pipeline configuration, **When** it runs, **Then** it must log all operations with timestamps for audit and debugging.

---

### User Story 6 - Verification and Monitoring (Priority: P2)

As a developer, I want a verification script that confirms vectors are stored correctly so that I can validate the pipeline output before deploying the RAG chatbot.

**Why this priority**: Early detection of pipeline issues prevents downstream problems in the chatbot. Verification gives confidence that the retrieval layer is functioning correctly.

**Independent Test**: Can be tested by running the verification script and confirming it reports accurate counts and performs successful sample searches.

**Acceptance Scenarios**:

1. **Given** a populated Qdrant collection, **When** verification runs, **Then** it must report the total vector count matching the expected number of chunks.
2. **Given** the verification script, **When** it executes a sample similarity search, **Then** it must return results with similarity scores above the configured threshold.
3. **Given** the verification script, **When** it checks metadata, **Then** it must confirm that filtering by URL and section is possible on stored vectors.
4. **Given** verification results, **When** issues are detected, **Then** the script must provide actionable error messages indicating the problem and potential fixes.

---

### Edge Cases

- **URL timeout or slow response**: The pipeline must handle slow-loading pages with configurable timeouts and log warnings without blocking the entire process.
- **Malformed HTML or encoding issues**: The extractor must handle various HTML structures and character encodings gracefully, preserving content accuracy.
- **Embedding API failures**: The pipeline must handle transient API failures with retry logic and track failed items for later reprocessing.
- **Qdrant connection issues**: Network connectivity problems with Qdrant must be caught, logged, and the pipeline must exit with a clear error message.
- **Empty or minimal content pages**: Pages with little or no textual content must be handled gracefully, either skipped or logged with a warning.
- **Rate limiting from source website**: The crawler must respect robots.txt and implement rate limiting to avoid overwhelming the source server.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST crawl and extract text content from all publicly accessible URLs in the configured book website.
- **FR-002**: System MUST parse HTML content to extract main article text while filtering out navigation, sidebars, footers, and other non-content elements.
- **FR-003**: System MUST split extracted content into chunks that do not exceed the maximum token limit for the chosen embedding model.
- **FR-004**: System MUST attach metadata to each chunk including: source URL, section hierarchy, heading context, and chunk sequence number.
- **FR-005**: System MUST generate embeddings for all content chunks using Cohere embedding models with consistent vector dimensions.
- **FR-006**: System MUST store all embeddings and their payloads in a Qdrant Cloud collection with appropriate indexing for URL and section filtering.
- **FR-007**: System MUST create the Qdrant collection if it does not exist, configuring it with the correct vector parameters and payload schema.
- **FR-008**: System MUST implement idempotent upsert operations so that re-running the pipeline does not create duplicate vectors.
- **FR-009**: System MUST provide a verification script that confirms vector storage, enables sample similarity searches, and reports collection statistics.
- **FR-010**: System MUST load configuration from environment variables, accepting at minimum: Qdrant URL, Qdrant API key, Cohere API key, and target website URLs.
- **FR-011**: System MUST implement appropriate retry logic with exponential backoff for transient failures in embedding generation.
- **FR-012**: System MUST log all operations with sufficient detail for debugging, including processing progress, errors, and completion status.

### Key Entities

- **ContentChunk**: Represents a segment of extracted text with associated metadata
  - `text`: The textual content of the chunk
  - `source_url`: The original URL the content was extracted from
  - `section_path`: Hierarchical path of the section (e.g., "docs/modules/module-1")
  - `heading`: The nearest heading above the chunk content
  - `chunk_index`: Sequence number for ordering chunks from the same source
  - `char_count`: Length of the chunk text for debugging

- **VectorPoint**: Represents an embedding vector stored in Qdrant
  - `id`: Unique identifier for the point
  - `vector`: The embedding vector generated by Cohere
  - `payload`: Metadata including all ContentChunk fields
  - `score`: Similarity score (populated during search operations)

- **PipelineConfig**: Configuration for the ingestion pipeline
  - `qdrant_url`: Qdrant Cloud endpoint URL
  - `qdrant_api_key`: Authentication key for Qdrant
  - `cohere_api_key`: API key for Cohere embedding service
  - `source_urls`: List of URLs to ingest
  - `chunk_max_tokens`: Maximum tokens per chunk
  - `chunk_overlap`: Overlap between chunks for context preservation
  - `embedding_model`: Cohere model name to use
  - `collection_name`: Name of the Qdrant collection

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: The pipeline MUST successfully crawl and extract content from at least 95% of provided URLs in a single run.
- **SC-002**: Content chunks MUST NOT exceed the embedding model's token limit, with at least 99% of chunks processing successfully.
- **SC-003**: Generated embeddings MUST have consistent dimensionality matching the selected Cohere model specification.
- **SC-004**: Stored vectors MUST support filtering by URL and section, enabling targeted searches within specific book sections.
- **SC-005**: Similarity search MUST return relevant results with cosine similarity scores above 0.7 for known similar queries.
- **SC-006**: The pipeline MUST complete a full ingestion run without manual intervention once started.
- **SC-007**: Re-running the pipeline MUST NOT create duplicate vectors in the Qdrant collection.
- **SC-008**: The verification script MUST confirm all vectors are stored and searchable with a single command execution.

## Assumptions

- The target book website is publicly accessible without authentication requirements.
- Cohere API and Qdrant Cloud Free Tier accounts will be provisioned by the user.
- The book content is primarily Markdown/HTML and contains substantial textual content worth indexing.
- Network connectivity to the target website and APIs is available during pipeline execution.
- The user has basic familiarity with Python and command-line tools for running the pipeline.

## Out of Scope

- Query-time retrieval logic, ranking algorithms, or response generation
- Integration with FastAPI endpoints or OpenAI Agents (will be added later)
- Frontend user interface for the chatbot
- User authentication or authorization for the pipeline
- Fine-tuning or reranking models beyond base embeddings
- Real-time updates when source content changes (manual re-run required)
