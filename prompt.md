Feature Name: **Integrated RAG Chatbot for the Textbook**


The chatbot must be embedded inside the Docusaurus website and:

* Answer questions about the book’s content
* Support **“Selected Text Only”** mode (answer using only user-highlighted text)
* Use a **RAG pipeline**
* Use **Gemini (free tier)** for:

  * Embeddings
  * Chat/completion
* Use:

  * **FastAPI** as backend API
  * **Qdrant Cloud (Free Tier)** as vector database
  * **Neon Serverless Postgres** for persistent chat history
  * OpenAI Agents / ChatKit–style interaction model

---

Write the specification in **phases**, showing system evolution from start to finish:

### Phase 1 – Knowledge Ingestion

* Discover all Markdown files inside `my-website/docs`
* Extract and normalize book content
* Chunk documents using a deterministic, repeatable strategy
* Track source metadata (module, chapter, file path, heading)

### Phase 2 – Embedding & Indexing

* Generate embeddings using Gemini embedding model
* Store vectors in Qdrant with:

  * chunk_id
  * source metadata
  * text payload
* Define re-indexing and update strategy when content changes

### Phase 3 – Retrieval & Context Assembly

* Query Qdrant using user input or selected text
* Rank and filter chunks
* Construct bounded context windows
* Enforce “Selected Text Only” isolation mode

### Phase 4 – Generation Layer

* Use Gemini chat model
* System prompt enforces:

  * Book-only knowledge
  * Citation-style responses
  * Refusal when answer is not in corpus

### Phase 5 – Chat Interface (Docusaurus)

* Embedded chat widget
* Supports:

  * Free-form question
  * Selected-text query
  * Session switch
  * History replay

### Phase 6 – Persistent Chat History (Neon)

All conversations must be stored in **Neon Serverless Postgres**.

Each session must contain:

* session_id
* user_id (anonymous or authenticated)
* timestamped message pairs
* retrieval context snapshot
* model metadata

Define a **Chat History Lifecycle**:

* Session creation
* Message append
* Context replay
* Session resume
* Session reset/delete

---

For the full specification, include:

* User personas
* User stories with priorities
* Functional requirements (FR-RAG-xxx, FR-CH-xxx)
* Acceptance scenarios in **Given / When / Then** format
* Edge cases and constraints
* Key entities and system concepts
* Measurable success criteria (SC-xxx)

---

Add a final section titled:

## Environment & Deployment Configuration

Define a canonical `env.example` **as a contract**, not as code.

It must:

* Group variables by subsystem:

  * LLM (Gemini)
  * Vector DB (Qdrant)
  * Database (Neon)
  * API (FastAPI)
  * Frontend (Docusaurus)
  * Security

* For each variable:

  * Name
  * Purpose
  * Server-only vs Public

Include (at minimum):

* Gemini API keys (chat + embeddings)
* Qdrant URL and API key
* Neon Postgres connection string
* FastAPI service config
* Frontend public chat variables
* Security secrets (JWT/session signing)

Do **NOT** include real values.
Do **NOT** write code.
Treat `env.example` as a production deployment contract.


You MUST consult official documentation whenever behavior, structure, or constraints are unclear.
Use these sources as ground truth:

* Docusaurus Docs: [https://docusaurus.io/docs](https://docusaurus.io/docs)
* FastAPI Docs: [https://fastapi.tiangolo.com/](https://fastapi.tiangolo.com/)
* Qdrant Docs: [https://qdrant.tech/documentation/](https://qdrant.tech/documentation/)
* Neon Docs: [https://neon.tech/docs](https://neon.tech/docs)
* Gemini API Docs: [https://ai.google.dev/gemini-api/docs](https://ai.google.dev/gemini-api/docs)
* OpenAI Agents / ChatKit Concepts: [https://platform.openai.com/docs/assistants](https://platform.openai.com/docs/assistants)

If any architectural or lifecycle decision is ambiguous, infer it by aligning with these docs and modern best practices.

---

