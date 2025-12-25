<!--
SYNC IMPACT REPORT:
Version change: N/A -> 1.0.0
Added sections: All principles and sections based on project requirements
Removed sections: Template placeholders
Modified principles: None (new constitution)
Templates requiring updates: ⚠ pending - need to check plan-template.md, spec-template.md, tasks-template.md
Follow-up TODOs: None
-->
# Technical Book with RAG Chatbot Constitution

## Core Principles

### I. Spec-First Development
All content and code must be derived from formal specs (Spec-Kit Plus). Specifications serve as the single source of truth for the project. No implementation work begins without a corresponding specification. All features must be traced back to explicit requirements in the specification documents.

### II. Technical Accuracy and Verifiability
Content must be technically accurate and verifiable. No fabricated citations or unverifiable claims are allowed. All code examples must be executable, reproducible, and properly commented. Information must be sourced from authoritative, verifiable resources.

### III. Clarity for Software Engineers
Documentation and code must be designed for intermediate-to-advanced software engineers. Tone should be concise, technical, and instructional. Complex concepts must be explained with practical examples and clear diagrams where relevant.

### IV. Reproducibility and Automation
The entire project must be reproducible from the repository alone. All processes must be automated where possible, including build, test, and deployment workflows. Environment setup must be documented and scripted to ensure consistent reproduction across different machines.

### V. Modular, Auditable Architecture
System architecture must be modular with clear separation of concerns. The RAG chatbot must separate retrieval context from generation. Different components (Docusaurus frontend, FastAPI backend, Qdrant vector store, PostgreSQL database) must have well-defined interfaces and responsibilities.

### VI. RAG Integrity and Scope Compliance
The RAG chatbot must answer strictly from indexed book content, support "selected-text-only" question answering, and reject out-of-scope or hallucinated responses. Retrieval and generation processes must be separated to ensure content integrity.

## Technology Stack Standards

### Frontend Framework
- Book framework: Docusaurus (Markdown/MDX only)
- Deployment: GitHub Pages (fully automated)
- Documentation standards: Audience-appropriate, Docusaurus-compatible Markdown/MDX only

### Backend and AI Integration
- Backend API: FastAPI
- AI integration: OpenAI Agents / ChatKit SDKs
- Vector store: Qdrant Cloud (Free Tier)
- Relational DB: Neon Serverless PostgreSQL

### Security and Configuration
- Secrets managed via environment variables
- No hardcoded credentials or sensitive information in code
- Free tier services only (no paid services beyond free tiers)

## Development Workflow

### Implementation Standards
- Claude Code is the primary AI agent for writing and implementation
- All implemented features must trace back to explicit specs
- APIs must be OpenAPI-compliant with proper documentation
- Diagrams should use Mermaid or ASCII where relevant for clarity

### Quality Assurance
- Code must be executable, reproducible, and commented
- APIs must follow OpenAPI-compliant documentation standards
- Success criteria must be met: book builds/deploy to GitHub Pages, functional RAG chatbot, accurate query responses

### Change Management
- Specification is the single source of truth
- Ambiguities require spec updates before implementation
- Any deviation from specifications must be explicitly documented

## Governance

Specifications govern all development decisions. All changes must be approved through specification updates before implementation. Project success is measured by: successful book build and deployment to GitHub Pages, functional embedded RAG chatbot, accurate content-based and selected-text query responses, and traceability of all features back to explicit specifications.

**Version**: 1.0.0 | **Ratified**: 2025-12-16 | **Last Amended**: 2025-12-16
