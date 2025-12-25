---
id: 1
title: Create-book-rag-constitution
stage: constitution
date: 2025-12-16
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: user
command: /sp.constitution
labels: ["constitution", "spec", "book", "rag"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 -
---

## Prompt

```text
Project: Spec-Driven Technical Book with Embedded RAG Chatbot

Purpose:
Author and deploy a production-ready technical book using AI-first specifications, and integrate a contextual RAG chatbot capable of answering book-specific and user-selected content queries.

Core principles:
- Spec-first development
- Technical accuracy and verifiability
- Clarity for software engineers
- Reproducibility and automation
- Modular, auditable architecture

Key standards:
- All content and code must be derived from formal specs (Spec-Kit Plus)
- Claude Code is the primary AI agent for writing and implementation
- Book framework: Docusaurus (Markdown/MDX only)
- Deployment: GitHub Pages (fully automated)
- Backend API: FastAPI
- AI integration: OpenAI Agents / ChatKit SDKs
- Vector store: Qdrant Cloud (Free Tier)
- Relational DB: Neon Serverless PostgreSQL
- RAG chatbot must:
  - Answer strictly from indexed book content
  - Support "selected-text-only" question answering
  - Reject out-of-scope or hallucinated responses
  - Separate retrieval context from generation

Documentation standards:
- Audience: intermediate–advanced software engineers
- Tone: concise, technical, instructional
- Code: executable, reproducible, and commented
- APIs: OpenAPI-compliant documentation
- Diagrams: Mermaid or ASCII where relevant

Constraints:
- Docusaurus-compatible Markdown/MDX only
- No paid services beyond free tiers
- Secrets managed via environment variables
- No fabricated citations or unverifiable claims

Success criteria:
- Book builds and deploys successfully to GitHub Pages
- Embedded RAG chatbot is functional within the book UI
- Chatbot accurately answers content-based and selected-text queries
- All implemented features trace back to explicit specs
- Project is reproducible from the repository alone

Governance:
- Specification is the single source of truth
- Ambiguities require spec updates before implementation
- Any deviation must be explicitly documented
```

## Response snapshot

Updated the project constitution to reflect the requirements for the Spec-Driven Technical Book with Embedded RAG Chatbot. The new constitution includes:

- Core Principles: Spec-First Development, Technical Accuracy, Clarity for Engineers, Reproducibility, Modularity, and RAG Integrity
- Technology Stack Standards: Docusaurus, FastAPI, OpenAI Agents, Qdrant Cloud, Neon PostgreSQL
- Development Workflow: Implementation standards, Quality Assurance, and Change Management
- Governance: Clear success criteria and decision-making framework

The constitution is versioned at 1.0.0 and follows the requirements specified in the user input.

## Outcome

- ✅ Impact: Created comprehensive project constitution for Technical Book with RAG Chatbot
- 🧪 Tests: N/A
- 📁 Files: Updated .specify/memory/constitution.md
- 🔁 Next prompts: Plan development tasks based on the new constitution
- 🧠 Reflection: The constitution properly captures all specified requirements and constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A