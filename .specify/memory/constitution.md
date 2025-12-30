<!--
SYNC IMPACT REPORT
==================
Version change: [none] → 1.0.0
Modified principles: [N/A - initial creation]
Added sections:
  - Core Principles (5 principles)
  - Key Standards
  - Technical Constraints
  - Content Constraints
  - Quality Checks
  - Success Criteria
  - Governance
Removed sections: [N/A - initial creation]
Templates requiring updates:
  - ✅ spec-template.md (aligns with accuracy and clarity principles)
  - ✅ plan-template.md (includes constitution check placeholder)
  - ✅ tasks-template.md (supports spec-driven workflow)
  - ✅ phr-template.prompt.md (reproducibility and documentation)
Follow-up TODOs: None
-->

# Docusaurus Book Project Constitution

## Core Principles

### I. Accuracy Through Verification

All factual and technical claims MUST be traceable to reliable sources. Every assertion requires citation from official documentation, specifications, or reputable technical sources. This is NON-NEGOTIABLE.

**Rationale**: Ensures technical correctness and builds reader trust by grounding all content in authoritative sources rather than assumptions.

### II. Clarity for Developer Audience

Content MUST target developers, CS students, and AI learners. Explanations must be clear, structured, and suitable for self-learning. Technical jargon MUST be explained when first introduced.

**Rationale**: The primary audience consists of practitioners and students who need precise, actionable information without ambiguity.

### III. Spec-Driven Consistency

All content development MUST follow Spec-Kit Plus workflows (spec → plan → tasks → implement). Every chapter and feature MUST have corresponding specification documentation before implementation begins.

**Rationale**: Ensures consistency across the entire book, enables reproducible development processes, and provides clear audit trails for all decisions.

### IV. Reproducibility

All steps, commands, and decisions MUST be documented. Code examples MUST be runnable and logically correct. Readers MUST be able to reproduce all procedures and examples from the documentation alone.

**Rationale**: Enables true learning through hands-on practice and validates that examples actually work as described.

### V. AI-Assisted Rigor with Human Validation

AI (Claude Code) assists with content creation but ALL human-generated content MUST be validated for accuracy. AI-generated content MUST be reviewed before inclusion. No content is auto-published without human oversight.

**Rationale**: Leverages AI efficiency while maintaining quality control and preventing hallucinations or incorrect information.

## Key Standards

### Source Traceability
- All factual and technical claims MUST be traceable to reliable sources
- Source preference MUST follow this hierarchy:
  1. Official documentation and specifications
  2. Reputable technical blogs (e.g., official engineering blogs)
  3. Peer-reviewed academic papers
- Citation format MUST use Markdown-compatible references (APA-style where applicable)

### Originality
- Zero plagiarism tolerance is NON-NEGOTIABLE
- All explanations and examples MUST be original
- Quotations MUST be properly attributed and limited in scope

### Writing Quality
- Writing MUST be clear, structured, and suitable for self-learning
- Progressive difficulty MUST be maintained (beginner → intermediate where applicable)
- Code examples MUST be runnable and logically correct

## Technical Constraints

### Tooling
- Development workflow: Spec-Kit Plus
- AI assistance: Claude Code
- Version control: Git with GitHub
- Deployment: GitHub Pages

### Framework
- Static site generator: Docusaurus
- Content format: Markdown (.md) and MDX (.mdx)
- Navigation: Sidebar-based chapter structure

### Version Control
- Repository MUST be hosted on GitHub
- Commit history MUST be clean and descriptive
- Branches SHOULD follow feature-based naming convention

### Deployment
- Target: GitHub Pages
- Build MUST pass Docusaurus production build
- Site MUST be publicly accessible

## Content Constraints

### Cohesion
- Book MUST be unified and cohesive
- No disconnected chapters allowed
- Each chapter MUST connect logically to preceding and following chapters

### Chapter Template (MANDATORY)
Each chapter MUST follow this structure:
1. **Introduction**: What this chapter covers, prerequisites, learning objectives
2. **Core Concepts**: Key ideas, terminology, theoretical foundations
3. **Examples / Code Snippets**: Practical applications, runnable code
4. **Summary**: Key takeaways, what was learned, next steps

### Progressive Difficulty
- Content MUST progress from beginner concepts to intermediate topics
- Each chapter MUST build upon previous chapters
- Avoid jumping ahead without proper context

## Quality Checks

### Markdown Quality
- Markdown linting MUST be performed before commits
- Formatting MUST be consistent across all files
- Headings MUST follow hierarchical structure

### Link Validation
- All internal links MUST resolve correctly
- External links MUST be checked for accessibility
- Broken links MUST be fixed before deployment

### Code Snippet Review
- All code examples MUST be tested and verified as runnable
- Code MUST follow language-specific best practices
- Comments MUST explain non-obvious logic

### Build Validation
- Docusaurus production build MUST pass without errors
- Warnings MUST be reviewed and addressed where appropriate
- Site preview MUST be validated locally

### Deployment Validation
- Deployed site MUST load correctly on GitHub Pages
- All pages MUST be accessible
- Navigation MUST work as expected

## Success Criteria

- Book successfully written using AI + spec-driven workflow
- Repository pushed to GitHub with clean commit history
- Docusaurus site deployed and publicly accessible via GitHub Pages
- Content is original, well-structured, and technically accurate
- Meets all Hackathon Phase 1 requirements

## Governance

### Amendment Procedure
- Constitution amendments MUST be proposed through feature specifications
- Changes MUST document rationale and impact analysis
- Amendments MUST propagate to all dependent templates

### Versioning
- Version follows semantic versioning (MAJOR.MINOR.PATCH)
- MAJOR: Backward incompatible governance/principle removals or redefinitions
- MINOR: New principle/section added or materially expanded guidance
- PATCH: Clarifications, wording, typo fixes, non-semantic refinements

### Compliance Review
- All features MUST verify compliance with constitution principles
- Violations MUST be justified in plan.md Complexity Tracking table
- PR reviews MUST check constitution alignment

### Runtime Guidance
- Use CLAUDE.md for agent-specific development guidance
- Constitution takes precedence over all other practices
- Ambiguity MUST be resolved before implementation

**Version**: 1.0.0 | **Ratified**: 2025-12-30 | **Last Amended**: 2025-12-30
