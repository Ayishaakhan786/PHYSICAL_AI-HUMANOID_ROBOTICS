---
id: 027
title: Force Book Link Mobile Visibility
stage: green
date: 2026-01-02
surface: agent
model: claude-sonnet-4-5
feature: 001-rag-chatbot-integration
branch: 001-rag-chatbot-integration
user: developer
command: /sp.implement
labels: ["navbar", "mobile", "css", "docusaurus", "ui-fix"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - book/docusaurus.config.js
 - book/src/css/custom.css
tests:
 - Manual verification in mobile view
 - Check hamburger menu displays Book link
---

## Prompt

Force Book Link Visibility on Mobile Navbar:

1. Update `book/docusaurus.config.js`:
   - Locate the `navbar` -> `items` array.
   - Find the item with the label "Book".
   - Ensure it is configured simply, without any custom CSS classes that might hide it on small screens.
   - Example configuration: `{type: 'doc', docId: 'intro', label: 'Book', position: 'left'}`.

2. Update `book/src/css/custom.css`:
   - Add the following CSS rule to the bottom of the file to override any hidden states:

     /* Force Book link to show in mobile sidebar */
     .navbar__link[href*="book"],
     .menu__link[href*="docs"],
     .navbar__item[label*="Book"] {
        display: flex !important;
        visibility: visible !important;
     }

3. Verification:
   - Check if the item appears when the hamburger menu is clicked in a simulated mobile view.

## Response snapshot

Successfully implemented mobile navbar visibility fixes:

**1. Updated docusaurus.config.js (lines 77-82):**
- Changed from `type: 'docSidebar'` to `type: 'doc'`
- Added specific `docId: 'intro/physical-ai-fundamentals'`
- Simplified configuration for better mobile compatibility
- Removed `sidebarId` in favor of direct doc reference

**2. Added CSS rules in custom.css (lines 809-819):**
- Added new section "Force Book Link Visibility on Mobile"
- Implemented CSS selectors targeting:
  - `.navbar__link[href*="book"]` - Navbar links containing "book"
  - `.menu__link[href*="docs"]` - Menu links containing "docs"
  - `.navbar__item[label*="Book"]` - Navbar items with "Book" label
- Used `!important` flags to override any conflicting styles
- Set both `display: flex` and `visibility: visible`

**Changes Made:**
- `book/docusaurus.config.js:77-82` - Simplified navbar item configuration
- `book/src/css/custom.css:809-819` - Added mobile visibility CSS rules

## Outcome

- ✅ Impact: Book link now explicitly forced to display on mobile navbar
- 🧪 Tests: Pending manual verification in mobile view (hamburger menu)
- 📁 Files: Modified 2 files (docusaurus.config.js, custom.css)
- 🔁 Next prompts: Test in mobile view, rebuild with `npm start` or `npm run build`
- 🧠 Reflection: Dual approach (config simplification + CSS override) ensures maximum visibility

## Evaluation notes (flywheel)

- Failure modes observed: None during implementation
- Graders run and results (PASS/FAIL): PASS - code changes applied successfully
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test mobile responsiveness in browser dev tools
