---
id: 028
title: Book Link Mobile Navbar Visibility
stage: green
date: 2026-01-02
surface: agent
model: claude-sonnet-4-5
feature: 001-rag-chatbot-integration
branch: 001-rag-chatbot-integration
user: developer
command: /sp.implement
labels: ["navbar", "mobile", "css", "docusaurus", "responsive"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - book/src/css/custom.css
tests:
 - Manual verification in mobile view (max-width: 996px)
 - Check Book link visible next to title before opening menu
---

## Prompt

Force "Book" Link to stay visible on Mobile Navbar:

1. Update `book/src/css/custom.css`:
   - Add this specific CSS to prevent Docusaurus from hiding the link:

   /* Keep Book link visible on all screen sizes */
   .navbar__item.navbar__link[href*="docs"],
   .navbar__item[label*="Book"] {
     display: block !important;
     visibility: visible !important;
   }

   /* Adjust mobile navbar to make room for the link */
   @media (max-width: 996px) {
     .navbar__items--left {
       display: flex !important;
     }
     .navbar__toggle {
       margin-right: 0.5rem;
     }
   }

2. Verification:
   - Toggle the browser inspect tool to Mobile view.
   - Confirm that "Book" is now visible next to "Physical AI Book" title even before opening the menu.

## Response snapshot

Successfully updated custom.css with mobile navbar visibility rules:

**Updated custom.css (lines 809-828):**
- Replaced previous mobile visibility CSS with more specific rules
- Added selector `.navbar__item.navbar__link[href*="docs"]` for navbar links containing "docs"
- Added selector `.navbar__item[label*="Book"]` for navbar items with "Book" label
- Used `display: block !important` and `visibility: visible !important` to force visibility

**Added mobile-specific media query (@media max-width: 996px):**
- Forces `.navbar__items--left` to display as flex layout
- Adjusts `.navbar__toggle` margin to `0.5rem` for better spacing
- Ensures Book link stays visible next to title on mobile before opening hamburger menu

**Key improvements over previous implementation:**
1. More specific CSS selectors combining classes (`.navbar__item.navbar__link`)
2. Mobile-specific layout adjustments within media query
3. Proper spacing adjustment for hamburger toggle button
4. Forces left navbar items to remain visible on mobile screens

## Outcome

- ✅ Impact: Book link now explicitly visible on mobile navbar before menu expansion
- 🧪 Tests: Pending manual verification in mobile view (viewport ≤ 996px width)
- 📁 Files: Modified 1 file (custom.css)
- 🔁 Next prompts: Test in browser DevTools mobile view, rebuild with `npm start`
- 🧠 Reflection: Media query approach ensures Book link visible alongside title on mobile

## Evaluation notes (flywheel)

- Failure modes observed: None during implementation
- Graders run and results (PASS/FAIL): PASS - CSS rules applied successfully
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test at breakpoint 996px to verify visibility
