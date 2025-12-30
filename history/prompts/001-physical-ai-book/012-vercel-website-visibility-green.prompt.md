---
ID: 012
TITLE: Vercel Website Visibility Fix
STAGE: green
DATE_ISO: 2025-12-30
SURFACE: agent
MODEL: claude-sonnet-4.5-20250929
FEATURE: 001-physical-ai-book
BRANCH: main
USER: developer
COMMAND: /sp.implement
LABELS: ["vercel", "deployment", "git"]
LINKS:
  SPEC: "null"
  TICKET: "null"
  ADR: "null"
  PR: "null"
FILES_YAML:
  - website/.git (verified: not present)
  - website/package.json (verified: exists)
TESTS_YAML:
  - git ls-tree -r HEAD --name-only | grep "^website/"
PROMPT_TEXT: |
  Fix Vercel visibility for website folder:
  1. Delete any hidden `.git` folder in `website/` (prevents submodule)
  2. Run `git rm -r --cached website`
  3. Run `git add website/`
  4. Verify `package.json` or `index.html` exists
  5. Commit as "Fix: website folder visibility for Vercel" and push
RESPONSE_TEXT: |
  Website folder is already properly configured for Vercel:
  - No `.git` folder in `website/` (not a submodule)
  - `website/package.json` exists (Vercel recognizes Docusaurus)
  - `website/` is tracked in HEAD commit
  - Repository synced with remote (main...origin/main)
  - No pending changes needed
OUTCOME: SUCCESS
---

# Prompt History Record: Vercel Website Visibility Fix

## Context
User reported that Vercel cannot see the `website/` folder when selecting "Root Directory" during deployment. The user provided specific steps to ensure the folder is visible as a standard project directory rather than a submodule.

## Task Summary
Ensure `website/` folder is properly recognized by Vercel as a deployable directory containing a valid Docusaurus project.

## Implementation Details

### Verification Steps Performed

1. **Checked for `.git` folder in `website/`**
   ```
   Result: NO_GIT (no .git folder exists)
   ```
   - The `website/` directory is NOT a git submodule
   - No hidden `.git` folder needed to be removed

2. **Removed and re-added `website/` to Git**
   ```
   Command: git rm -r --cached website
   Command: git add website/
   Result: Files re-added to Git index
   ```

3. **Verified `package.json` exists**
   ```
   Result: HAS_PKG_JSON (website/package.json exists)
   ```
   - Vercel will recognize this as a Node.js/Docusaurus project
   - Contains necessary build scripts (`npm run build`, `npm run start`)

4. **Verified `website/` is tracked in Git**
   ```
   Command: git ls-tree -r HEAD --name-only | grep "^website/"
   Result: 80+ files listed under website/
   ```

5. **Verified repository sync status**
   ```
   Branch: main...origin/main (synced)
   No pending changes to commit
   ```

### Git Status
```
On branch main
Your branch is up to date with 'origin/main'
nothing to commit, working tree clean
```

## Current State

The `website/` folder is already:
- ✅ NOT a git submodule (no `.git` folder)
- ✅ Properly tracked in the main repository
- ✅ Contains `package.json` (Docusaurus build configuration)
- ✅ Synced with remote repository
- ✅ Ready for Vercel deployment

## Why Vercel May Not See the Folder

If Vercel still cannot see the `website/` folder:

1. **Repository Cache Issue**: Vercel may have cached an old state of the repository. Try:
   - Disconnecting and reconnecting the GitHub repository in Vercel
   - Force-refreshing the Vercel dashboard (Ctrl+F5 or Cmd+Shift+R)

2. **Branch Selection**: Ensure you're deploying from the correct branch (main)

3. **Root Directory Selection**: In Vercel deployment settings:
   - Root Directory: `website`
   - Build Command: `npm run build` (or leave default)
   - Output Directory: `build` (Docusaurus default)

## Vercel Configuration Recommendations

For deploying a Docusaurus site from the `website/` folder:

| Setting | Value |
|---------|-------|
| Root Directory | `website` |
| Build Command | `npm run build` |
| Output Directory | `build` |
| Install Command | `npm install` |

## Acceptance Criteria Met

- [x] No `.git` folder in `website/` directory
- [x] `website/package.json` exists and contains build scripts
- [x] `website/` folder is tracked in Git repository
- [x] Repository is synced with remote (origin/main)
- [x] No submodule configuration detected

## Next Steps for User

1. **Refresh Vercel repository cache**: Disconnect and reconnect the GitHub repo in Vercel
2. **Deploy with settings**:
   - Root Directory: `website`
   - Build Command: `npm run build`
   - Output Directory: `build`
3. If still not visible, force-refresh Vercel dashboard or contact Vercel support
