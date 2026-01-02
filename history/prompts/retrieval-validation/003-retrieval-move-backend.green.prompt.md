---
id: "003"
title: "Move retrieve.py to backend"
stage: "green"
date: "2025-12-31"
surface: "agent"
model: "sonnet-4-5-20250929"
feature: "retrieval-validation"
branch: "2-retrieval-validation"
user: "sp.implement"
command: "sp.implement"
labels:
  - "rag"
  - "retrieval"
  - "refactor"
links:
  spec: "specs/2-retrieval-validation/spec.md"
  ticket: null
  adr: null
  pr: null
files:
  - "backend/retrieve.py"
tests: []
---

# Prompt Text

Please move `retrieve.py` from the root directory into the `backend/` folder.
1. After moving, update `retrieve.py` to ensure it correctly loads the `.env` file from the root directory (use `../.env`).
2. Make sure it can still find the local modules if it imports anything from the backend.
3. Also, modify the script to print the "Score" and "Text" of the top 3 results for every query so we can debug why it's failing.

# Response Text

Moved `retrieve.py` to `backend/` and updated with debug output.

**Changes:**
1. Moved `retrieve.py` from root to `backend/retrieve.py`
2. Updated `.env` path loading to use `Path(__file__).parent.parent / ".env"` (root .env from backend/)
3. Added UTF-8 encoding fix for Windows (`sys.platform == "win32"`)
4. Modified report to always include `details` with Score, Text (200 chars), and URL for top 3 results
5. Added `full_results` array with all results for reference
6. Updated print loop to display Score and Text for each top 3 result

**Debug Output Example:**
```
[PASS] humanoid robot kinematics... (5 results, 788.60ms)

    Top 3 Results:
      [1] Score: 0.6727
          Text: On this page Learning Objectives  Understand forward and inverse kinematics for humanoids Describe humanoid joint structure and degrees of freedom Ex...
          URL: https://physical-ai-humanoid-robotics-3mrv.vercel.app/docs/modules/module-6-humanoid/humanoid-kinematics
```

**Usage:**
```bash
cd backend && python retrieve.py
```
