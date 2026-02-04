# Soft Robotics Lab — Institution-Ready Upgrade Plan

## Change plan (file targets)

| # | Deliverable | File(s) | Status |
|---|-------------|---------|--------|
| 1 | First Run Setup (lab name, first hypothesis, Generate Zoo, Weekly Brief explanation) | `pages/0_First_Run_Setup.py` (new), default lab in 5 & 6 | Done |
| 2 | MuJoCo validation in Gripper Design | `pages/1_Gripper_Design.py` (Validate in MuJoCo button, result display) | Done |
| 3 | Weekly Brief empty state (CTA + create hypothesis button) | `pages/6_Weekly_Brief.py` | Done |
| 4 | Semantic search UI in Research Memory (Keyword vs Semantic, add to hypothesis evidence) | `pages/5_Research_Memory.py` (Search tab) | Done |
| 5 | Hypothesis spine (link design to hypothesis; Failure Analysis add evidence) | `pages/1_Gripper_Design.py`, `pages/2_Failure_Analysis.py`, `research_system/research_memory.py` (add_evidence_note) | Done |
| 6 | One-click Zoo generation on Zoo page if missing | `pages/4_Gripper_Zoo.py` | Done |
| 7 | Portability: relative paths in index.json | `gripper_zoo/generate_zoo.py` | Done |
| 8 | Export Bundle (.zip) (design JSON, sim exports, README, optional STL) | `pages/1_Gripper_Design.py` | Done |
| 9 | Assumptions / Operating Envelope panel per design | `pages/1_Gripper_Design.py` | Done |
| 10 | Top-of-page status banner (Zoo | Embeddings | MuJoCo) | `utils/status.py` (new), `app.py` | Done |

## Implemented details

- **Status banner:** `utils/status.py` provides `get_system_status()` and `render_status_banner(st)`; used on home and First Run Setup.
- **First Run Setup:** Lab name (default `demo_lab`), create first hypothesis, Generate Zoo button (runs `generate_zoo` in-process), Weekly Brief explanation. Persists `lab_name` in `st.session_state`.
- **Gripper Design:** Assumptions/Operating Envelope expander; after export: Validate in MuJoCo (if MuJoCo available), Export Bundle (.zip) with README; link design to hypothesis (writes `research_system/data/{lab}/design_hypothesis_links.json`).
- **Weekly Brief:** If no hypotheses, single CTA + “Create your first hypothesis” button → First Run Setup. Default lab `demo_lab`.
- **Research Memory:** New “Search” tab with Keyword vs Semantic toggle, `memory.search(..., use_semantic=...)`, results with “Add to hypothesis evidence” (hypothesis + supports/weakens). Default lab `demo_lab`; sidebar lab name synced to `st.session_state["lab_name"]`.
- **Failure Analysis:** “Record as evidence” section: select hypothesis, supports/weakens, optional summary; calls `memory.add_evidence_note(hyp_id, note_text, direction)`.
- **Research Memory backend:** `add_evidence_note(hyp_id, note_text, direction, source_label)` creates a one-chunk evidence doc and links it to the hypothesis.
- **Gripper Zoo:** If `index.json` missing, “Generate Gripper Zoo” button runs `generate_zoo` in-process and reruns. `generate_zoo.py` writes relative paths in `designs[].exports` (e.g. `gd_xxx/gd_xxx.json`).

## Portability and logging

- Index and design exports use paths relative to `gripper_zoo/designs/`.
- No absolute paths written in `index.json`.
- Minimal logging: success/error messages in UI (e.g. “Generated N designs”, “Evidence recorded”). No new log files added.
