# PRODUCT.md — OMEGA Product Template

Copy this file into your product directory and fill in the sections. Use one PRODUCT.md per product (e.g. omega_foundry, reality_bridge).

---

## Overview

**Product name:**  
**Purpose:**  
**Port(s):** (e.g. 8504 for Streamlit, 8000 for API.)

One short paragraph: what the product does, who uses it, and how it fits in the OMEGA Stack (e.g. "Design from intent; generates MJCF/URDF; validates via Reality Bridge; governed by Guardian Runtime.")

---

## Components

| Component    | Path / module        | Responsibility                          |
|-------------|----------------------|-----------------------------------------|
| (e.g. Core) | `core/`              | (e.g. design engine, validation)        |
| (e.g. API)  | `app.py` or `api/`   | (e.g. Streamlit UI, FastAPI endpoints)  |
| (e.g. Data) | `data/`, `outputs/`  | (e.g. local storage, exports)           |

List main directories and what they contain. Reference `AGENTS.md` if the product has one or more agents.

---

## Guardian Policies

- **Does this product use Guardian Runtime?** Yes / No.
- **Policies:** (e.g. file_access, api_call, model_export.) Link to policy files or list rules.
- **Audit bundles:** Where stored? How long retained? Export to Substrate or shared audit store?

If the product does not yet use Guardian, list the actions that should be governed and the planned policy set.

---

## API (if any)

- **Type:** (e.g. REST FastAPI, Streamlit only, CLI.)
- **Base URL / entrypoint:** (e.g. `http://localhost:8000`, `streamlit run app.py --server.port 8504`.)
- **Key endpoints or flows:** (e.g. `POST /validate`, "Upload MJCF → validate → return report.")
- **Auth:** (e.g. none for MVP, API key, OAuth.) Document in product README.

---

## Testing

- **How to run tests:** (e.g. `PYTHONPATH=.. pytest tests/ -v` from product root, or `pytest products/<name>/tests/` from repo root.)
- **Required env:** (e.g. API keys in `.env`, optional MuJoCo.)
- **Coverage expectations:** (e.g. core logic and Guardian integration must be covered.)

---

## Deployment

- **Local:** (e.g. `streamlit run app.py`, `uvicorn app:app --port 8000`.)
- **Env vars:** (e.g. `OPENAI_API_KEY`, `GUARDIAN_POLICIES_PATH`.)
- **Data directories:** (e.g. `data/`, `outputs/` — ensure writable and optionally backed up.)

---

*Template: [products/templates/PRODUCT.md](PRODUCT.md)*
