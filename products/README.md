# OMEGA Products

Product apps and tools in the OMEGA Stack.

## Single entry point: Omega Console

**Start here:** [Omega Console](omega_console/) is the single entry point for all OMEGA products.

```bash
cd products/omega_console && python -m streamlit run app.py --server.port 8500
```

Or from repo root: `make console`

From the Console you can:
- See status of all 6 products (Foundry, Reality Bridge, World Model Studio, Breakthrough Engine, Decision Brief, Tutor)
- Follow the **Golden Path**: Intent → Design → Validate → Learn → Decide
- Open any product (Streamlit apps or CLI)
- Use Quick Commands to start each service

## Products

| Product | Description |
|--------|-------------|
| **omega_console** | Single entry point — status, golden path, product launcher |
| **omega_foundry** | Design from natural language; physics-validated, fabrication-ready |
| **reality_bridge** | Physics validation API for MJCF/URDF (FastAPI, port 8000) |
| **world_model_studio** | MuJoCo scenes, grasp scripts, training (Streamlit) |
| **breakthrough_engine** | Hypothesis ledger, weekly review (CLI) |
| **omega_tutor** | Chat tutor — LM Studio + Gemini (Streamlit) |
| **enterprise/decision_brief** | OMEGA-MAX ΦΩ decision brief (CLI) |
| **enterprise/eval_framework** | Evaluation framework |
| **soft_robotics_lab** | Gripper design, research memory, Streamlit lab |

Each product has its own `requirements.txt`. Shared IDs: `products/shared/id_generator.py`.
