# OMEGA Stack

Canonical repository for OMEGA Stack products. Clone and run.

## Structure

```
omega-stack/
├── products/
│   ├── soft_robotics_lab/   # Gripper design, research memory, Streamlit lab
│   ├── omega_foundry/       # Gripper foundry outputs
│   ├── reality_bridge/      # Validation bridge, FastAPI
│   ├── world_model_studio/  # MuJoCo scenes, grasp scripts
│   ├── breakthrough_engine/ # Hypothesis ledger, weekly review
│   ├── omega_tutor/        # Chat tutor (LM Studio + Gemini)
│   └── enterprise/
│       ├── decision_brief/  # Decision brief (OMEGA-MAX ΦΩ)
│       └── eval_framework/ # Evaluation framework
├── scripts/
│   ├── bootstrap.sh        # Venv + base deps (Unix)
│   └── smoke_test.py       # Structure + import smoke test
├── docs/
├── Makefile
├── requirements.txt
└── README.md
```

## Quick start

1. **Clone** (after repo exists on GitHub):
   ```bash
   git clone https://github.com/YOUR_ORG/omega-stack.git
   cd omega-stack
   ```

2. **Bootstrap** (Unix/macOS or Git Bash):
   ```bash
   chmod +x scripts/bootstrap.sh
   ./scripts/bootstrap.sh
   source .venv/bin/activate
   ```
   Or on Windows (PowerShell):
   ```powershell
   python -m venv .venv
   .\.venv\Scripts\Activate.ps1
   pip install -r requirements.txt
   ```

3. **Smoke test**:
   ```bash
   python scripts/smoke_test.py
   ```

4. **Run a product** (each has its own `requirements.txt`):
   - **omega_tutor**: `cd products/omega_tutor && pip install -r requirements.txt && streamlit run app.py`
   - **soft_robotics_lab**: `cd products/soft_robotics_lab && streamlit run app.py`
   - **reality_bridge**: `cd products/reality_bridge && pip install -r requirements.txt && uvicorn app:app --reload`
   - **breakthrough_engine**: `cd products/breakthrough_engine && python hypothesis_ledger.py --help`
   - **enterprise/decision_brief**: `cd products/enterprise/decision_brief && python decision_brief.py --help`
   - **enterprise/eval_framework**: `cd products/enterprise/eval_framework && python omega_eval.py --help`

## Requirements

- Python 3.10+
- Product-specific: see `products/<name>/requirements.txt`

## License

Private. OMEGA Stack canonical repo.
