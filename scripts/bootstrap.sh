#!/usr/bin/env bash
# OMEGA Stack bootstrap: venv + base deps. Run from repo root.
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

if command -v python3 &>/dev/null; then
  PY=python3
elif command -v python &>/dev/null; then
  PY=python
else
  echo "No Python found. Install Python 3.10+."
  exit 1
fi

echo "Using: $($PY --version)"
if [ ! -d ".venv" ]; then
  "$PY" -m venv .venv
  echo "Created .venv"
fi
# shellcheck disable=SC1091
source .venv/bin/activate 2>/dev/null || source .venv/Scripts/activate
pip install -U pip
pip install -r requirements.txt
echo "Bootstrap done. Activate with: source .venv/bin/activate (or .venv\\Scripts\\activate on Windows)"
