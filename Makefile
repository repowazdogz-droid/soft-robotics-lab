# OMEGA Stack — canonical repo
# Run from repo root

.PHONY: bootstrap smoke install help console

help:
	@echo "OMEGA Stack"
	@echo "  make bootstrap   - Create venv and install root requirements"
	@echo "  make smoke       - Run smoke_test.py"
	@echo "  make install     - Same as bootstrap"
	@echo "  make console     - Run Omega Console (streamlit port 8500)"

bootstrap:
	@if command -v python3 >/dev/null 2>&1; then \
		python3 -m venv .venv 2>/dev/null || true; \
		. .venv/bin/activate 2>/dev/null || . .venv/Scripts/activate; \
		pip install -U pip; pip install -r requirements.txt; \
	else \
		echo "Run: python -m venv .venv && .venv\\Scripts\\activate && pip install -r requirements.txt"; \
	fi

install: bootstrap

smoke:
	python scripts/smoke_test.py

# Windows-friendly (use from PowerShell or Git Bash)
bootstrap-win:
	python -m venv .venv
	.venv\Scripts\activate && pip install -U pip && pip install -r requirements.txt

console:
	cd products/omega_console && python -m streamlit run app.py --server.port 8500
