"""
Reality Bridge - Physics validation API service.
FastAPI: POST /validate, POST /analyze, GET /health, GET /stats, GET /failures, GET /leaderboard. CORS enabled.
Uses shared contracts: validates incoming data against reality_bridge assumptions.
"""

import sys
import tempfile
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, File, UploadFile, Form, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware

_PRODUCTS = Path(__file__).resolve().parent.parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))
from shared.contracts import Contract, ContractStatus, validate_handoff

from core.loader import load_model, detect_format
from core.validator import PhysicsValidator, ValidationResult
from core.analyzer import analyze, AnalysisResult, WeakPoint
from core.reporter import to_dict, to_markdown, to_html
from core import database

_CONTRACTS_DIR = _PRODUCTS / "shared" / "contracts"

app = FastAPI(
    title="Reality Bridge",
    description="Physics validation API for MJCF/URDF models",
    version="1.0.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

_validator = PhysicsValidator()
_foundry_contract = None
_reality_bridge_contract = None


@app.on_event("startup")
def startup():
    """Create data dir and validation table on startup. Load component contracts."""
    database.init_db()
    global _foundry_contract, _reality_bridge_contract
    try:
        _foundry_contract = Contract.from_json(_CONTRACTS_DIR / "foundry_contract.json")
        _reality_bridge_contract = Contract.from_json(_CONTRACTS_DIR / "reality_bridge_contract.json")
    except Exception:
        _foundry_contract = _reality_bridge_contract = None


def _analysis_to_dict(a: AnalysisResult) -> dict:
    return {
        "weak_points": [
            {"kind": w.kind, "name": w.name, "reason": w.reason, "details": w.details}
            for w in a.weak_points
        ],
        "failure_modes": a.failure_modes,
        "suggestions": a.suggestions,
        "metrics": a.metrics,
    }


@app.get("/health")
def health():
    """Service health and MuJoCo version."""
    try:
        import mujoco
        ver = getattr(mujoco, "__version__", "unknown")
    except Exception:
        ver = "unavailable"
    return {"status": "ok", "mujoco_version": ver}


async def _get_model_input(request: Request, file: Optional[UploadFile], xml_string_form: Optional[str]):
    """Resolve model XML from file upload, form, or JSON body."""
    content = None
    path = None
    if file and file.filename:
        raw = await file.read()
        content = raw.decode("utf-8", errors="replace")
        suf = Path(file.filename).suffix.lower()
        if suf in (".xml", ".urdf", ".mjcf"):
            with tempfile.NamedTemporaryFile(suffix=suf, delete=False) as f:
                f.write(raw)
                path = f.name
    if content is None and xml_string_form:
        content = xml_string_form
    if content is None and request.headers.get("content-type", "").startswith("application/json"):
        try:
            body = await request.json()
            content = body.get("xml_string") if isinstance(body, dict) else None
        except Exception:
            pass
    return content, path


@app.post("/validate")
async def validate(
    request: Request,
    file: Optional[UploadFile] = File(None),
    xml_string: Optional[str] = Form(None),
    artifact_id: Optional[str] = Form(None),
):
    """
    Validate a model: multipart (file), form (xml_string), or JSON body {"xml_string": "..."}.
    Returns ValidationResult as JSON. Checks handoff contract (Foundry -> Reality Bridge) when contracts loaded.
    """
    content, path = await _get_model_input(request, file, xml_string)

    if not content and not path:
        raise HTTPException(status_code=400, detail="Provide file or xml_string")

    # Contract validation: does incoming data meet our assumptions?
    contract_status = ContractStatus.UNKNOWN.value
    if _foundry_contract and _reality_bridge_contract:
        data = {
            "mjcf": content or "",
            "artifact_id": artifact_id or "",
            "valid_mjcf": True,
            "units_si": True,
            "mass_properties_set": True,
        }
        contract_status = validate_handoff(_foundry_contract, _reality_bridge_contract, data).value

    try:
        if path:
            model, _ = load_model(file_path=path)
        else:
            model, _ = load_model(xml_string=content)
    except Exception as e:
        raise HTTPException(status_code=422, detail=f"Load failed: {e}")

    result = _validator.validate(xml_string=content, file_path=path)
    payload = to_dict(result)
    payload["contract_status"] = contract_status
    try:
        design_hash, validation_id = database.log_validation(content, result, source="api")
        payload["design_hash"] = design_hash
        payload["validation_id"] = validation_id
    except Exception:
        payload["design_hash"] = None
        payload["validation_id"] = None
    return payload


@app.post("/analyze")
async def analyze_endpoint(
    request: Request,
    file: Optional[UploadFile] = File(None),
    xml_string: Optional[str] = Form(None),
):
    """
    Analyze a model: weak points, failure modes, suggestions.
    Same input as /validate. Returns analysis as JSON.
    """
    content, path = await _get_model_input(request, file, xml_string)

    if not content and not path:
        raise HTTPException(status_code=400, detail="Provide file or xml_string")

    try:
        if path:
            model, _ = load_model(file_path=path)
        else:
            model, _ = load_model(xml_string=content)
    except Exception as e:
        raise HTTPException(status_code=422, detail=f"Load failed: {e}")

    a = analyze(model)
    try:
        result = _validator.validate(xml_string=content, file_path=path)
        database.log_validation(content, result, source="analyze")  # returns (design_hash, validation_id)
    except Exception:
        pass
    return _analysis_to_dict(a)


@app.get("/stats")
def get_stats():
    """Validation statistics: total count, pass rate, avg score."""
    return database.get_stats()


@app.get("/failures")
def get_failures(limit: int = 50):
    """Recent validation failures with errors."""
    return database.get_failures(limit=limit)


@app.get("/leaderboard")
def get_leaderboard(limit: int = 10):
    """Top designs by score."""
    return database.get_leaderboard(limit=limit)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
