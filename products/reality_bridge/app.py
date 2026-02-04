"""
Reality Bridge - Physics validation API service.
FastAPI: POST /validate, POST /validate/batch, POST /analyze, GET /health, GET /stats, GET /failures, GET /leaderboard,
GET /validations/history/{design_id}, POST/DELETE/GET /webhooks. CORS enabled.
Uses shared contracts and failure taxonomy: no raw tracebacks to users.
"""

import asyncio
import base64
import os
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, File, UploadFile, Form, Request, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

_PRODUCTS = Path(__file__).resolve().parent.parent
if str(_PRODUCTS) not in sys.path:
    sys.path.insert(0, str(_PRODUCTS))
from shared.contracts import Contract, ContractStatus, validate_handoff
from shared.failures import create_failure, handle_exception, FailureCode
from shared.audit import create_bundle
from shared.trust import TrustScoreTracker
try:
    from shared.substrate import vector_store, knowledge_graph, lineage_graph
    from shared.substrate.knowledge_graph import NodeType, EdgeType
except ImportError:
    vector_store = knowledge_graph = lineage_graph = None
    NodeType = EdgeType = None

from core.loader import load_model, detect_format
from core.validator import PhysicsValidator, ValidationResult
from core.analyzer import analyze, AnalysisResult, WeakPoint
from core.reporter import to_dict, to_markdown, to_html
from core import database
from core.fix_suggestions import failure_from_validation_result
from core.prescriptive_fixer import generate_prescriptive_fixes, FixReport
from core.design_comparator import compare_designs, ComparisonReport
from core import webhooks as webhooks_module

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
    Returns ValidationResult as JSON or structured failure (no raw tracebacks).
    """
    content, path = await _get_model_input(request, file, xml_string)

    if not content and not path:
        failure = create_failure(FailureCode.MISSING_PARAMS, details="Provide file or xml_string")
        return JSONResponse(
            status_code=400,
            content={"success": False, "failure": failure.to_dict(), "message": failure.to_user_message()},
        )

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
        failure = handle_exception(e, context="validate")
        TrustScoreTracker().record_run(had_traceback=False)
        return JSONResponse(
            status_code=422,
            content={"success": False, "failure": failure.to_dict(), "message": failure.to_user_message()},
        )

    t0 = time.perf_counter()
    result = _validator.validate(xml_string=content, file_path=path)
    validation_time_ms = int((time.perf_counter() - t0) * 1000)
    payload = to_dict(result)
    payload["success"] = True
    payload["contract_status"] = contract_status
    payload["validation_time_ms"] = validation_time_ms
    if not result.passed:
        fail_info = failure_from_validation_result(result)
        if fail_info:
            payload["failures"] = [{
                "code": fail_info["code"],
                "message": fail_info["message"],
                "suggestions": fail_info["suggestions"],
                "tutor_link": fail_info.get("tutor_link"),
            }]
    try:
        design_hash, vid = database.log_validation(content, result, source="api", validation_time_ms=validation_time_ms, artifact_id=artifact_id)
        payload["design_hash"] = design_hash
        payload["validation_id"] = vid
    except Exception:
        payload["design_hash"] = None
        payload["validation_id"] = None

    # Substrate: vector store, lineage, knowledge graph (if available)
    aid = artifact_id or payload.get("design_hash") or "unknown"
    if vector_store and knowledge_graph and lineage_graph and NodeType and EdgeType:
        try:
            vector_store.add(
                "designs",
                content[:50000],
                {"artifact_id": aid, "passed": result.passed, "score": result.score, "design_hash": payload.get("design_hash")},
            )
            vid = payload.get("validation_id")
            if vid:
                lineage_graph.record(vid, aid, "validation", {"passed": result.passed, "score": result.score})
            knowledge_graph.add_node(NodeType.Design, aid, {"design_hash": payload.get("design_hash"), "passed": result.passed, "score": result.score})
            if vid:
                knowledge_graph.add_node(NodeType.Validation, vid, {"passed": result.passed, "score": result.score})
                knowledge_graph.add_edge(vid, aid, EdgeType.validates, {"passed": result.passed, "score": result.score})
                if not result.passed and payload.get("failures"):
                    fc = payload["failures"][0].get("code", "UNKNOWN")
                    if not knowledge_graph.get_node(fc):
                        knowledge_graph.add_node(NodeType.Concept, fc, {"name": fc})
                    knowledge_graph.add_edge(vid, fc, EdgeType.related_to, {"reason": "has_failure"})
        except Exception:
            pass

    # Webhooks
    try:
        event = "validation_failed" if not result.passed else "validation_completed"
        asyncio.create_task(webhooks_module.trigger_webhook(event, {
            "validation_id": payload.get("validation_id"),
            "artifact_id": aid,
            "passed": result.passed,
            "score": result.score,
            "design_hash": payload.get("design_hash"),
            "failures": payload.get("failures", []),
        }))
    except Exception:
        pass

    # Audit bundle for reproducibility (artifact = MJCF, contract, validation)
    try:
        artifact = {"mjcf": content, "design_hash": payload.get("design_hash")}
        contract_dict = _reality_bridge_contract.to_dict() if _reality_bridge_contract else None
        bundle = create_bundle(
            component="reality_bridge",
            artifact=artifact,
            artifact_id=artifact_id,
            validation_id=payload.get("validation_id"),
            contract=contract_dict,
            validation=payload,
        )
        zip_bytes = bundle.save_to_bytes()
        payload["bundle_base64"] = base64.b64encode(zip_bytes).decode("ascii")
        payload["bundle_filename"] = f"audit_bundle_{bundle.metadata.bundle_id}.zip"
    except Exception:
        payload["bundle_base64"] = None
        payload["bundle_filename"] = None

    TrustScoreTracker().record_run(had_traceback=False)
    return payload


@app.post("/analyze")
async def analyze_endpoint(
    request: Request,
    file: Optional[UploadFile] = File(None),
    xml_string: Optional[str] = Form(None),
):
    """
    Analyze a model: weak points, failure modes, suggestions.
    Same input as /validate. Returns analysis as JSON or structured failure.
    """
    content, path = await _get_model_input(request, file, xml_string)

    if not content and not path:
        failure = create_failure(FailureCode.MISSING_PARAMS, details="Provide file or xml_string")
        TrustScoreTracker().record_run(had_traceback=False)
        return JSONResponse(
            status_code=400,
            content={"success": False, "failure": failure.to_dict(), "message": failure.to_user_message()},
        )

    try:
        if path:
            model, _ = load_model(file_path=path)
        else:
            model, _ = load_model(xml_string=content)
    except Exception as e:
        failure = handle_exception(e, context="analyze")
        TrustScoreTracker().record_run(had_traceback=False)
        return JSONResponse(
            status_code=422,
            content={"success": False, "failure": failure.to_dict(), "message": failure.to_user_message()},
        )

    a = analyze(model)
    try:
        result = _validator.validate(xml_string=content, file_path=path)
        database.log_validation(content, result, source="analyze")  # returns (design_hash, validation_id)
    except Exception:
        pass
    TrustScoreTracker().record_run(had_traceback=False)
    return {"success": True, **_analysis_to_dict(a)}


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


@app.post("/validate/batch")
async def validate_batch(request: Request):
    """
    Validate multiple designs. Body: {"designs": [{"mjcf": "...", "artifact_id": "..."}, ...]}.
    Returns { "results": [...], "summary": { "total", "passed", "failed" } }. Triggers batch_completed webhook.
    """
    try:
        body = await request.json()
        designs = body.get("designs") or []
    except Exception:
        return JSONResponse(status_code=400, content={"success": False, "message": "JSON body with 'designs' array required"})
    if not designs:
        return JSONResponse(status_code=400, content={"success": False, "message": "designs array must not be empty"})
    results: List[Dict[str, Any]] = []
    for i, item in enumerate(designs):
        content = item.get("mjcf") or item.get("xml_string") or ""
        artifact_id = item.get("artifact_id") or f"batch-{i}"
        if not content:
            results.append({"artifact_id": artifact_id, "success": False, "error": "Missing mjcf/xml_string"})
            continue
        try:
            model, _ = load_model(xml_string=content)
        except Exception as e:
            results.append({"artifact_id": artifact_id, "success": False, "error": str(e)})
            continue
        t0 = time.perf_counter()
        result = _validator.validate(xml_string=content, file_path=None)
        validation_time_ms = int((time.perf_counter() - t0) * 1000)
        payload = to_dict(result)
        payload["success"] = True
        payload["artifact_id"] = artifact_id
        payload["validation_time_ms"] = validation_time_ms
        if not result.passed:
            fail_info = failure_from_validation_result(result)
            if fail_info:
                payload["failures"] = [{"code": fail_info["code"], "message": fail_info["message"], "suggestions": fail_info["suggestions"], "tutor_link": fail_info.get("tutor_link")}]
        try:
            design_hash, vid = database.log_validation(content, result, source="batch", validation_time_ms=validation_time_ms, artifact_id=artifact_id)
            payload["design_hash"] = design_hash
            payload["validation_id"] = vid
        except Exception:
            payload["design_hash"] = None
            payload["validation_id"] = None
        results.append(payload)
    summary = {"total": len(results), "passed": sum(1 for r in results if r.get("passed", False)), "failed": sum(1 for r in results if not r.get("passed", True))}
    try:
        asyncio.create_task(webhooks_module.trigger_webhook("batch_completed", {"summary": summary, "results_count": len(results)}))
    except Exception:
        pass
    return {"success": True, "results": results, "summary": summary}


@app.get("/validations/history/{design_id}")
def get_validation_history(design_id: str, limit: int = 50):
    """All validations for a design (by artifact_id or design_hash)."""
    return database.get_validation_history(design_id, limit=limit)


@app.get("/validations/recent")
def get_recent_validations(limit: int = 20):
    """Recent validations for dashboard."""
    return database.get_recent_validations(limit=limit)


def _fix_report_to_dict(report: FixReport) -> dict:
    """Serialize FixReport to JSON-serializable dict."""
    return {
        "design_id": report.design_id,
        "failure_codes": report.failure_codes,
        "fixes": [
            {
                "fix_type": f.fix_type.value,
                "component": f.component,
                "parameter": f.parameter,
                "current_value": f.current_value,
                "suggested_value": f.suggested_value,
                "unit": f.unit,
                "confidence": f.confidence,
                "reasoning": f.reasoning,
                "priority": f.priority,
            }
            for f in report.fixes
        ],
        "estimated_improvement": report.estimated_improvement,
        "warnings": report.warnings,
    }


def _comparison_report_to_dict(comp: ComparisonReport) -> dict:
    """Serialize ComparisonReport to JSON-serializable dict."""
    return {
        "design_a_id": comp.design_a_id,
        "design_b_id": comp.design_b_id,
        "task": comp.task,
        "overall_winner": comp.overall_winner,
        "confidence": comp.confidence,
        "metric_comparisons": [
            {
                "metric": c.metric.value,
                "design_a_score": c.design_a_score,
                "design_b_score": c.design_b_score,
                "winner": c.winner,
                "difference": c.difference,
                "significance": c.significance,
                "notes": c.notes,
            }
            for c in comp.metric_comparisons
        ],
        "design_a_strengths": comp.design_a_strengths,
        "design_b_strengths": comp.design_b_strengths,
        "recommendation": comp.recommendation,
        "detailed_analysis": comp.detailed_analysis,
    }


@app.post("/fixes")
async def prescriptive_fixes_endpoint(request: Request):
    """
    Generate prescriptive fixes from a validation result.
    Body: {"validation_result": {...}, "design_id": "..."}.
    validation_result: output from /validate or to_dict(ValidationResult).
    Returns FixReport as JSON.
    """
    try:
        body = await request.json()
        validation_result = body.get("validation_result")
        design_id = body.get("design_id", "unknown")
    except Exception:
        return JSONResponse(
            status_code=400,
            content={"success": False, "message": "JSON body with validation_result required"},
        )
    if not validation_result or not isinstance(validation_result, dict):
        return JSONResponse(
            status_code=400,
            content={"success": False, "message": "validation_result must be a dict from /validate"},
        )
    try:
        report = generate_prescriptive_fixes(validation_result, design_id=design_id)
        return {"success": True, **_fix_report_to_dict(report)}
    except Exception as e:
        failure = handle_exception(e, context="fixes")
        return JSONResponse(
            status_code=500,
            content={"success": False, "failure": failure.to_dict(), "message": failure.to_user_message()},
        )


@app.post("/compare")
async def compare_designs_endpoint(request: Request):
    """
    Compare two designs for a task. Validates both then compares.
    Body: {"design_a": {"mjcf": "...", "id": "..."}, "design_b": {"mjcf": "...", "id": "..."}, "task": "general"}.
    Returns ComparisonReport as JSON.
    """
    try:
        body = await request.json()
        design_a = body.get("design_a") or {}
        design_b = body.get("design_b") or {}
        task = body.get("task", "general")
    except Exception:
        return JSONResponse(
            status_code=400,
            content={"success": False, "message": "JSON body with design_a, design_b required"},
        )
    mjcf_a = (design_a.get("mjcf") or design_a.get("xml_string") or "").strip()
    mjcf_b = (design_b.get("mjcf") or design_b.get("xml_string") or "").strip()
    if not mjcf_a or not mjcf_b:
        return JSONResponse(
            status_code=400,
            content={"success": False, "message": "design_a and design_b must include mjcf or xml_string"},
        )
    id_a = design_a.get("id", "Design A")
    id_b = design_b.get("id", "Design B")
    try:
        result_a = _validator.validate(xml_string=mjcf_a)
        result_b = _validator.validate(xml_string=mjcf_b)
    except Exception as e:
        failure = handle_exception(e, context="compare")
        return JSONResponse(
            status_code=422,
            content={"success": False, "failure": failure.to_dict(), "message": failure.to_user_message()},
        )
    validation_a = to_dict(result_a)
    validation_b = to_dict(result_b)
    comparison = compare_designs(
        design_a={"id": id_a, "mjcf": mjcf_a},
        design_b={"id": id_b, "mjcf": mjcf_b},
        validation_a=validation_a,
        validation_b=validation_b,
        task=task,
    )
    return {"success": True, **_comparison_report_to_dict(comparison)}


@app.post("/webhooks")
async def register_webhook_endpoint(request: Request):
    """Register a webhook. Body: {"url": "...", "events": ["validation_completed", "validation_failed", "batch_completed"], "secret": "..."}. Returns webhook_id."""
    try:
        body = await request.json()
        url = body.get("url") or ""
        events = body.get("events") or ["validation_completed", "validation_failed"]
        secret = body.get("secret")
    except Exception:
        return JSONResponse(status_code=400, content={"success": False, "message": "JSON body with url and events required"})
    if not url:
        return JSONResponse(status_code=400, content={"success": False, "message": "url required"})
    wid = webhooks_module.register_webhook(url, events, secret)
    return {"success": True, "webhook_id": wid}


@app.get("/webhooks")
def list_webhooks_endpoint():
    """List registered webhooks (id, url, events, active; no secret)."""
    return {"webhooks": webhooks_module.list_webhooks()}


@app.delete("/webhooks/{webhook_id}")
def remove_webhook_endpoint(webhook_id: str):
    """Remove webhook by id."""
    if webhooks_module.remove_webhook(webhook_id):
        return {"success": True, "message": "Webhook removed"}
    raise HTTPException(status_code=404, detail="Webhook not found")


if __name__ == "__main__":
    import uvicorn
    # Port from env for OMEGA port convention (18xxx); default 18000
    port = int(os.environ.get("REALITY_BRIDGE_PORT", "18000"))
    uvicorn.run(app, host="0.0.0.0", port=port)
