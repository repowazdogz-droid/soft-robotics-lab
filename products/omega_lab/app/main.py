"""OMEGA Lab Cognition Server — FastAPI app."""

import json
from datetime import datetime, timedelta
from typing import Annotated

from fastapi import Depends, FastAPI, HTTPException
from sqlalchemy.orm import Session

from app import config, models
from app.artifact_store import get_artifacts_path, store_artifacts
from app.database import get_db, init_db
from app.hypothesis_engine import update_confidence
from app.models import Evidence, Hypothesis, Run
from app.registry import index_run, add_surface_point
from app.run_graph import build_run_graph, get_hypothesis_tree, export_graph_for_viz
from app.semantic_query import natural_query, query_summary, QueryFilter, query_runs
from app.nightly_digest import generate_digest
from app.schemas import (
    EvidenceResponse,
    HypothesisCreate,
    HypothesisResponse,
    RunBundle,
    RunResponse,
)

app = FastAPI(title="OMEGA Lab Cognition Server")


@app.on_event("startup")
def startup():
    """Create DB tables and ensure dirs exist."""
    init_db()
    config.RUNS_DIR.mkdir(parents=True, exist_ok=True)
    config.ARTIFACTS_DIR.mkdir(parents=True, exist_ok=True)


@app.get("/health")
def health():
    """Liveness check."""
    return {"status": "alive", "timestamp": datetime.utcnow().isoformat() + "Z"}


@app.post("/hypotheses", response_model=HypothesisResponse)
def create_hypothesis(
    body: HypothesisCreate,
    db: Annotated[Session, Depends(get_db)],
):
    """Create a new hypothesis."""
    existing = db.get(Hypothesis, body.id)
    if existing:
        raise HTTPException(status_code=409, detail=f"Hypothesis {body.id} already exists")
    confidence = max(0.05, min(0.95, body.confidence))
    h = Hypothesis(
        id=body.id,
        claim=body.claim,
        status="active",
        confidence=confidence,
        schema_version=getattr(body, "schema_version", "1.0"),
    )
    db.add(h)
    db.commit()
    db.refresh(h)
    return HypothesisResponse(
        id=h.id,
        claim=h.claim,
        status=h.status,
        confidence=h.confidence,
        schema_version=getattr(h, "schema_version", "1.0"),
        created_at=h.created_at,
        updated_at=h.updated_at,
    )


@app.get("/hypotheses", response_model=list[HypothesisResponse])
def list_hypotheses(db: Annotated[Session, Depends(get_db)]):
    """List all hypotheses."""
    rows = db.query(Hypothesis).all()
    return [
        HypothesisResponse(
            id=h.id,
            claim=h.claim,
            status=h.status,
            confidence=h.confidence,
            schema_version=getattr(h, "schema_version", "1.0"),
            created_at=h.created_at,
            updated_at=h.updated_at,
        )
        for h in rows
    ]


@app.get("/hypotheses/{id}", response_model=HypothesisResponse)
def get_hypothesis(
    id: str,
    db: Annotated[Session, Depends(get_db)],
):
    """Get a single hypothesis by id."""
    h = db.get(Hypothesis, id)
    if not h:
        raise HTTPException(status_code=404, detail="Hypothesis not found")
    return HypothesisResponse(
        id=h.id,
        claim=h.claim,
        status=h.status,
        confidence=h.confidence,
        schema_version=getattr(h, "schema_version", "1.0"),
        created_at=h.created_at,
        updated_at=h.updated_at,
    )


@app.get("/hypotheses/{id}/evidence", response_model=list[EvidenceResponse])
def get_hypothesis_evidence(
    id: str,
    db: Annotated[Session, Depends(get_db)],
):
    """Get all evidence for a hypothesis."""
    h = db.get(Hypothesis, id)
    if not h:
        raise HTTPException(status_code=404, detail="Hypothesis not found")
    rows = db.query(Evidence).filter(Evidence.hypothesis_id == id).all()
    return [
        EvidenceResponse(
            id=e.id,
            hypothesis_id=e.hypothesis_id,
            run_id=e.run_id,
            direction=e.direction,
            strength=e.strength,
            rationale=e.rationale,
            timestamp=e.timestamp,
        )
        for e in rows
    ]


@app.post("/runs")
def submit_run(
    bundle: RunBundle,
    db: Annotated[Session, Depends(get_db)],
):
    """
    Submit a run bundle. Atomic: validate → persist run → store artifacts
    → create evidence → update hypothesis confidence.
    """
    hypothesis = db.get(Hypothesis, bundle.hypothesis_id)
    if not hypothesis:
        raise HTTPException(status_code=404, detail="Hypothesis not found")

    existing_run = db.get(Run, bundle.run_id)
    if existing_run:
        raise HTTPException(status_code=409, detail=f"Run {bundle.run_id} already exists")

    # 1. Store artifacts first (filesystem); if this fails we don't touch DB
    run_payload = bundle.model_dump(mode="json")
    artifacts_path = store_artifacts(
        bundle.run_id,
        bundle.artifact_manifest,
        run_payload,
        None,
    )

    try:
        # 2. Persist run
        run_row = Run(
            run_id=bundle.run_id,
            parent_run_id=bundle.parent_run_id,
            batch_id=bundle.batch_id,
            schema_version=bundle.schema_version,
            design_id=bundle.design_id,
            hypothesis_id=bundle.hypothesis_id,
            experiment_id=bundle.experiment_id,
            engine=bundle.engine,
            timestamp=bundle.timestamp,
            environment=bundle.environment,
            metrics_json=json.dumps(bundle.metrics),
            artifacts_path=str(artifacts_path),
            notes=bundle.notes,
        )
        db.add(run_row)

        # 3. Create evidence
        evidence_row = Evidence(
            hypothesis_id=bundle.hypothesis_id,
            run_id=bundle.run_id,
            direction=bundle.outcome.direction,
            strength=bundle.outcome.strength,
            rationale=bundle.outcome.rationale,
            timestamp=bundle.timestamp,
        )
        db.add(evidence_row)

        # 4. Update hypothesis confidence
        old_confidence = hypothesis.confidence
        new_confidence = update_confidence(
            hypothesis.confidence,
            bundle.outcome.direction,
            bundle.outcome.strength,
        )
        hypothesis.confidence = new_confidence
        hypothesis.updated_at = datetime.utcnow()

        db.commit()

        # 5. Index run in experiment registry (non-fatal)
        try:
            index_run(
                run_id=bundle.run_id,
                hypothesis_id=bundle.hypothesis_id,
                design_id=bundle.design_id,
                environment=bundle.environment,
                primary_metric=bundle.metrics.get("slip_rate", 0),
                outcome_direction=bundle.outcome.direction,
                confidence_delta=new_confidence - old_confidence,
                batch_id=bundle.batch_id,
            )
        except Exception as e:
            print(f"Registry indexing failed: {e}")  # Non-fatal

        # 6. Add to discovery surface (non-fatal)
        try:
            param_vector = {
                "design_id": bundle.design_id,
                "environment": bundle.environment,
                "slip_rate": bundle.metrics.get("slip_rate"),
                "contact_area": bundle.metrics.get("contact_area"),
            }
            slip = bundle.metrics.get("slip_rate", 0.5)
            result_score = 1.0 - min(slip, 1.0)
            conf_delta = new_confidence - old_confidence if hypothesis else 0
            novelty = bundle.outcome.direction == "refutes" or bundle.outcome.strength > 0.9
            add_surface_point(
                run_id=bundle.run_id,
                hypothesis_id=bundle.hypothesis_id,
                parameter_vector=param_vector,
                environment=bundle.environment,
                result_score=result_score,
                confidence_delta=conf_delta,
                novelty_flag=novelty,
                exploration_mode="exploit",
            )
        except Exception as e:
            print(f"Discovery surface indexing failed: {e}")  # Non-fatal
    except Exception:
        db.rollback()
        raise HTTPException(status_code=500, detail="Failed to persist run")

    return {
        "status": "accepted",
        "run_id": bundle.run_id,
        "new_confidence": round(new_confidence, 4),
    }


@app.get("/runs/{run_id}", response_model=RunResponse)
def get_run(
    run_id: str,
    db: Annotated[Session, Depends(get_db)],
):
    """Get a single run by run_id."""
    r = db.get(Run, run_id)
    if not r:
        raise HTTPException(status_code=404, detail="Run not found")
    return RunResponse(
        run_id=r.run_id,
        hypothesis_id=r.hypothesis_id,
        experiment_id=r.experiment_id,
        engine=r.engine,
        timestamp=r.timestamp,
        environment=r.environment,
        metrics_json=r.metrics_json,
        artifacts_path=r.artifacts_path,
        notes=r.notes,
    )


@app.get("/brief")
def generate_brief(db: Session = Depends(get_db)):
    """Generate weekly brief data from Lab OS"""
    from datetime import timedelta

    # Get all hypotheses
    hypotheses = db.query(models.Hypothesis).all()

    # Get recent evidence (last 7 days)
    week_ago = datetime.utcnow() - timedelta(days=7)
    recent_evidence = db.query(models.Evidence).filter(
        models.Evidence.timestamp >= week_ago
    ).all()

    # Get recent runs
    recent_runs = db.query(models.Run).filter(
        models.Run.timestamp >= week_ago
    ).all()

    # Compute what changed
    changes = []
    for h in hypotheses:
        h_evidence = [e for e in recent_evidence if e.hypothesis_id == h.id]
        if h_evidence:
            supports = sum(1 for e in h_evidence if e.direction == "supports")
            refutes = sum(1 for e in h_evidence if e.direction == "refutes")
            changes.append({
                "hypothesis_id": h.id,
                "claim": h.claim,
                "confidence": h.confidence,
                "evidence_count": len(h_evidence),
                "supports": supports,
                "refutes": refutes
            })

    # Find contradictions
    contradictions = [c for c in changes if c["supports"] > 0 and c["refutes"] > 0]

    # Fragility (low evidence count)
    fragile = [
        {"hypothesis_id": h.id, "claim": h.claim, "confidence": h.confidence}
        for h in hypotheses
        if len([e for e in recent_evidence if e.hypothesis_id == h.id]) < 3
    ]

    # Run summary
    run_summary = {
        "total_runs": len(recent_runs),
        "by_environment": {},
        "by_engine": {}
    }
    for r in recent_runs:
        env = r.environment or "unknown"
        eng = r.engine or "unknown"
        run_summary["by_environment"][env] = run_summary["by_environment"].get(env, 0) + 1
        run_summary["by_engine"][eng] = run_summary["by_engine"].get(eng, 0) + 1

    return {
        "generated_at": datetime.utcnow().isoformat(),
        "period": "last_7_days",
        "hypotheses_count": len(hypotheses),
        "evidence_count": len(recent_evidence),
        "runs_count": len(recent_runs),
        "changes": changes,
        "contradictions": contradictions,
        "fragile": fragile,
        "run_summary": run_summary
    }


@app.get("/brief/detailed")
def generate_detailed_brief(db: Session = Depends(get_db)):
    """Detailed brief with charts data and trends"""
    from collections import defaultdict
    from datetime import timedelta

    hypotheses = db.query(models.Hypothesis).all()
    all_evidence = db.query(models.Evidence).order_by(models.Evidence.timestamp).all()
    all_runs = db.query(models.Run).order_by(models.Run.timestamp).all()

    # Confidence over time (for charting)
    confidence_history = []
    running_confidence = 0.5
    for e in all_evidence:
        if e.direction == "supports":
            running_confidence = min(0.95, running_confidence + 0.05 * e.strength)
        elif e.direction == "refutes":
            running_confidence = max(0.05, running_confidence - 0.05 * e.strength)
        confidence_history.append({
            "timestamp": e.timestamp.isoformat() if e.timestamp else None,
            "confidence": round(running_confidence, 4),
            "direction": e.direction,
            "run_id": e.run_id,
        })

    # Metrics over time
    metrics_history = []
    for r in all_runs:
        if r.metrics_json:
            metrics = json.loads(r.metrics_json)
            metrics_history.append({
                "run_id": r.run_id,
                "timestamp": r.timestamp.isoformat() if r.timestamp else None,
                "environment": r.environment or "unknown",
                "slip_rate": metrics.get("slip_rate"),
                "contact_area": metrics.get("contact_area"),
                "grasp_force": metrics.get("grasp_force"),
                "grasp_success": metrics.get("grasp_success"),
            })

    # Best/worst runs
    sorted_by_slip = sorted(
        metrics_history,
        key=lambda x: x.get("slip_rate") if x.get("slip_rate") is not None else 999,
    )
    best_runs = sorted_by_slip[:3]
    worst_runs = sorted_by_slip[-3:] if len(sorted_by_slip) >= 3 else sorted_by_slip

    # Environment comparison
    env_stats = defaultdict(
        lambda: {"runs": 0, "successes": 0, "avg_slip": [], "avg_contact": []}
    )
    for m in metrics_history:
        env = m["environment"]
        env_stats[env]["runs"] += 1
        if m.get("grasp_success"):
            env_stats[env]["successes"] += 1
        if m.get("slip_rate") is not None:
            env_stats[env]["avg_slip"].append(m["slip_rate"])
        if m.get("contact_area") is not None:
            env_stats[env]["avg_contact"].append(m["contact_area"])

    env_comparison = {}
    for env, stats in env_stats.items():
        env_comparison[env] = {
            "runs": stats["runs"],
            "success_rate": (
                stats["successes"] / stats["runs"] if stats["runs"] > 0 else 0
            ),
            "avg_slip_rate": (
                sum(stats["avg_slip"]) / len(stats["avg_slip"])
                if stats["avg_slip"]
                else None
            ),
            "avg_contact_area": (
                sum(stats["avg_contact"]) / len(stats["avg_contact"])
                if stats["avg_contact"]
                else None
            ),
        }

    # Design comparison (which design_id performed best)
    design_stats = defaultdict(lambda: {"runs": 0, "successes": 0, "avg_slip": []})
    for r in all_runs:
        if r.metrics_json:
            metrics = json.loads(r.metrics_json)
            design_id = r.design_id or "unknown"
            design_stats[design_id]["runs"] += 1
            if metrics.get("grasp_success"):
                design_stats[design_id]["successes"] += 1
            if metrics.get("slip_rate") is not None:
                design_stats[design_id]["avg_slip"].append(metrics["slip_rate"])

    design_comparison = {}
    for d, stats in design_stats.items():
        design_comparison[d] = {
            "runs": stats["runs"],
            "success_rate": (
                stats["successes"] / stats["runs"] if stats["runs"] > 0 else 0
            ),
            "avg_slip_rate": (
                sum(stats["avg_slip"]) / len(stats["avg_slip"])
                if stats["avg_slip"]
                else None
            ),
        }

    return {
        "generated_at": datetime.utcnow().isoformat(),
        "confidence_history": confidence_history,
        "metrics_history": metrics_history,
        "best_runs": best_runs,
        "worst_runs": worst_runs,
        "environment_comparison": env_comparison,
        "design_comparison": design_comparison,
        "total_runs": len(all_runs),
        "total_evidence": len(all_evidence),
    }


@app.get("/surface")
def get_discovery_surface():
    """Get discovery surface statistics"""
    from app.registry import get_surface_stats, get_anomalies, get_undersampled_regions

    return {
        "stats": get_surface_stats(),
        "anomalies": get_anomalies()[:10],
        "undersampled": get_undersampled_regions(),
    }


@app.get("/graph")
def get_run_graph():
    """Get the full run graph for visualization"""
    return build_run_graph()


@app.get("/graph/hypothesis/{hypothesis_id}")
def get_hypothesis_graph(hypothesis_id: str):
    """Get experiment tree for a specific hypothesis"""
    return get_hypothesis_tree(hypothesis_id)


@app.get("/graph/export")
def export_graph():
    """Export graph in visualization format"""
    return export_graph_for_viz()


@app.get("/query")
def semantic_query(q: str):
    """
    Semantic query over runs.

    Examples:
    - /query?q=wet%20environment%20supports
    - /query?q=slip_rate%20<%200.1
    - /query?q=H-TEST%20refutes
    """
    results = natural_query(q)
    return {
        "query": q,
        "results": results,
        "summary": query_summary(results)
    }


@app.get("/digest")
def get_digest(hours: int = 24):
    """Generate lab digest for the last N hours"""
    return generate_digest(hours)
