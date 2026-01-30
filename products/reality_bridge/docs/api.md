# Reality Bridge API

Physics validation API for MJCF/URDF models. All endpoints return JSON. CORS enabled.

**Base URL:** `http://localhost:8000` (default)

---

## Endpoints

### POST /validate

Validate a single model.

**Input:** One of:
- Multipart form: `file` (XML/URDF/MJCF)
- Form: `xml_string`, optional `artifact_id`
- JSON body: `{"xml_string": "...", "artifact_id": "..."}`

**Response (200):**
```json
{
  "success": true,
  "passed": true,
  "score": 0.83,
  "contract_status": "ok",
  "validation_id": "VB-...",
  "design_hash": "...",
  "validation_time_ms": 234,
  "tests": { ... },
  "metrics": { ... },
  "warnings": [],
  "errors": [],
  "failures": [
    {
      "code": "PHYSICS_INSTABILITY",
      "message": "Simulation became unstable",
      "suggestions": ["Reduce timestep", "Add damping"],
      "tutor_link": "http://localhost:8503/?topic=physics+simulation+stability"
    }
  ],
  "bundle_base64": "...",
  "bundle_filename": "audit_bundle_....zip"
}
```

**Errors:** 400 (missing params), 422 (load/sim failed). Response includes `failure` object with `code`, `message`, `actions`, `tutor_link`.

---

### POST /validate/batch

Validate multiple designs in one request.

**Request body:**
```json
{
  "designs": [
    { "mjcf": "<mujoco>...</mujoco>", "artifact_id": "ART-001" },
    { "xml_string": "<robot>...</robot>", "artifact_id": "ART-002" }
  ]
}
```

**Response (200):**
```json
{
  "success": true,
  "results": [
    { "artifact_id": "ART-001", "passed": true, "score": 1.0, "validation_id": "...", ... }
  ],
  "summary": { "total": 2, "passed": 1, "failed": 1 }
}
```

**Webhook:** `batch_completed` is triggered with `summary` and `results_count`.

---

### POST /analyze

Analyze a model: weak points, failure modes, suggestions. Same input as `/validate`.

**Response (200):**
```json
{
  "success": true,
  "weak_points": [ { "kind": "...", "name": "...", "reason": "...", "details": {} } ],
  "failure_modes": [],
  "suggestions": [],
  "metrics": {}
}
```

---

### GET /health

Service health and MuJoCo version.

**Response (200):** `{ "status": "ok", "mujoco_version": "3.0.0" }`

---

### GET /stats

Validation statistics and performance metrics.

**Response (200):**
```json
{
  "total_validations": 1247,
  "pass_rate": 0.87,
  "avg_score": 0.85,
  "avg_validation_ms": 234,
  "today": { "count": 47, "pass_rate": 0.91 },
  "this_week": { "count": 312, "pass_rate": 0.88 },
  "failure_distribution": {
    "PHYSICS_INSTABILITY": 0.34,
    "GRASP_FAILURE": 0.28,
    "GEOMETRY_SELF_INTERSECTION": 0.18
  }
}
```

---

### GET /failures

Recent validation failures.

**Query:** `limit` (default 50)

**Response (200):** Array of `{ id, validation_id, timestamp, design_hash, score, errors, source }`

---

### GET /leaderboard

Top designs by score.

**Query:** `limit` (default 10)

**Response (200):** Array of `{ id, timestamp, design_hash, passed, score, source }`

---

### GET /validations/recent

Recent validations for dashboard.

**Query:** `limit` (default 20)

**Response (200):** Array of `{ id, validation_id, timestamp, design_hash, artifact_id, passed, score, validation_time_ms }`

---

### GET /validations/history/{design_id}

All validations for a design (by `artifact_id` or `design_hash`).

**Query:** `limit` (default 50)

**Response (200):** Array of `{ id, validation_id, timestamp, passed, score, validation_time_ms, errors }`

---

### POST /webhooks

Register a webhook.

**Request body:**
```json
{
  "url": "https://example.com/hook",
  "events": ["validation_completed", "validation_failed", "batch_completed"],
  "secret": "optional-signing-secret"
}
```

**Response (200):** `{ "success": true, "webhook_id": "wh-xxxxxxxx" }`

**Events:** `validation_completed`, `validation_failed`, `batch_completed`. Payload includes `event`, `timestamp`, `validation_id`, `artifact_id`, `passed`, `score`, `failures` (if failed). Optional `X-Webhook-Signature` (HMAC-SHA256 of body with secret).

---

### GET /webhooks

List registered webhooks (id, url, events, active; no secret).

**Response (200):** `{ "webhooks": [ { "id": "wh-...", "url": "...", "events": [...], "active": true } ] }`

---

### DELETE /webhooks/{webhook_id}

Remove a webhook.

**Response:** 200 `{ "success": true }` or 404.

---

## Error codes (failure taxonomy)

| Code | Description |
|------|-------------|
| PHYSICS_INSTABILITY | Simulation unstable (NaN/Inf) |
| MATERIAL_OUT_OF_RANGE | Material properties out of bounds |
| GEOMETRY_SELF_INTERSECTION | Parts intersect |
| GRASP_FAILURE | Object not grasped |
| INVALID_INPUT_UNITS | Not SI units |
| VALIDATION_TIMEOUT | Too long |
| MISSING_PARAMS | Required params missing |
| UNKNOWN | Other |

Responses include `suggestions` and `tutor_link` for learning.

---

## Rate limits

No built-in rate limit. For production, put a reverse proxy (e.g. nginx) in front.

---

## Examples

**Validate with curl:**
```bash
curl -X POST http://localhost:8000/validate -F "xml_string=<mujoco model='test'>...</mujoco>" -F "artifact_id=ART-001"
```

**Batch validate:**
```bash
curl -X POST http://localhost:8000/validate/batch -H "Content-Type: application/json" -d '{"designs":[{"mjcf":"<mujoco>...</mujoco>","artifact_id":"ART-001"}]}'
```

**Register webhook:**
```bash
curl -X POST http://localhost:8000/webhooks -H "Content-Type: application/json" -d '{"url":"https://example.com/hook","events":["validation_failed"]}'
```
