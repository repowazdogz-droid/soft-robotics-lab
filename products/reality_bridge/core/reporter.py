"""
Reporter - Generate validation reports as dict, markdown, or HTML.
"""

from typing import Dict, Any, List
import html


def to_dict(result) -> Dict[str, Any]:
    """
    Convert ValidationResult to a JSON-serializable dict.
    """
    tests = {}
    for k, v in result.tests.items():
        tests[k] = {
            "name": v.name,
            "passed": v.passed,
            "message": v.message,
            "details": _sanitize(v.details),
        }
    return {
        "passed": result.passed,
        "score": result.score,
        "tests": tests,
        "warnings": list(result.warnings),
        "errors": list(result.errors),
        "metrics": _sanitize(result.metrics),
    }


def to_markdown(result) -> str:
    """
    Human-readable markdown report.
    """
    lines = [
        "# Validation Report",
        "",
        f"- **Passed:** {result.passed}",
        f"- **Score:** {result.score:.2f} (0–1)",
        "",
        "## Tests",
        "",
    ]
    for name, tr in result.tests.items():
        status = "✓" if tr.passed else "✗"
        lines.append(f"- {status} **{name}**: {tr.message}")
        if tr.details:
            for k, v in tr.details.items():
                lines.append(f"  - {k}: {v}")
    lines.append("")
    lines.append("## Metrics")
    lines.append("")
    for k, v in result.metrics.items():
        if isinstance(v, float):
            lines.append(f"- **{k}:** {v:.4f}")
        else:
            lines.append(f"- **{k}:** {v}")
    if result.warnings:
        lines.append("")
        lines.append("## Warnings")
        lines.append("")
        for w in result.warnings:
            lines.append(f"- {w}")
    if result.errors:
        lines.append("")
        lines.append("## Errors")
        lines.append("")
        for e in result.errors:
            lines.append(f"- {e}")
    return "\n".join(lines)


def to_html(result) -> str:
    """
    Styled HTML report.
    """
    tests_rows = []
    for name, tr in result.tests.items():
        status = "pass" if tr.passed else "fail"
        detail = ""
        if tr.details:
            detail = " ".join(f"{k}={v}" for k, v in tr.details.items())
        tests_rows.append(
            f"<tr><td>{_esc(name)}</td><td class=\"{status}\">{status}</td>"
            f"<td>{_esc(tr.message)}</td><td>{_esc(detail)}</td></tr>"
        )
    metrics_rows = []
    for k, v in result.metrics.items():
        val = f"{v:.4f}" if isinstance(v, float) else str(v)
        metrics_rows.append(f"<tr><td>{_esc(k)}</td><td>{_esc(val)}</td></tr>")
    warnings_li = "".join(f"<li>{_esc(w)}</li>" for w in result.warnings)
    errors_li = "".join(f"<li>{_esc(e)}</li>" for e in result.errors)

    html_body = f"""
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8"/>
  <title>Reality Bridge – Validation Report</title>
  <style>
    body {{ font-family: system-ui, sans-serif; margin: 1rem 2rem; background: #1a1a1a; color: #e0e0e0; }}
    h1 {{ color: #7dd3fc; }}
    h2 {{ color: #a5b4fc; margin-top: 1.5rem; }}
    table {{ border-collapse: collapse; width: 100%; max-width: 640px; }}
    th, td {{ border: 1px solid #444; padding: 0.5rem 0.75rem; text-align: left; }}
    th {{ background: #333; color: #c4b5fd; }}
    .pass {{ color: #86efac; }}
    .fail {{ color: #fca5a5; }}
    ul {{ padding-left: 1.5rem; }}
    .score {{ font-size: 1.25rem; }}
  </style>
</head>
<body>
  <h1>Validation Report</h1>
  <p class="score">Passed: <strong>{result.passed}</strong> &nbsp; Score: <strong>{result.score:.2f}</strong></p>

  <h2>Tests</h2>
  <table>
    <thead><tr><th>Test</th><th>Status</th><th>Message</th><th>Details</th></tr></thead>
    <tbody>
      {"".join(tests_rows)}
    </tbody>
  </table>

  <h2>Metrics</h2>
  <table>
    <thead><tr><th>Metric</th><th>Value</th></tr></thead>
    <tbody>
      {"".join(metrics_rows)}
    </tbody>
  </table>

  {"<h2>Warnings</h2><ul>" + warnings_li + "</ul>" if result.warnings else ""}
  {"<h2>Errors</h2><ul>" + errors_li + "</ul>" if result.errors else ""}
</body>
</html>
"""
    return html_body.strip()


def _sanitize(obj: Any) -> Any:
    """Make value JSON-serializable."""
    if obj is None or isinstance(obj, (bool, int, str)):
        return obj
    if isinstance(obj, float):
        if __import__("math").isnan(obj) or __import__("math").isinf(obj):
            return None
        return obj
    if isinstance(obj, dict):
        return {str(k): _sanitize(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_sanitize(x) for x in obj]
    return str(obj)


def _esc(s: str) -> str:
    return html.escape(str(s))
