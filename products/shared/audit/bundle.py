"""
OMEGA Audit Bundle

Single zip file containing all information to reproduce a result.
"""
import json
import zipfile
import io
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any, List
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent.parent.parent))
from shared.id_generator import bundle_id


@dataclass
class BundleMetadata:
    """Metadata for an audit bundle"""
    bundle_id: str
    created_at: str
    component: str  # Which product created this
    artifact_id: Optional[str] = None
    validation_id: Optional[str] = None
    run_id: Optional[str] = None
    user: str = "omega-user"
    omega_version: str = "2.0"

    def to_dict(self) -> Dict:
        return asdict(self)


@dataclass
class AuditBundle:
    """
    Audit bundle structure:

    audit_bundle_BDL-XXXXXXXX.zip
    ├── metadata.json       # Bundle info, timestamps, IDs
    ├── artifact.json       # The design/scene/result itself
    ├── contract.json       # Component contract used
    ├── validation.json     # Validation results (if any)
    ├── logs.txt            # Execution logs
    └── video.mp4           # (optional) Recorded simulation
    """
    metadata: BundleMetadata
    artifact: Dict[str, Any]
    contract: Optional[Dict[str, Any]] = None
    validation: Optional[Dict[str, Any]] = None
    logs: Optional[str] = None
    video_path: Optional[Path] = None

    def _write_to_zip(self, zf: zipfile.ZipFile, video_path: Optional[Path] = None) -> None:
        """Write bundle contents to an open ZipFile."""
        zf.writestr("metadata.json", json.dumps(self.metadata.to_dict(), indent=2))
        zf.writestr("artifact.json", json.dumps(self.artifact, indent=2, default=str))
        if self.contract:
            zf.writestr("contract.json", json.dumps(self.contract, indent=2))
        if self.validation:
            zf.writestr("validation.json", json.dumps(self.validation, indent=2, default=str))
        if self.logs:
            zf.writestr("logs.txt", self.logs)
        path = video_path or self.video_path
        if path and Path(path).exists():
            zf.write(path, "video.mp4")

    def save_to_bytes(self) -> bytes:
        """Save bundle to in-memory zip and return bytes (for download)."""
        buffer = io.BytesIO()
        with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as zf:
            self._write_to_zip(zf)
        buffer.seek(0)
        return buffer.getvalue()

    def save(self, output_dir: Path = None) -> Path:
        """Save bundle as zip file."""
        if output_dir is None:
            output_dir = Path.cwd()
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

        zip_name = f"audit_bundle_{self.metadata.bundle_id}.zip"
        zip_path = output_dir / zip_name

        with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
            self._write_to_zip(zf)

        return zip_path

    @classmethod
    def load(cls, zip_path: Path) -> "AuditBundle":
        """Load bundle from zip file."""
        with zipfile.ZipFile(zip_path, "r") as zf:
            metadata_dict = json.loads(zf.read("metadata.json"))
            metadata = BundleMetadata(**metadata_dict)

            artifact = json.loads(zf.read("artifact.json"))

            contract = None
            if "contract.json" in zf.namelist():
                contract = json.loads(zf.read("contract.json"))

            validation = None
            if "validation.json" in zf.namelist():
                validation = json.loads(zf.read("validation.json"))

            logs = None
            if "logs.txt" in zf.namelist():
                logs = zf.read("logs.txt").decode("utf-8")

            video_path = None

            return cls(
                metadata=metadata,
                artifact=artifact,
                contract=contract,
                validation=validation,
                logs=logs,
                video_path=video_path,
            )


def create_bundle(
    component: str,
    artifact: Dict[str, Any],
    artifact_id: str = None,
    validation_id: str = None,
    run_id: str = None,
    contract: Dict = None,
    validation: Dict = None,
    logs: str = None,
    video_path: Path = None,
) -> AuditBundle:
    """Helper to create an audit bundle."""
    metadata = BundleMetadata(
        bundle_id=bundle_id(),
        created_at=datetime.now().isoformat(),
        component=component,
        artifact_id=artifact_id,
        validation_id=validation_id,
        run_id=run_id,
    )

    return AuditBundle(
        metadata=metadata,
        artifact=artifact,
        contract=contract,
        validation=validation,
        logs=logs,
        video_path=video_path,
    )
