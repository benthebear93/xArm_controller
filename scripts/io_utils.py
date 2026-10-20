"""JSON persistence and timestamps shared by command-line tools."""

from datetime import datetime, timezone
import json
from pathlib import Path


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def read_json(path):
    with Path(path).open(encoding="utf-8") as stream:
        return json.load(stream)


def write_json(path, data):
    """Replace a JSON file atomically, never writing non-standard NaN values."""
    path = Path(path)
    payload = json.dumps(data, indent=2, allow_nan=False) + "\n"
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(payload, encoding="utf-8")
    temporary.replace(path)
