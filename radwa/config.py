"""Configuration loading and provenance."""

import hashlib
import json


def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def config_fingerprint(cfg):
    """Stable SHA-256 over the frozen configuration (provenance stamp)."""
    blob = json.dumps(cfg, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()
