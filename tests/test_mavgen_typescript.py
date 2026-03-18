#!/usr/bin/env python3
"""
Tests for the TypeScript generator.
"""

from pathlib import Path
import shutil
import sys

try:
    from pymavlink.generator import mavgen
except ModuleNotFoundError:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from generator import mavgen


def test_typescript_generator_uses_scoped_node_mavlink_package():
    xml_filepath = Path(__file__).parent / "snapshottests" / "resources" / "common.xml"
    output_dir = Path(__file__).resolve().parents[1] / ".tmp" / "typescript-generator"
    shutil.rmtree(output_dir, ignore_errors=True)
    output_dir.mkdir(parents=True, exist_ok=True)

    ok = mavgen.mavgen(
        mavgen.Opts(
            output=str(output_dir),
            language="TypeScript",
            wire_protocol="2.0",
            validate=False,
        ),
        [str(xml_filepath)],
    )

    assert ok is True

    heartbeat = (output_dir / "messages" / "heartbeat.ts").read_text(encoding="utf-8")
    registry = (output_dir / "message-registry.ts").read_text(encoding="utf-8")

    assert "@ifrunistuttgart/node-mavlink" in heartbeat
    assert "from 'node-mavlink'" not in heartbeat
    assert "@ifrunistuttgart/node-mavlink" in registry
    assert "from 'node-mavlink'" not in registry
