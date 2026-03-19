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

    generated_typescript_files = sorted(output_dir.rglob("*.ts"))
    assert generated_typescript_files

    file_contents = {
        path.relative_to(output_dir).as_posix(): path.read_text(encoding="utf-8")
        for path in generated_typescript_files
    }

    assert "@ifrunistuttgart/node-mavlink" in file_contents["message-registry.ts"]
    assert "from 'node-mavlink'" not in "\n".join(file_contents.values())
