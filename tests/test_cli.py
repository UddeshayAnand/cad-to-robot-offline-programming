import json
import subprocess
import sys

import numpy as np
import pytest
import trimesh

from cad_to_robot.export import export_paths
from cad_to_robot.frames import frame, validate_frame
from cad_to_robot.geometry import resample


def test_cli_complete_run_and_output_protection(tmp_path):
    mesh = tmp_path / "plate.stl"
    trimesh.creation.box(extents=[100, 60, 10]).export(mesh)
    out = tmp_path / "run"
    command = [sys.executable, "-m", "cad_to_robot.cli", str(mesh),
               "--units", "mm", "--z", "0", "--output", str(out)]
    result = subprocess.run(command, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    metrics = json.loads((out / "metrics.json").read_text())
    assert metrics["lengths_mm"] == pytest.approx([320])
    assert metrics["controller_validated"] is False
    assert metrics["environment"]["python"]
    assert (out / "paths.png").stat().st_size > 1000
    assert (out / "CONTOUR_000.SRC").is_file()
    before = (out / "waypoints.csv").read_bytes()
    again = subprocess.run(command, capture_output=True, text=True)
    assert again.returncode != 0
    assert "new or empty" in again.stderr
    assert (out / "waypoints.csv").read_bytes() == before


def test_cli_bad_input_creates_no_outputs(tmp_path):
    output = tmp_path / "output"
    result = subprocess.run([sys.executable, "-m", "cad_to_robot.cli",
                             str(tmp_path / "missing.stl"), "--units", "mm",
                             "--z", "0", "--output", str(output)], capture_output=True)
    assert result.returncode != 0
    assert not output.exists()


def test_export_rejects_empty_paths(tmp_path):
    with pytest.raises(ValueError):
        export_paths([], tmp_path / "empty")
    assert not (tmp_path / "empty").exists()


def test_reject_small_nonrigid_scale():
    transform = frame()
    transform[0, 0] += 1e-6
    with pytest.raises(ValueError):
        validate_frame(transform)


def test_resampling_resource_limit():
    with pytest.raises(ValueError, match="too many"):
        resample(np.array([[0, 0, 0], [1, 0, 0]]), 1e-20)
