import csv

import numpy as np
import pytest
import trimesh

from cad_to_robot.export import export_paths
from cad_to_robot.frames import frame, flange_poses, tcp_poses, validate_frame, xyzabc
from cad_to_robot.geometry import load_stl, resample, section_contours


def test_rectangle_perimeter_spacing_and_corners():
    mesh = trimesh.creation.box(extents=[100, 60, 10])
    loops = section_contours(mesh, 0)
    assert len(loops) == 1
    p = resample(loops[0], 7)
    assert np.allclose(p[0], p[-1])
    assert np.linalg.norm(np.diff(p, axis=0), axis=1).sum() == pytest.approx(320)
    assert np.linalg.norm(np.diff(p, axis=0), axis=1).max() <= 7 + 1e-10
    for x, y in [(-50, -30), (-50, 30), (50, -30), (50, 30)]:
        assert np.min(np.linalg.norm(p - [x, y, 0], axis=1)) < 1e-9


def test_disconnected_sections_stay_separate():
    a = trimesh.creation.box(extents=[10, 10, 10])
    b = a.copy()
    b.apply_translation([30, 0, 0])
    loops = section_contours(trimesh.util.concatenate([a, b]), 0)
    assert len(loops) == 2
    assert all(np.ptp(p[:, 0]) == pytest.approx(10) for p in loops)


def test_cylinder_matches_polygon_and_approximates_circle():
    n, radius = 64, 40
    mesh = trimesh.creation.cylinder(radius=radius, height=10, sections=n)
    points = resample(section_contours(mesh, 0)[0], 2)
    length = np.linalg.norm(np.diff(points, axis=0), axis=1).sum()
    assert length == pytest.approx(2 * n * radius * np.sin(np.pi / n))
    assert abs(length - 2 * np.pi * radius) / (2 * np.pi * radius) < .001


def test_frame_composition_and_tool_offset():
    base = frame([100, 200, 300], [90, 0, 0])
    poses = tcp_poses([[10, 0, 0]], base, [0, 0, 180])
    assert np.allclose(poses[0, :3, 3], [100, 210, 300])
    tool = frame([0, 0, 50])
    flange = flange_poses(poses, tool)
    assert np.allclose(flange @ tool, poses)
    assert np.allclose(flange[0, :3, 3], [100, 210, 350])
    recovered = xyzabc(poses[0])
    assert np.allclose(frame(recovered[:3], recovered[3:]), poses[0])


@pytest.mark.parametrize("spacing", [0, -1, float("nan"), float("inf")])
def test_bad_spacing(spacing):
    with pytest.raises(ValueError):
        resample([[0, 0, 0], [1, 0, 0]], spacing)


@pytest.mark.parametrize("z", [-5, 5, 10, float("nan")])
def test_bad_section(z):
    with pytest.raises(ValueError):
        section_contours(trimesh.creation.box(extents=[10, 10, 10]), z)


def test_reject_reflection():
    t = np.eye(4)
    t[0, 0] = -1
    with pytest.raises(ValueError):
        validate_frame(t)


def test_stl_units_cleanup_and_open_mesh(tmp_path):
    path = tmp_path / "box.stl"
    mesh = trimesh.creation.box(extents=[.1, .06, .01])
    mesh.export(path)
    loaded = load_stl(path, "m")
    assert loaded.extents == pytest.approx([100, 60, 10])
    mesh.update_faces(np.arange(len(mesh.faces) - 1))
    mesh.export(path)
    with pytest.raises(ValueError, match="closed"):
        load_stl(path, "mm")


def test_export_round_trip_and_no_executable_motion(tmp_path):
    paths = [tcp_poses([[0, 0, 0], [10, 0, 0]], frame()),
             tcp_poses([[20, 0, 0], [30, 0, 0]], frame())]
    export_paths(paths, tmp_path)
    with (tmp_path / "waypoints.csv").open() as f:
        rows = list(csv.DictReader(f))
    assert [r["path_id"] for r in rows] == ["0", "0", "1", "1"]
    assert [float(r["x_mm"]) for r in rows] == [0, 10, 20, 30]
    for src in tmp_path.glob("*.SRC"):
        lines = src.read_text().splitlines()
        assert sum(line.startswith("; LIN ") for line in lines) == 2
        assert not any(line.startswith(("LIN ", "PTP ", "$OUT")) for line in lines)
        assert src.with_suffix(".DAT").exists()
