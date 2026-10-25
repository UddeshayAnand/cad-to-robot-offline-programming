"""STL preprocessing and explicitly selected planar contour paths."""
from pathlib import Path

import numpy as np
import trimesh


def load_stl(path: str | Path, units: str) -> trimesh.Trimesh:
    """Load ASCII/binary STL, merge coincident vertices, remove bad faces."""
    if units not in {"mm", "m"}:
        raise ValueError("STL units must be explicitly mm or m")
    if Path(path).suffix.lower() != ".stl":
        raise ValueError("Only STL input is supported")
    mesh = trimesh.load_mesh(path, process=False)
    if not isinstance(mesh, trimesh.Trimesh) or not len(mesh.faces):
        raise ValueError("Input must contain a nonempty triangle mesh")
    if not np.isfinite(mesh.vertices).all():
        raise ValueError("Mesh contains nonfinite coordinates")
    if units == "m":
        mesh.apply_scale(1000)
    mesh.merge_vertices()
    mesh.update_faces(mesh.unique_faces())
    mesh.update_faces(mesh.nondegenerate_faces())
    mesh.remove_unreferenced_vertices()
    if not len(mesh.faces) or not mesh.is_watertight:
        raise ValueError("Mesh must be closed after cleanup; repair holes explicitly")
    return mesh


def section_contours(mesh: trimesh.Trimesh, z_mm: float) -> list[np.ndarray]:
    """Intersect an interior horizontal plane; keep closed loops separate."""
    if not np.isfinite(z_mm):
        raise ValueError("Section height must be finite")
    # Coplanar boundary faces do not define an unambiguous interior section.
    if not mesh.bounds[0, 2] < z_mm < mesh.bounds[1, 2]:
        raise ValueError("Section plane must be strictly inside mesh Z bounds")
    section = mesh.section(plane_origin=[0, 0, z_mm], plane_normal=[0, 0, 1])
    if section is None:
        raise ValueError("Section does not intersect the mesh")
    loops = []
    for path in section.discrete:
        if len(path) < 4 or not np.allclose(path[0], path[-1], atol=1e-7, rtol=0):
            raise ValueError("Section contains an open or ambiguous contour")
        p = np.asarray(path[:-1])
        start = np.lexsort((p[:, 2], p[:, 1], p[:, 0]))[0]
        p = np.roll(p, -start, axis=0)
        area = np.sum(p[:, 0] * np.roll(p[:, 1], -1) - p[:, 1] * np.roll(p[:, 0], -1))
        if area < 0:
            p = np.vstack([p[0], p[:0:-1]])
        loops.append(np.vstack([p, p[0]]))
    if not loops:
        raise ValueError("No closed contours found")
    return sorted(loops, key=lambda p: tuple(p[0]))


def resample(path: np.ndarray, spacing_mm: float) -> np.ndarray:
    """Split each segment independently, preserving every original corner."""
    p = np.asarray(path, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 2 or not np.isfinite(p).all():
        raise ValueError("Path must contain at least two finite XYZ points")
    if not np.isfinite(spacing_mm) or spacing_mm <= 0:
        raise ValueError("Spacing must be finite and positive")
    pieces = []
    total_samples = 1
    for a, b in zip(p[:-1], p[1:]):
        length = np.linalg.norm(b - a)
        if length <= 1e-10:
            continue
        ratio = length / spacing_mm
        if not np.isfinite(ratio) or ratio > 1_000_000:
            raise ValueError("Spacing requests too many samples")
        count = int(np.ceil(ratio))
        total_samples += count
        if total_samples > 1_000_000:
            raise ValueError("Path exceeds one million samples")
        pieces.append(a + np.arange(count)[:, None] / count * (b - a))
    if not pieces:
        raise ValueError("Path has zero length")
    return np.vstack([*pieces, p[-1:]])
