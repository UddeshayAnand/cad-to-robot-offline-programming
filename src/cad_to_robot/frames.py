"""Rigid transforms, with positions in mm and Euler angles in degrees."""
import numpy as np
from scipy.spatial.transform import Rotation


def frame(xyz=(0, 0, 0), abc=(0, 0, 0)) -> np.ndarray:
    values = np.asarray([xyz, abc], dtype=float)
    if values.shape != (2, 3) or not np.isfinite(values).all():
        raise ValueError("XYZ and ABC must each contain three finite values")
    t = np.eye(4)
    t[:3, :3] = Rotation.from_euler("ZYX", abc, degrees=True).as_matrix()
    t[:3, 3] = xyz
    return t


def validate_frame(t: np.ndarray) -> np.ndarray:
    t = np.asarray(t, dtype=float)
    if t.shape != (4, 4) or not np.isfinite(t).all():
        raise ValueError("Expected a finite 4x4 transform")
    r = t[:3, :3]
    if (not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8, rtol=0) or
            not np.allclose(r.T @ r, np.eye(3), atol=1e-8, rtol=0) or
            not np.isclose(np.linalg.det(r), 1, atol=1e-8, rtol=0)):
        raise ValueError("Transform must be rigid and right handed")
    return t


def tcp_poses(points, base_from_workpiece, tool_abc=(0, 0, 180)):
    """Fixed tool orientation relative to workpiece; no automatic weld angles."""
    base = validate_frame(base_from_workpiece)
    return np.asarray([base @ frame(p, tool_abc) for p in points])


def flange_poses(tcp, flange_from_tcp):
    """T_base_flange = T_base_tcp @ inverse(T_flange_tcp)."""
    return np.asarray(tcp) @ np.linalg.inv(validate_frame(flange_from_tcp))


def xyzabc(pose):
    t = validate_frame(pose)
    return np.r_[t[:3, 3], Rotation.from_matrix(t[:3, :3]).as_euler("ZYX", degrees=True)]
