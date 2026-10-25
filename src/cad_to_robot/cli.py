"""Command line entry point"""
import argparse
import hashlib
import json
import platform
from importlib.metadata import version
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from .export import export_paths
from .frames import flange_poses, frame, tcp_poses
from .geometry import load_stl, resample, section_contours


def run(args):
    mesh = load_stl(args.mesh, args.units)
    contours = section_contours(mesh, args.z)
    samples = [resample(p, args.spacing) for p in contours]
    base = frame(args.base_xyz, args.base_abc)
    tool = frame(args.tool_xyz, args.tool_abc)
    poses = [tcp_poses(p, base, args.orientation) for p in samples]
    flanges = [flange_poses(p, tool) for p in poses]
    out = Path(args.output)
    export_paths(poses, out)
    lengths = [float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum()) for p in samples]
    steps = [float(np.linalg.norm(np.diff(p, axis=0), axis=1).max()) for p in samples]
    report = {
        "status": "geometric reconstruction; robot feasibility unverified",
        "environment": {"python": platform.python_version(),
                        **{name: version(name) for name in
                           ("numpy", "scipy", "matplotlib", "trimesh", "networkx", "cad-to-robot")}},
        "input_sha256": hashlib.sha256(Path(args.mesh).read_bytes()).hexdigest(),
        "input_units": args.units, "output_units": "mm", "section_z_mm": args.z,
        "spacing_limit_mm": args.spacing, "path_count": len(samples),
        "waypoint_counts": [len(p) for p in samples], "lengths_mm": lengths,
        "max_steps_mm": steps, "mesh_faces_after_cleanup": len(mesh.faces),
        "T_base_workpiece": base.tolist(), "T_flange_tcp": tool.tolist(),
        "tool_orientation_workpiece_abc_deg": args.orientation,
        "flange_bounds_mm": [np.min(np.concatenate(flanges)[:, :3, 3], axis=0).tolist(),
                             np.max(np.concatenate(flanges)[:, :3, 3], axis=0).tolist()],
        "collision_checked": False, "inverse_kinematics_checked": False,
        "singularities_checked": False, "controller_validated": False,
    }
    (out / "metrics.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection="3d")
    transformed = mesh.vertices @ base[:3, :3].T + base[:3, 3]
    ax.add_collection3d(Poly3DCollection(transformed[mesh.faces], alpha=.10,
                                       facecolor="#64748b", edgecolor="#94a3b8", linewidth=.3))
    for i, path in enumerate(poses):
        xyz = path[:, :3, 3]
        ax.plot(*xyz.T, linewidth=2, label=f"Contour {i}: {len(path)} waypoints")
        ax.scatter(*xyz[0], marker="o", s=45)
        stride = max(1, len(path) // 8)
        for t in path[::stride]:
            ax.quiver(*t[:3, 3], *t[:3, 2], length=8, color="#475569", alpha=.5)
    ax.auto_scale_xyz(*transformed.T)
    extents = np.ptp(transformed, axis=0)
    ax.set_box_aspect(np.maximum(extents, extents.max() * .35))
    ax.zaxis.set_major_locator(MaxNLocator(3))
    ax.xaxis.set_major_locator(MaxNLocator(5))
    ax.yaxis.set_major_locator(MaxNLocator(3))
    ax.tick_params(labelsize=9)
    ax.xaxis.labelpad = 12
    ax.yaxis.labelpad = 12
    ax.zaxis.labelpad = 12
    ax.set(xlabel="Base X (mm)", ylabel="Base Y (mm)", zlabel="Base Z (mm)",
           title="CAD contours to TCP waypoints | synthetic reconstruction")
    ax.legend(loc="upper left")
    fig.text(.5, .03, "Dots: starts | Arrows: tool +Z | No robot feasibility validation", ha="center")
    fig.savefig(out / "paths.png", dpi=160)
    plt.close(fig)
    return report


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mesh", type=Path)
    p.add_argument("--units", choices=["mm", "m"], required=True)
    p.add_argument("--z", type=float, required=True, help="Section Z in workpiece mm")
    p.add_argument("--spacing", type=float, default=5)
    p.add_argument("--base-xyz", nargs=3, type=float, default=[300, 0, 100])
    p.add_argument("--base-abc", nargs=3, type=float, default=[0, 0, 0])
    p.add_argument("--tool-xyz", nargs=3, type=float, default=[0, 0, 50])
    p.add_argument("--tool-abc", nargs=3, type=float, default=[0, 0, 0])
    p.add_argument("--orientation", nargs=3, type=float, default=[0, 0, 180])
    p.add_argument("--output", type=Path, default=Path("output/demo"))
    args = p.parse_args()
    try:
        report = run(args)
    except (ValueError, OSError) as exc:
        p.error(str(exc))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
