"""Machine-readable waypoints and deliberately non-executing KRL previews."""
import csv
from pathlib import Path

from .frames import xyzabc


def export_paths(paths, destination):
    out = Path(destination)
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        raise ValueError("Output directory must be new or empty; choose a new run directory")
    rows = [(i, j, *xyzabc(p)) for i, path in enumerate(paths) for j, p in enumerate(path)]
    if not rows or any(len(path) < 2 for path in paths):
        raise ValueError("Export requires paths with at least two poses each")
    out.mkdir(parents=True, exist_ok=True)
    with (out / "waypoints.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["path_id", "point_id", "x_mm", "y_mm", "z_mm", "a_deg", "b_deg", "c_deg"])
        writer.writerows(rows)
    (out / "waypoints.txt").write_text(
        "# TCP poses in robot-base frame; mm, degrees; separate path IDs are not connected\n" +
        "\n".join(f"{i} {j} " + " ".join(f"{v:.6f}" for v in values)
                  for i, j, *values in rows) + "\n", encoding="utf-8")
    for i, path in enumerate(paths):
        name = f"CONTOUR_{i:03d}"
        src = ["; RECONSTRUCTION PREVIEW ONLY - NO EXECUTABLE MOTION", f"DEF {name}()",
               "; TCP values are expressed in robot-base coordinates.",
               "; Base/tool setup, initial configuration and transfers are unresolved.",
               "; All LIN statements below are comments; no welding I/O is generated."]
        dat = ["; UNVALIDATED CARTESIAN DATA - no S/T or external-axis configuration",
               f"DEFDAT {name}"]
        for j, pose in enumerate(path):
            text = ",".join(f"{key} {value:.6f}" for key, value in zip("XYZABC", xyzabc(pose)))
            dat.append(f"DECL FRAME P{j:05d}={{{text}}}")
            src.append(f"; LIN P{j:05d}")
        (out / f"{name}.SRC").write_text("\n".join([*src, "END", ""]), encoding="ascii")
        (out / f"{name}.DAT").write_text("\n".join([*dat, "ENDDAT", ""]), encoding="ascii")
