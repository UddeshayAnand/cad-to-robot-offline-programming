"""Create synthetic meshes"""
from pathlib import Path
import trimesh

out = Path(__file__).parent / "meshes"
out.mkdir(exist_ok=True)
box = trimesh.creation.box(extents=[100, 60, 10])
box.export(out / "plate.stl")
second = box.copy()
second.apply_translation([150, 0, 0])
trimesh.util.concatenate([box, second]).export(out / "two_plates.stl")
trimesh.creation.cylinder(radius=40, height=10, sections=64).export(out / "cylinder.stl")
print(f"Wrote three synthetic fixtures to {out}")
