# Technical report: geometric offline programming reconstruction

## Problem and scope

Convert CAD-derived tessellated geometry into inspectable Cartesian paths. The motivating workflow is welding-oriented offline programming, but the present implementation reconstructs only its geometric and export stages. It has no access to original employer files, a calibrated robot cell, a simulator, or a controller.

## Geometry method

STL input is loaded without automatic processing, checked for finite coordinates, scaled to millimetres, and cleaned by vertex merging and duplicate/degenerate face removal. Open meshes are rejected instead of silently filling holes.

The user selects an interior Z plane. Trimesh intersects it with triangles and constructs discrete contours. Closed loops are independently retained, normalized to a lexicographic start, and ordered consistently in XY. Loops are candidate sections, not automatically recognized weld seams; nested contours have no inferred joint semantics.

For each segment of length L and requested maximum spacing h, use n=ceil(L/h) subdivisions. Linear interpolation preserves every input corner. This avoids a common defect of globally spaced samples: skipping a corner and cutting across it. The closing point is retained. Collinear triangle intersections are also retained, so sample count depends on tessellation and floating-point rounding; spacing is a maximum rather than a claim of uniformity.

## Coordinate conventions

Column-vector homogeneous transforms are used. `T_A_B` maps coordinates from frame B to frame A. Distances are mm; Euler angles are degrees with `R = Rz(A) Ry(B) Rx(C)`.

```text
T_base_tcp = T_base_workpiece @ T_workpiece_tcp
T_base_flange = T_base_tcp @ inverse(T_flange_tcp)
```

The workpiece TCP position follows the sampled contour. Its orientation is a user-specified constant in the workpiece frame; the default C=180 degrees points tool +Z downward. The default 50 mm TCP offset and base translation (300,0,100) mm are illustrative assumptions. Neither is historical calibration. There is no torch standoff or automatic approach/retract movement. Euler representation can be ambiguous at gimbal lock; this is separate from a robot joint singularity.

TCP poses are exported in base coordinates. The flange calculation is reported through bounds in the metrics, not substituted for TCP coordinates in KRL. Applying a workpiece base transform again downstream would double-transform the data.

## KRL preview contract

Each contour gets a separate module name and Cartesian FRAME declarations. Every motion statement is a comment. No initial joint configuration, S/T values, external axes, base/tool activation, velocity, welding I/O, transfer moves, or controller-specific initialization is fabricated. These files are examples for inspection and future postprocessor development.

## Experiments

Reproduce the supplied outputs from the repository root after installation:

```text
python examples/generate_fixtures.py
python examples/run_all.py --output output
python -m pytest -q
```

Observed environment: Windows, Python 3.12.7, NumPy 1.26.4, Matplotlib 3.9.2, SciPy 1.13.1, Trimesh 4.4.9, NetworkX 3.3, pytest 7.4.4. Final verification used a fresh virtual environment without inherited scientific packages. This caught and corrected an initially undeclared NetworkX dependency used by Trimesh contour assembly. All 20 tests passed; `pip check` reported no broken requirements. Exact runtime/test dependency versions are recorded in `requirements.txt`. GitHub Actions has not yet run remotely. Use a new output directory for each reproduction; compare it with the saved example results.

The 100 by 60 mm rectangular section measures 320 mm, matching its analytic perimeter. Two separated rectangles measure 320 mm each with no generated link between them. The 64-sided radius-40 mm cylinder section measures 251.226492993 mm after STL round-trip; the ideal circle is 251.327412287 mm, a deficit of approximately 0.100919 mm (0.0402%). This difference is tessellation error, not robot error. Floating-point spacing values can exceed 5 mm by about 1e-14 mm.

Twenty tests cover analytic rectangle and polygon lengths, corner retention, disconnected paths, transform composition, tool offsets, invalid spacing and planes, invalid reflections and slight scaling, metre conversion, open-mesh rejection, CSV round-trip, absence of executable preview motion, end-to-end CLI output, existing-output protection, missing inputs, empty exports and excessive sampling. The sampled cylinder is tested against its polygon perimeter independently of the ideal circle approximation.

## Limitations and next validation stage

Current fixtures are small, synthetic, horizontal closed-solid sections. Arbitrary seam selection, native SolidWorks geometry, open sheet assemblies, tool orientation optimization, timing, trajectory smoothing, joint limits, inverse kinematics, collision checking, singularities, and welding physics are outside the implemented scope. A constant-orientation geometric path is not proof of a feasible robot trajectory.

Current public KR 4 R600 reach/load specifications do not determine an authoritative kinematic chain. Obtain model-specific joint transforms, limits, tool/load data, controller version and cell fixtures before extending the implementation. Validate known poses, then whole paths and transitions in the configured simulator. Record simulator version, robot model source, frame calibration, failures/corrections and outputs before making any simulation validation claim.
