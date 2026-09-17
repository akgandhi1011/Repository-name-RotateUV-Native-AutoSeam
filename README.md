# RotateUV Native Auto Seam V2 - Feature-Aware

This is the replacement for the OptCuts-first worker.

The worker is now self-contained C++17 and does **not** require OptCuts, CMake, TBB, DLLs, or any third-party runtime.

## Seam planning

1. Treats true/open mesh borders as already free (not proposed as seams).
2. Detects continuous structural feature cycles from dihedral/topology flow.
3. Detects planar cap / transition separator loops while rejecting individual smooth side quads.
4. Splits the mesh into regions using those structural cuts.
5. Adds controlled longitudinal openings between separated boundary loops for tube/strip-like regions.
6. Adds a single controlled fallback slit for a large fully closed smooth region instead of many random cuts.
7. Removes tiny dangling seam twigs.

The MaxScript workflow remains:

**Generate -> Preview -> Apply -> Unfold**

## Build

Create a repository with this package, push it, then run **Build RotateUV Feature-Aware Auto Seam V2** under GitHub Actions. Download the artifact `RotateUV-Feature-Aware-Auto-Seam-V2-Windows`.

The artifact contains only:
- `RotateUV_AutoSeam.exe`
- `Rotate_UV_PRO_NATIVE_AUTO_SEAM_V1.ms`
- `Rotate_UV_PRO_NATIVE_AUTO_SEAM_V1.mcr`

Keep the three files together.
