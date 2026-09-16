# Rotate UV - Native Auto Seam clean repository

This is the seam-first branch. It generates seams, previews them in 3ds Max, and applies them only when approved. It does not use xatlas flatten mapping as the final Auto Seam result.

The Windows build has two routes:

1. Prefer a published Windows OptCuts runtime from TopoPPI if available.
2. Otherwise build upstream OptCuts from source using an **explicit portable CMake 3.25.2 executable**, so nested TBB/libigl CMake calls do not silently use CMake 4.x.

## Build

Push the whole folder to a new empty GitHub repository. The supplied `PUSH_TO_NEW_GITHUB_REPO.bat` is recommended because it preserves the hidden `.github` workflow folder.

Then go to **Actions -> Build Windows Auto Seam Worker -> Run workflow**.

Download the artifact **RotateUV-Native-Auto-Seam-Windows** and extract the entire artifact into one folder.

## Max use

Open `Rotate_UV_PRO_NATIVE_AUTO_SEAM_V1.ms` in the MAXScript Editor and Evaluate All. Use **Generate -> Preview** first. If the seam preview is useful, use **Apply -> Unfold**.
