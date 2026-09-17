# V2.2 / Native Unfold V2 changelog

- Kept current feature-aware Auto Seam behavior frozen as the baseline.
- Replaced custom Native Unfold V1 solver with libigl 2.6.0 LSCM + SLIM.
- Added harmonic initialization fallback for LSCM initializations containing flips.
- SLIM energy: symmetric Dirichlet.
- Default SLIM iterations: 20.
- Rejects optimized result if it increases local triangle inversions.
- Exact planar projection is retained only for genuinely planar charts (<= 1 degree normal deviation).
- Same `RUVUNFOLD 1` / `RUVUV 1` protocol, so no change to the Max-side UV import architecture.
