# Metric definitions and evaluation conventions

Reference: [LiDAR-VGGT v1, Section III-D](https://arxiv.org/html/2511.01186v1#S3.SS4).
Let `s` be the reconstructed map and `r` the reference. Each point has a spatial
coordinate `x` in meters and an RGB vector `c` with channels in `[0,1]`.

## CD and CF — Eq. (16)

Let `pi(i)` select the spatial nearest neighbor of reconstructed point `i` in
the reference, and `pi'(j)` select the reverse match:

```text
CD = 0.5 * mean_i ||c_s[i] - c_r[pi(i)]||_2
   + 0.5 * mean_j ||c_r[j] - c_s[pi'(j)]||_2
CF = -20 * log10(CD)
```

The per-pair error is RGB L2, not channel RMSE. Each direction has weight 1/2,
even for different point counts. Every point is included; nearest-neighbor
matches have no maximum spatial distance. Consequently, nonoverlapping regions
also affect CD. Exact spatial ties follow the spatial tree's neighbor selection.

`compute_color_distance(source, reference)` returns
`(CD, mean_source_to_reference, mean_reference_to_source, errors_forward, errors_reverse)`.
`compute_color_fidelity(CD)` returns CF in dB. Error arrays and scores use float64.

CD ranges from 0 to `sqrt(3)` with the selected RGB convention. CF can therefore
be negative. A zero CD gives `+inf`, whereas every positive CD gives a finite CF;
the implementation does not clip small positive errors. `CD_rgb255 = 255*CD`
changes the RGB channel units, not the averaging convention. CF is always
computed from the `[0,1]` channel result.

## LCR — Eq. (17)

```text
LCR = number of reference points with at least one eligible map neighbor / N_reference
eligible = spatial_distance <= radius AND RGB_L2_distance <= 3*tau
```

The neighborhood is in the reconstructed map. Any color-compatible neighbor
suffices, including one that is not the spatial nearest neighbor. A reference
point with an empty neighborhood is a miss and remains in the denominator.
The result is a fraction in `[0,1]`; CLI output additionally includes percent.

Defaults `tau=0.1` and `radius=0.5 m` follow Section IV-B / Fig. 7. With these
settings the RGB L2 cutoff is `0.3`. Spatial searches are exact, processed in
batches controlled by `batch_size`; changing the batch size does not change the
metric. The individual LCR command can save matched maps, retaining each input
point once even when it participates in multiple neighborhoods.

## CCS — Eq. (18)

For each occupied voxel with `n >= 2` points:

```text
C_voxel = sum_j ||c[j] - mean(c)||_2^2 / (n - 1)
CCS = mean_over_voxels(C_voxel)
```

This is the trace of the sample RGB covariance matrix. A two-point voxel is
valid. Voxels receive equal weight regardless of their point counts. Centered
colors are squared directly, without PCA or pre-downsampling.

The grid is `floor((x - voxel_origin) / voxel_size)`. The default voxel edge
is `0.1 m` and the origin is `(0,0,0)`. Both are implementation defaults because
the paper does not specify them. Use the same grid and color units when
comparing maps.

The paper's `n-1` denominator is undefined for singleton voxels. The explicit
`singleton_policy` option resolves this edge case:

| Policy | Singleton treatment | Averaging denominator |
| --- | --- | --- |
| `zero` (default) | Assign zero trace | All occupied voxels |
| `exclude` | Skip singleton voxels | Voxels containing at least two points |
| `error` | Raise an error | No result if any singleton is present |

The report includes occupied, singleton and evaluated voxel counts. An all-singleton
map has CCS 0 with `zero`, and raises an error with `exclude` or `error`.
A high singleton fraction can strongly reduce CCS under `zero`; compare the counts
alongside the score. Averaging colors before evaluation changes CCS, so evaluate
the original map whenever possible.

## Inputs and report format

Empty maps, missing RGB, non-finite values, and RGB channels outside `[0,1]` are
rejected. The metric API expects a shared coordinate frame and performs no downsampling.
The CLI supports an explicit rigid map-to-reference transform and a fixed ROI;
both are recorded in the report. No scale correction is performed. The individual CD
command retains an explicit `--voxel-size` option and records it in its report.

The unified API returns `schema_version`, the paper URL, color units, map point
counts, preprocessing flags, selected `metrics`, and per-metric `details`.
The CLI adds package version and input paths. Reports use strict JSON; positive
infinity is encoded as `"+inf"`. The Python API keeps it as a floating-point
infinity. The legacy names CIS and LCCR are retained only as compatibility names;
they now compute CCS and reference-to-reconstruction LCR.

## Relation to the paper's numerical tables

The equations and published LCR settings are implemented directly. The paper does
not fully specify RGB units, CCS voxel settings, singleton handling or table
scaling. For example, Table III reports AMtown01 CD 82.93 and CF 15.81 dB;
using `82.93 / 255` as the normalized CD yields approximately 9.76 dB.
This package defines its units explicitly and does not apply an inferred scaling
factor to match the table. Formula agreement alone does not establish numerical
reproduction of Table III.

## Diagnostic data and visualization

`compute_lcr(..., return_data=True)` returns `(report, data)` with a reference
hit mask, geometric-neighbor mask and one eligible map index per recalled point
(`-1` on misses). The index minimizes spatial distance among color-compatible
neighbors, with input index as the deterministic tie-break. This witness
selection leaves the any-neighbor recall definition unchanged.

`compute_ccs(..., return_data=True)` returns `(report, data)` containing voxel
centers, occupancy counts, covariance traces and the evaluated-voxel mask.
Singletons are visually gray even when the selected scoring policy assigns zero.

`evaluate_map(..., visualization_dir=...)` exports all selected metric
visualizations. Diagnostic sampling affects only exports; aggregate scores and
histograms still use the complete metric population. Sampled point indices and
unclipped scalar values are stored beside each colored PLY in NPZ files. LCR
pair-cloud rows correspond one-to-one, while map input indices may repeat.

CD/CF are influenced by nonoverlap because their geometric nearest-neighbor
search has no distance gate. LCR visualizations separate lack of geometric
coverage from failure of the color test. These are diagnostics of the supplied
maps after the recorded frame conversion, not a reproduction of a paper table.
