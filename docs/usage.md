# Advanced usage

For installation and a complete example, start with the [README](../README.md).
All commands below run from the repository root after activating the environment.

## Configuration and metric selection

[examples/evaluation.json](../examples/evaluation.json) contains a complete
configuration. Keys use command line option names with underscores, such as
`ccs_voxel_size`. Relative paths resolve against the JSON file, while paths passed
directly on the command line resolve against the working directory. Command line
arguments override configuration values.

```bash
color-map-evaluate --config path/to/evaluation.json \
  --threads 8 --batch-size 8192 \
  --output results/experiment/metrics.json \
  --visualization-dir results/experiment/visualizations
```

Choose a new visualization directory for each run. An existing completed
visualization report is protected from accidental overwrite.

Select individual metrics with `--metrics cd cf lcr ccs`. CD and CF share the same
nearest-neighbor calculation. CCS can run without a reference:

```bash
color-map-evaluate --map reconstruction.ply --metrics ccs \
  --ccs-voxel-size 0.1 --output results/ccs.json
```

Reports record metric values, point and voxel counts, thresholds, RGB units,
explicit preprocessing, and input SHA-256 values. Perfect color fidelity is
encoded as the string `"+inf"` in strict JSON. `--progress` shows CD and LCR progress;
`--threads` controls spatial-query parallelism and `--batch-size` controls LCR query
batches. Neither option changes the metric definitions.

## Coordinate alignment and region of interest

The metric API expects aligned inputs and performs no registration, cropping, or
downsampling. The CLI applies a transform or crop only when explicitly requested.

`color-map-align` estimates a geometry-only rigid source-to-reference transform
using FPFH/RANSAC initialization and robust multiscale point-to-plane ICP:

```bash
color-map-align --source lidar_world.ply --reference reference.ply \
  --output-dir results/alignment
```

Its outputs are `map_to_reference.txt`, `alignment.json`, and
`alignment_overlay.ply` (source in red/orange, reference in blue). Inspect the
overlay before scoring. Prefer an independent LiDAR map for alignment and apply
the same saved transform to every reconstruction in that frontend frame.

The registration voxel scales target large outdoor maps. `--coarse-voxel` controls
global initialization; `--initial initial_transform.txt` skips RANSAC and starts
ICP from an existing estimate. Alignment does not correct scale or trajectory
deformation. Colors and evaluation scores do not select the transform.

Apply the saved 4×4 SE(3) matrix when evaluating:

```bash
color-map-evaluate --map reconstruction.ply --reference reference.ply \
  --map-transform results/alignment/map_to_reference.txt \
  --output results/aligned/metrics.json \
  --visualization-dir results/aligned/visualizations
```

Optional `--roi xmin ymin zmin xmax ymax zmax` crops **both** inputs to a fixed box
in the reference frame after transformation. Omit it to evaluate the full inputs.
Use the same reference population and ROI across comparisons; selecting reference
points based on their proximity to each reconstruction would inflate recall.
CCS uses the transformed coordinates and the configured voxel origin.

## Visualization settings

| Option | Default | Effect |
| --- | --- | --- |
| `--visualization-max-points` | `200000` | Maximum sampled points per exported cloud |
| `--visualization-seed` | `0` | Display sampling seed |
| `--cd-display-max` | `1.0` | Upper color limit for RGB L2 error |
| `--ccs-display-max` | `0.1` | Upper color limit for covariance trace |

These options affect visualization only. Metrics and histograms use the full
input populations. Keep display limits fixed across comparisons. Values above a
color limit saturate in PNG/PLY outputs; NPZ files retain unclipped values.

LCR separates recalled points, color misses, and points with no geometric neighbor.
For each exported pair, the reconstructed point is the closest spatially among
all neighbors that satisfy both thresholds; input index breaks exact distance
ties. Many reference points may share a reconstructed point. `lcr_pair_lines.ply`
shows at most 2,000 pairs and is absent when there are no matches.

CCS singleton voxels are gray because sample covariance is undefined for a single
point. The supported-voxel view shows only voxels with at least two points, without
changing the reported score or singleton policy. A large singleton fraction can
strongly reduce CCS under the default `zero` policy; inspect the counts in the
report. For formulas and conventions, see [metric definitions](metrics.md).

## Python API

```python
import open3d as o3d
from color_map_evaluation import evaluate_map

reconstruction = o3d.io.read_point_cloud("reconstruction.ply")
reference = o3d.io.read_point_cloud("reference.ply")
report = evaluate_map(
    reconstruction,
    reference,
    tau=0.1,
    radius=0.5,
    ccs_voxel_size=0.1,
    visualization_dir="results/python/visualizations",
)
print(report["metrics"])
```

For individual metrics, import `compute_color_distance`, `compute_color_fidelity`,
`compute_lcr`, or `compute_ccs`. `compute_lcr(..., return_data=True)` and
`compute_ccs(..., return_data=True)` expose per-point or per-voxel diagnostics.

## Individual commands and compatibility

```bash
color-map-cd reconstruction.ply reference.ply --json results/cd_cf.json
color-map-lcr --map reconstruction.ply --truth reference.ply --json results/lcr.json
color-map-ccs --pcd reconstruction.ply --voxel-size 0.1 --json results/ccs.json
```

The `cal_colordist.py`, `cal_lccr.py`, and `cal_cis.py` scripts remain as
compatibility entry points after installation. The historical LCCR and CIS names
compute the current reference-to-reconstruction LCR and CCS definitions.

## Development

```bash
python -m pip install -e '.[dev]'
OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 MPLBACKEND=Agg \
  python -m unittest discover -s tests -v
python -m build
```

[requirements-tested.txt](../requirements-tested.txt) records the direct dependency
versions used for validation. Install with
`python -m pip install -c requirements-tested.txt '.[dev]'` to use those versions.
Tests cover metric formulas, matching direction and thresholds, voxel conventions,
visualization exports, configuration, and command line interfaces.
