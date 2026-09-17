"""Evaluate a reconstructed map with one consistent set of input conventions."""
import sys

from ._common import PAPER, colored_arrays, validate_search
from .ccs import compute_ccs
from .color_distance import compute_color_distance, to_db
from .lcr import compute_lcr

METRICS = ("cd", "cf", "lcr", "ccs")


def evaluate_map(reconstruction, reference=None, *, metrics=METRICS, tau=0.1,
                 radius=0.5, ccs_voxel_size=0.1, voxel_origin=(0., 0., 0.),
                 singleton_policy="zero", threads=8, batch_size=1024,
                 show_progress=False, visualization_dir=None, visualization_max_points=200000,
                 visualization_seed=0, cd_display_max=1.0, ccs_display_max=0.1):
    """Return metrics and the parameters/counts needed to interpret them.

    Inputs are Open3D point clouds with normalized RGB and metric coordinates.
    CD, CF and LCR require a reference. CCS can be requested on its own.
    No registration, cropping or downsampling is performed. Zero CD produces
    float('inf') CF; CLI/JSON output encodes this as the string '+inf'.
    """
    requested = (metrics,) if isinstance(metrics, str) else tuple(metrics)
    if not requested or any(name not in METRICS for name in requested):
        raise ValueError(f"metrics must be a nonempty selection from {METRICS}")
    points, _ = colored_arrays(reconstruction, "reconstructed map")
    validate_search(threads, batch_size)
    needs_reference = any(name in requested for name in ("cd", "cf", "lcr"))
    if needs_reference and reference is None:
        raise ValueError("CD, CF and LCR require a reference map; use metrics=('ccs',) without one")
    reference_points = None
    if reference is not None:
        reference_points = len(colored_arrays(reference, "reference map")[0])
    report = {"schema_version": 1, "paper": PAPER, "rgb_channels": "[0, 1]",
              "map_points": len(points), "reference_points": reference_points,
              "preprocessing": {"registration": False, "downsampling": False},
              "metrics": {}, "details": {}}
    visualizer = None
    if visualization_dir is not None:
        from .visualization import Visualizations
        visualizer = Visualizations(visualization_dir, visualization_max_points,
                                    visualization_seed, cd_display_max, ccs_display_max)
    values, details = report["metrics"], report["details"]
    if "cd" in requested or "cf" in requested:
        if show_progress:
            print("Computing bidirectional Color Distance...", file=sys.stderr, flush=True)
        cd, forward, backward, errors_forward, errors_reverse = compute_color_distance(
            reconstruction, reference, threads, show_progress=show_progress)
        if "cd" in requested:
            values.update(CD=cd, CD_rgb255=255.0 * cd)
        if "cf" in requested:
            values["CF_dB"] = to_db(cd)
        details["color_distance"] = {"mean_map_to_reference": forward,
                                      "mean_reference_to_map": backward,
                                      "direction_weights": [0.5, 0.5]}
        if visualizer:
            visualizer.color_distance(reconstruction, reference, errors_forward, errors_reverse)
        del errors_forward, errors_reverse
    if "lcr" in requested:
        lcr = compute_lcr(reconstruction, reference, tau=tau, r_g=radius,
                          thread_num=threads, batch_size=batch_size,
                          show_progress=show_progress, return_details=True, return_data=visualizer is not None)
        if visualizer:
            lcr, data = lcr
            visualizer.recall(reconstruction, reference, data, lcr)
            del data
        values.update(LCR=lcr["LCR"], LCR_percent=lcr["LCR_percent"])
        details["lcr"] = lcr
    if "ccs" in requested:
        if show_progress:
            print("Computing Color Consistency Score...", file=sys.stderr, flush=True)
        ccs = compute_ccs(reconstruction, ccs_voxel_size, voxel_origin=voxel_origin,
                          singleton_policy=singleton_policy, return_details=True, return_data=visualizer is not None)
        if visualizer:
            ccs, data = ccs
            visualizer.consistency(data, ccs)
            del data
        values["CCS"] = ccs["CCS"]
        details["ccs"] = ccs
    if visualizer:
        report["visualization"] = visualizer.finish(report)
    return report
