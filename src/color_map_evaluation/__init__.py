"""Color map metrics from LiDAR-VGGT, arXiv:2511.01186, Eqs. (16)-(18)."""
from .ccs import compute_ccs, compute_cis
from .color_distance import compute_color_distance, to_db
from .lcr import compute_lcr, compute_LCCR
from .evaluate import evaluate_map

__version__ = "0.1.0"
compute_color_fidelity = to_db

__all__ = ["compute_ccs", "compute_color_distance", "compute_color_fidelity",
           "compute_lcr", "evaluate_map", "compute_cis", "compute_LCCR"]
