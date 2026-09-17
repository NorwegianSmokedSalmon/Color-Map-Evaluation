"""Explicit rigid frame conversion and fixed spatial ROI; no metric downsampling."""
from pathlib import Path
import hashlib

import numpy as np


def load_transform(path):
    matrix = np.loadtxt(path)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('map transform must be a finite 4x4 matrix')
    rotation = matrix[:3, :3]
    if not (np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8) and
            np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) and
            np.isclose(np.linalg.det(rotation), 1, atol=1e-6)):
        raise ValueError('map transform must be rigid SE(3), without scale or reflection')
    return matrix


def prepare_clouds(reconstruction, reference=None, map_transform=None, roi=None):
    details = {'registration': False, 'downsampling': False,
               'map_input_points': len(reconstruction.points),
               'reference_input_points': len(reference.points) if reference is not None else None}
    if map_transform:
        matrix = load_transform(map_transform)
        reconstruction.transform(matrix)
        details.update(registration='supplied rigid map-to-reference transform',
                       map_transform_path=str(Path(map_transform).resolve()),
                       map_transform_sha256=hashlib.sha256(Path(map_transform).read_bytes()).hexdigest(),
                       map_to_reference=matrix.tolist())
    if roi is not None:
        import open3d as o3d
        bounds = np.asarray(roi, dtype=float)
        if bounds.shape != (6,) or not np.isfinite(bounds).all() or np.any(bounds[3:] <= bounds[:3]):
            raise ValueError('roi must be xmin ymin zmin xmax ymax zmax, with max > min')
        box = o3d.geometry.AxisAlignedBoundingBox(bounds[:3], bounds[3:])
        reconstruction = reconstruction.crop(box)
        if reference is not None:
            reference = reference.crop(box)
        details['roi_in_reference_frame'] = bounds.tolist()
    else:
        details['roi_in_reference_frame'] = None
    details['ccs_frame'] = 'reference' if map_transform else 'input map'
    return reconstruction, reference, details
