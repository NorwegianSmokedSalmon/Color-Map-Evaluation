"""Explicit geometry-only frame alignment, separate from color scoring."""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d

from ._common import write_report
from .preprocessing import load_transform


def align_geometry(source, reference, *, initial=None, coarse_voxel=3.0,
                   scales=(2.0, 1.0, 0.5), distances=(5.0, 2.0, 1.0), seeds=3):
    """FPFH/RANSAC initialization followed by robust point-to-plane rigid ICP.

    Only geometry is used. Output source-to-reference is SE(3), without scale.
    Fitness measures source coverage inside each stated distance gate, not a
    guarantee of correct global alignment. Inspect the overlay before scoring.
    """
    for name, cloud in [('source', source), ('reference', reference)]:
        xyz = np.asarray(cloud.points)
        if len(xyz) < 3 or not np.isfinite(xyz).all():
            raise ValueError(f'{name} requires at least three finite points')
    if (not np.isfinite(coarse_voxel) or coarse_voxel <= 0 or seeds < 1 or
            len(scales) != len(distances) or not scales or
            not np.isfinite([*scales, *distances]).all() or min(*scales, *distances) <= 0):
        raise ValueError('Invalid registration voxel sizes, gates or seeds')
    reg = o3d.pipelines.registration
    report = {'method': 'geometry-only FPFH RANSAC + robust multiscale point-to-plane ICP',
              'scale_optimization': False, 'stages': []}
    if initial is None:
        down = [p.voxel_down_sample(coarse_voxel) for p in (source, reference)]
        features = []
        for p in down:
            p.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=coarse_voxel*2, max_nn=30))
            features.append(reg.compute_fpfh_feature(p, o3d.geometry.KDTreeSearchParamHybrid(
                radius=coarse_voxel*5, max_nn=100)))
        candidates = []
        for seed in range(seeds):
            o3d.utility.random.seed(seed)
            result = reg.registration_ransac_based_on_feature_matching(*down, *features, True,
                coarse_voxel*1.5, reg.TransformationEstimationPointToPoint(False), 3,
                [reg.CorrespondenceCheckerBasedOnEdgeLength(.9),
                 reg.CorrespondenceCheckerBasedOnDistance(coarse_voxel*1.5)],
                reg.RANSACConvergenceCriteria(100000, .999))
            candidates.append(result)
            report['stages'].append({'stage': 'RANSAC', 'seed': seed, 'voxel_m': coarse_voxel,
                                     'gate_m': coarse_voxel*1.5, 'fitness': result.fitness,
                                     'inlier_rmse_m': result.inlier_rmse})
        best = max(candidates, key=lambda r: (r.fitness, -r.inlier_rmse))
        transform = best.transformation
    else:
        transform = np.asarray(initial, dtype=float)
        report['initial_transform'] = transform.tolist()
    for size, gate in zip(scales, distances):
        a, b = (p.voxel_down_sample(size) for p in (source, reference))
        b.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=size*3, max_nn=40))
        result = reg.registration_icp(a, b, gate, transform,
            reg.TransformationEstimationPointToPlane(reg.TukeyLoss(gate)),
            reg.ICPConvergenceCriteria(max_iteration=70))
        transform = result.transformation
        report['stages'].append({'stage': 'ICP', 'voxel_m': size, 'gate_m': gate,
                                 'fitness': result.fitness, 'inlier_rmse_m': result.inlier_rmse})
    report['map_to_reference'] = transform.tolist()
    return transform, report


def main(argv=None):
    parser = argparse.ArgumentParser(description='Estimate one geometry-only rigid map-to-reference transform')
    parser.add_argument('--source', required=True, help='Geometry in the reconstruction frame; independent LiDAR preferred')
    parser.add_argument('--reference', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--initial', help='Optional initial 4x4 SE(3) matrix; skips global RANSAC')
    parser.add_argument('--coarse-voxel', type=float, default=3.)
    parser.add_argument('--seeds', type=int, default=3)
    args = parser.parse_args(argv)
    try:
        for path in (args.source, args.reference):
            if not Path(path).is_file():
                raise FileNotFoundError(path)
        output = Path(args.output_dir)
        output.mkdir(parents=True, exist_ok=True)
        if (output/'map_to_reference.txt').exists():
            raise ValueError('Alignment output already exists; use a new directory')
        source = o3d.io.read_point_cloud(args.source).voxel_down_sample(.5)
        reference = o3d.io.read_point_cloud(args.reference).voxel_down_sample(.5)
        transform, report = align_geometry(source, reference,
            initial=load_transform(args.initial) if args.initial else None,
            coarse_voxel=args.coarse_voxel, seeds=args.seeds)
        report.update(source=str(Path(args.source).resolve()), reference=str(Path(args.reference).resolve()),
                      source_points=len(source.points), reference_points=len(reference.points),
                      input_downsampling_m=.5)
        np.savetxt(output/'map_to_reference.txt', transform, fmt='%.12g')
        write_report(output/'alignment.json', report)
        source.transform(transform).paint_uniform_color([.9, .25, .1])
        reference.paint_uniform_color([.1, .6, .9])
        if not o3d.io.write_point_cloud(str(output/'alignment_overlay.ply'), source+reference):
            raise OSError('Failed to save alignment overlay')
        print(f'Transform: {output / "map_to_reference.txt"}')
        print(report['stages'][-1])
    except (ValueError, OSError) as exc:
        parser.error(str(exc))


if __name__ == '__main__':
    main()
