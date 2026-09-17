"""Unified command line interface for metrics and headless visualization."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

from . import __version__
from ._common import format_report, read_cloud, write_report
from .evaluate import METRICS, evaluate_map
from .preprocessing import prepare_clouds


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description='Evaluate colored point clouds using CD/CF, LCR and CCS')
    parser.add_argument('--config', help='JSON defaults; paths resolve relative to this file; CLI overrides them')
    parser.add_argument('--version', action='version', version=f'%(prog)s {__version__}')
    parser.add_argument('--map', help='Reconstructed colored PCD or PLY before voxel averaging')
    parser.add_argument('--reference', '--truth', help='Reference colored map')
    parser.add_argument('--map-transform', help='4x4 rigid map-to-reference matrix in a text file')
    parser.add_argument('--roi', nargs=6, type=float, help='Fixed ROI in reference frame: xmin ymin zmin xmax ymax zmax')
    parser.add_argument('--metrics', nargs='+', choices=METRICS, default=list(METRICS))
    parser.add_argument('--tau', type=float, default=0.1, help='LCR uses RGB L2 <= 3*tau')
    parser.add_argument('--radius', type=float, default=0.5, help='LCR radius in meters')
    parser.add_argument('--ccs-voxel-size', type=float, default=0.1)
    parser.add_argument('--voxel-origin', nargs=3, type=float, default=(0., 0., 0.))
    parser.add_argument('--singleton-policy', choices=('zero', 'exclude', 'error'), default='zero')
    parser.add_argument('--threads', type=int, default=8)
    parser.add_argument('--batch-size', type=int, default=4096)
    parser.add_argument('--progress', action='store_true')
    parser.add_argument('--output', help='Save JSON report; also printed to stdout')
    parser.add_argument('--visualization-dir', help='Export PNG, colored PLY and diagnostic NPZ files')
    parser.add_argument('--visualization-max-points', type=int, default=200000, help='Display/export cap only')
    parser.add_argument('--visualization-seed', type=int, default=0)
    parser.add_argument('--cd-display-max', type=float, default=1.0)
    parser.add_argument('--ccs-display-max', type=float, default=0.1)
    preliminary = argparse.ArgumentParser(add_help=False)
    preliminary.add_argument('--config')
    config_path = preliminary.parse_known_args(argv)[0].config
    if config_path:
        try:
            config_path = Path(config_path).resolve()
            defaults = json.loads(config_path.read_text())
            if not isinstance(defaults, dict):
                raise ValueError('Config must be a JSON object')
            allowed = {action.dest for action in parser._actions} - {'help', 'version', 'config'}
            unknown = set(defaults) - allowed
            if unknown:
                raise ValueError(f'Unknown config keys: {sorted(unknown)}')
            for key in ('map', 'reference', 'map_transform', 'output', 'visualization_dir'):
                if defaults.get(key) is not None:
                    defaults[key] = str((config_path.parent/str(defaults[key])).resolve())
            parser.set_defaults(**defaults)
        except (ValueError, OSError, TypeError) as exc:
            parser.error(str(exc))
    args = parser.parse_args(argv)
    if not args.map:
        parser.error('--map is required (or provide map in --config)')
    if not args.reference and any(name != 'ccs' for name in args.metrics):
        parser.error('--reference is required for CD, CF and LCR; use --metrics ccs without a reference')
    started = time.monotonic()
    try:
        print('Loading colored maps...', file=sys.stderr, flush=True)
        reconstruction, reference, preprocessing = prepare_clouds(
            read_cloud(args.map), read_cloud(args.reference) if args.reference else None,
            args.map_transform, args.roi)
        print(f'Evaluating {len(reconstruction.points):,} map points' +
              (f' against {len(reference.points):,} reference points' if reference is not None else ''),
              file=sys.stderr, flush=True)
        report = evaluate_map(reconstruction, reference, metrics=args.metrics, tau=args.tau,
            radius=args.radius, ccs_voxel_size=args.ccs_voxel_size, voxel_origin=args.voxel_origin,
            singleton_policy=args.singleton_policy, threads=args.threads, batch_size=args.batch_size,
            show_progress=args.progress, visualization_dir=args.visualization_dir,
            visualization_max_points=args.visualization_max_points, visualization_seed=args.visualization_seed,
            cd_display_max=args.cd_display_max, ccs_display_max=args.ccs_display_max)
        report.update(package_version=__version__, map=str(Path(args.map).resolve()),
                      reference=str(Path(args.reference).resolve()) if args.reference else None,
                      preprocessing=preprocessing, elapsed_seconds=time.monotonic()-started,
                      configuration={k: v for k, v in vars(args).items() if k != 'config'})
        if config_path:
            report['config_file'] = str(config_path)
        # Pin the input bytes, without allocating a second full point-cloud copy.
        report['input_sha256'] = {}
        for name, path in [('map', args.map), ('reference', args.reference)]:
            if path:
                digest = hashlib.sha256()
                with open(path, 'rb') as stream:
                    for block in iter(lambda: stream.read(8*1024*1024), b''):
                        digest.update(block)
                report['input_sha256'][name] = digest.hexdigest()
        if args.output:
            write_report(args.output, report)
        if args.visualization_dir:
            write_report(Path(args.visualization_dir)/'metrics.json', report)
    except (ValueError, OSError) as exc:
        parser.error(str(exc))
    print(format_report(report), end='')


if __name__ == '__main__':
    main()
