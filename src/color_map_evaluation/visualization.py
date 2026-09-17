"""Headless metric diagnostics. Display sampling never changes metric populations."""
from pathlib import Path

import numpy as np
import open3d as o3d

from ._common import write_report


def sample_indices(size, limit, rng):
    return np.arange(size) if size <= limit else np.sort(rng.choice(size, limit, replace=False))


def write_cloud(path, points, colors):
    """Binary PLY, including a valid zero-vertex file for an empty category."""
    points, colors = np.asarray(points), np.asarray(colors)
    if len(points) == 0:
        Path(path).write_text('ply\nformat ascii 1.0\nelement vertex 0\nproperty double x\n'
                              'property double y\nproperty double z\nproperty uchar red\n'
                              'property uchar green\nproperty uchar blue\nend_header\n')
        return
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    cloud.colors = o3d.utility.Vector3dVector(colors)
    if not o3d.io.write_point_cloud(str(path), cloud):
        raise OSError(f'Failed to write {path}')


class Visualizations:
    def __init__(self, directory, max_points=200000, seed=0, cd_max=1.0, ccs_max=0.1):
        if not isinstance(max_points, int) or max_points < 1:
            raise ValueError('visualization max_points must be a positive integer')
        if not all(np.isfinite(x) and x > 0 for x in (cd_max, ccs_max)):
            raise ValueError('Display color limits must be positive and finite')
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        self.plt = plt
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=True)
        if (self.root / 'manifest.json').exists():
            raise ValueError('Visualization directory already contains a completed report; use a new directory')
        self.limit, self.rng = max_points, np.random.default_rng(seed)
        self.cd_max, self.ccs_max = cd_max, ccs_max
        self.seed = seed
        self.manifest = {'sampling': 'uniform without replacement for display only; all input points evaluated',
                         'max_points_per_cloud': max_points, 'seed': seed,
                         'cd_display_range': [0, cd_max], 'ccs_display_range': [0, ccs_max],
                         'units': 'meters; RGB channels [0,1]', 'files': {}}

    def scatter(self, ax, points, colors, title, axes=(0, 1)):
        ax.scatter(points[:, axes[0]], points[:, axes[1]], c=colors, s=.3, linewidths=0,
                   rasterized=True)
        ax.set(xlabel='XYZ'[axes[0]]+' (m)', ylabel='XYZ'[axes[1]]+' (m)', title=title)
        ax.set_aspect('equal', adjustable='box')
        ax.set_facecolor('#eeeeee')

    def scalar(self, ax, points, values, title, upper, cmap='inferno'):
        colors = self.plt.get_cmap(cmap)(np.clip(values / upper, 0, 1))[:, :3]
        self.scatter(ax, points, colors, title)
        from matplotlib.cm import ScalarMappable
        from matplotlib.colors import Normalize
        self.plt.colorbar(ScalarMappable(norm=Normalize(0, upper), cmap=cmap), ax=ax,
                         label='RGB L2' if cmap == 'inferno' else 'Sample RGB covariance trace',
                         extend='max')
        return colors

    def finish_figure(self, fig, name):
        fig.tight_layout()
        fig.savefig(self.root / name, dpi=170)
        self.plt.close(fig)

    def color_distance(self, reconstruction, reference, forward, reverse):
        fig, axs = self.plt.subplots(2, 2, figsize=(13, 10))
        for row, (cloud, errors, label) in enumerate(((reconstruction, forward, 'map_to_reference'),
                                                    (reference, reverse, 'reference_to_map'))):
            ids = sample_indices(len(errors), self.limit, self.rng)
            xyz, values = np.asarray(cloud.points)[ids], errors[ids]
            colors = self.scalar(axs[row, 0], xyz, values, label.replace('_', ' '), self.cd_max)
            write_cloud(self.root/f'cd_{label}.ply', xyz, colors)
            np.savez_compressed(self.root/f'cd_{label}.npz', input_indices=ids, rgb_l2=values)
            # Histogram uses all points, while rendered maps use the recorded sample.
            counts, edges = np.histogram(errors, bins=100, range=(0, np.sqrt(3)))
            axs[row, 1].stairs(counts / len(errors), edges)
            axs[row, 1].axvline(float(errors.mean()), color='red', label=f'Mean = {errors.mean():.4f}')
            axs[row, 1].set(xlabel='RGB L2 error', ylabel='Fraction of all points', title='All-point distribution')
            axs[row, 1].legend()
            self.manifest['files'][f'cd_{label}.ply'] = {'points': len(ids), 'population': len(errors),
                'above_display_max': int(np.count_nonzero(errors > self.cd_max))}
        fig.suptitle('Color Distance: spatial nearest-neighbor RGB error (no spatial cutoff)')
        self.finish_figure(fig, 'color_distance.png')

    def recall(self, reconstruction, reference, data, report):
        xyz, rgb = np.asarray(reference.points), np.asarray(reference.colors)
        map_xyz, map_rgb = np.asarray(reconstruction.points), np.asarray(reconstruction.colors)
        ids = sample_indices(len(xyz), self.limit, self.rng)
        category = np.where(data['hits'][ids], 2, data['geometric'][ids].astype(np.uint8))
        palette = np.array([[.5, .5, .5], [.9, .22, .13], [.1, .7, .35]])
        write_cloud(self.root/'lcr_reference_status.ply', xyz[ids], palette[category])
        np.savez_compressed(self.root/'lcr_reference_status.npz', reference_indices=ids, status=category)
        for name, mask in [('recalled', data['hits']), ('color_miss', data['geometric'] & ~data['hits']),
                           ('no_neighbor', ~data['geometric'])]:
            population = np.flatnonzero(mask)
            subset = population[sample_indices(len(population), self.limit, self.rng)]
            write_cloud(self.root/f'lcr_reference_{name}.ply', xyz[subset], rgb[subset])
            self.manifest['files'][f'lcr_reference_{name}.ply'] = {'points': len(subset), 'population': len(population)}
        all_pairs = np.flatnonzero(data['hits'])
        ref_ids = all_pairs[sample_indices(len(all_pairs), self.limit, self.rng)]
        map_ids = data['map_indices'][ref_ids]
        # Matching row numbers form pairs; map rows can repeat for many-to-one matches.
        write_cloud(self.root/'lcr_pairs_reference.ply', xyz[ref_ids], rgb[ref_ids])
        write_cloud(self.root/'lcr_pairs_map.ply', map_xyz[map_ids], map_rgb[map_ids])
        distances = np.linalg.norm(xyz[ref_ids]-map_xyz[map_ids], axis=1)
        color_errors = np.linalg.norm(rgb[ref_ids]-map_rgb[map_ids], axis=1)
        np.savez_compressed(self.root/'lcr_pairs.npz', reference_indices=ref_ids, map_indices=map_ids,
                            spatial_distance_m=distances, rgb_l2=color_errors)
        line_count = min(2000, len(ref_ids))
        line_ids = sample_indices(len(ref_ids), line_count, self.rng)
        if line_count:
            lines = o3d.geometry.LineSet()
            lines.points = o3d.utility.Vector3dVector(np.r_[xyz[ref_ids[line_ids]], map_xyz[map_ids[line_ids]]])
            lines.lines = o3d.utility.Vector2iVector(np.c_[np.arange(line_count), np.arange(line_count)+line_count])
            lines.colors = o3d.utility.Vector3dVector(np.tile([.1, .7, .35], (line_count, 1)))
            if not o3d.io.write_line_set(str(self.root/'lcr_pair_lines.ply'), lines):
                raise OSError('Failed to write LCR line set')
        fig, axs = self.plt.subplots(1, 2, figsize=(14, 6))
        self.scatter(axs[0], xyz[ids], palette[category], f'Local Color Recall = {report["LCR_percent"]:.2f}%')
        from matplotlib.patches import Patch
        axs[0].legend(handles=[Patch(color=palette[i], label=n) for i, n in
                                enumerate(['No geometric neighbor', 'Neighbor present, color fails', 'Recalled'])],
                       loc='upper right', fontsize=8)
        counts = [report['no_neighbor_reference_points'], report['color_miss_reference_points'], report['matched_reference_points']]
        axs[1].bar(['No neighbor', 'Color miss', 'Recalled'], np.array(counts)/len(xyz)*100, color=palette)
        axs[1].set(ylabel='Percent of all reference points', title=f'Radius {report["radius_m"]} m; RGB L2 <= {report["rgb_l2_threshold"]:.3f}')
        self.finish_figure(fig, 'recall.png')
        self.manifest['lcr'] = {'status': {'0': 'no geometric neighbor', '1': 'color miss', '2': 'recalled'},
            'pair_rule': 'nearest spatial eligible map point; ties by map input index; many-to-one allowed',
            'pair_rows_correspond': True, 'exported_pairs': len(ref_ids), 'passing_reference_points': len(all_pairs),
            'line_count': line_count, 'index_space': 'point order after explicit transform/crop preprocessing'}

    def consistency(self, data, report):
        ids = sample_indices(len(data['counts']), self.limit, self.rng)
        xyz, values, counts = data['centers'][ids], data['traces'][ids], data['counts'][ids]
        fig, axs = self.plt.subplots(1, 2, figsize=(14, 6))
        colors = self.scalar(axs[0], xyz, values, f'CCS = {report["CCS"]:.6f}', self.ccs_max, 'magma')
        # A singleton has no defined sample covariance; distinguish it visually.
        colors[counts == 1] = [.5, .5, .5]
        axs[0].clear()
        self.scatter(axs[0], xyz, colors, f'CCS = {report["CCS"]:.6f}; singletons in gray')
        write_cloud(self.root/'ccs_voxels.ply', xyz, colors)
        np.savez_compressed(self.root/'ccs_voxels.npz', voxel_indices=ids, centers=xyz,
                            covariance_trace=values, point_counts=counts, evaluated=data['evaluated'][ids])
        evaluated = data['traces'][data['evaluated']]
        bins, edges = np.histogram(evaluated, bins=100, range=(0, 1.5))
        axs[1].stairs(bins/len(evaluated), edges)
        axs[1].set(xlabel='Sample RGB covariance trace', ylabel='Fraction of evaluated voxels',
                   title=f'Voxel {report["voxel_size_m"]} m; singleton policy: {report["singleton_policy"]}')
        axs[1].set_yscale('log')
        self.finish_figure(fig, 'color_consistency.png')
        self.supported_consistency(data)
        self.manifest['files']['ccs_voxels.ply'] = {'points': len(ids), 'population': len(data['counts']),
            'gray': 'singleton voxel', 'above_display_max': int(np.count_nonzero(data['traces'] > self.ccs_max))}

    def supported_consistency(self, data):
        """Separate supported voxels so singleton gray does not hide color variance."""
        population = np.flatnonzero(data['counts'] >= 2)
        ids = population[sample_indices(len(population), self.limit, np.random.default_rng(self.seed))]
        xyz, values = data['centers'][ids], data['traces'][ids]
        fig, ax = self.plt.subplots(figsize=(9, 7))
        colors = self.scalar(ax, xyz, values, 'CCS diagnostic: voxels containing at least two points',
                             self.ccs_max, 'magma')
        ax.text(.01, .01, f'{len(population):,} supported voxels; display only, score unchanged',
                transform=ax.transAxes, fontsize=8, bbox={'facecolor': 'white', 'alpha': .8})
        self.finish_figure(fig, 'color_consistency_supported.png')
        write_cloud(self.root/'ccs_supported_voxels.ply', xyz, colors)
        np.savez_compressed(self.root/'ccs_supported_voxels.npz', voxel_indices=ids, centers=xyz,
                            covariance_trace=values, point_counts=data['counts'][ids])
        self.manifest['files']['ccs_supported_voxels.ply'] = {
            'points': len(ids), 'population': len(population),
            'selection': 'voxels with at least two points; display only, scoring policy unchanged'}

    def finish(self, report):
        self.manifest['metrics'] = report['metrics']
        write_report(self.root/'manifest.json', self.manifest)
        write_report(self.root/'metrics.json', report)
        (self.root/'README.md').write_text('''# Color-map diagnostics

Open the PLY files in CloudCompare or Open3D. PNGs render top-down XY views.
Sampling limits display/export size only; metrics use all supplied points.
Scalar colors saturate at the limits recorded in `manifest.json`.

- `color_distance.png`, `cd_*.ply`: both directions of spatial nearest-neighbor RGB L2 error.
  NPZ files contain the displayed input indices and unclipped scalar errors.
- `recall.png`, `lcr_reference_status.ply`: green = recalled, red = geometric neighbor
  exists but no eligible color, gray = no geometric neighbor. NPZ status codes are 2/1/0.
- `lcr_reference_recalled.ply`, `lcr_reference_color_miss.ply`,
  `lcr_reference_no_neighbor.ply`: separately sampled categories in original RGB.
- `lcr_pairs_reference.ply` and `lcr_pairs_map.ply`: matching row numbers form
  eligible pairs, in original RGB. Map points may repeat. `lcr_pairs.npz` stores
  both input indices, spatial distances and RGB errors. `lcr_pair_lines.ply`
  displays at most 2,000 of these pairs (absent when there are no matches).
- `color_consistency.png`, `ccs_voxels.ply`: voxel-center sample color covariance
  traces. Gray marks undefined singleton covariance under the recorded policy.
  `ccs_voxels.npz` preserves trace, occupancy, center and evaluated flag.
  `color_consistency_supported.png` and `ccs_supported_voxels.ply` show only
  voxels with at least two points so gray singletons cannot hide the errors;
  this separate diagnostic does not change the reported CCS.

Empty categories have valid zero-vertex PLY files. Input indices refer to point
order after any explicitly recorded rigid transform or ROI crop. Visualization
subsamples are not intended as inputs for recomputing the reported metrics.
''', encoding='utf-8')
        return {'directory': str(self.root), **self.manifest}
