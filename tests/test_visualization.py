import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import open3d as o3d

from color_map_evaluation import compute_ccs, compute_lcr, evaluate_map
from color_map_evaluation.preprocessing import load_transform


def cloud(points, colors):
    result = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points, float)))
    result.colors = o3d.utility.Vector3dVector(np.asarray(colors, float))
    return result


class VisualizationTest(unittest.TestCase):
    def test_lcr_witnesses_are_eligible_and_failures_are_separate(self):
        source = cloud([[0, 0, 0], [.2, 0, 0], [5, 0, 0]],
                       [[1, 1, 1], [0, 0, 0], [1, 1, 1]])
        target = cloud([[0, 0, 0], [5, 0, 0], [10, 0, 0]], [[0, 0, 0]]*3)
        report, data = compute_lcr(source, target, return_data=True, thread_num=1)
        np.testing.assert_array_equal(data['map_indices'], [1, -1, -1])
        np.testing.assert_array_equal(data['geometric'], [True, True, False])
        self.assertEqual(report['LCR'], 1/3)
        self.assertEqual(report['color_miss_reference_points'], 1)
        self.assertEqual(report['no_neighbor_reference_points'], 1)

    def test_random_lcr_against_dense_bruteforce_and_ties(self):
        rng = np.random.default_rng(12)
        x, y = rng.random((35, 3)), rng.random((29, 3))
        a, b = rng.random((35, 3)), rng.random((29, 3))
        source, target = cloud(x, a), cloud(y, b)
        distance = np.linalg.norm(y[:, None]-x[None], axis=2)
        error = np.linalg.norm(b[:, None]-a[None], axis=2)
        valid = (distance <= .5) & (error <= .3)
        for batch in [1, 8, 100]:
            report, data = compute_lcr(source, target, batch_size=batch, thread_num=1, return_data=True)
            np.testing.assert_array_equal(data['hits'], valid.any(axis=1))
            for i in np.flatnonzero(data['hits']):
                eligible = np.flatnonzero(valid[i])
                self.assertEqual(data['map_indices'][i], eligible[np.argmin(distance[i, eligible])])
        source = cloud([[.1, 0, 0], [-.1, 0, 0]], [[0, 0, 0]]*2)
        _, data = compute_lcr(source, cloud([[0, 0, 0]], [[0, 0, 0]]), return_data=True)
        self.assertEqual(data['map_indices'][0], 0)

    def test_ccs_voxels_preserve_formula_and_singletons(self):
        source = cloud([[-.2, 0, 0], [-.1, 0, 0], [2.1, 0, 0]],
                       [[0, 0, 0], [1, 1, 1], [0, 0, 0]])
        report, data = compute_ccs(source, 1, singleton_policy='exclude', return_data=True)
        self.assertEqual(report['CCS'], 1.5)
        np.testing.assert_allclose(data['traces'], [1.5, 0])
        np.testing.assert_allclose(data['centers'], [[-.5, .5, .5], [2.5, .5, .5]])
        np.testing.assert_array_equal(data['evaluated'], [True, False])

    def test_visualization_is_metric_neutral_and_pairs_roundtrip(self):
        source = cloud([[.01, 0, 0], [.02, 0, 0], [5, 0, 0]],
                       [[0, 0, 0], [.2, 0, 0], [1, 1, 1]])
        target = cloud([[0, 0, 0], [5, 0, 0], [10, 0, 0]], [[0, 0, 0]]*3)
        expected = evaluate_map(source, target, threads=1)['metrics']
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)
            actual = evaluate_map(source, target, threads=1, visualization_dir=tmp,
                                  visualization_max_points=2)
            self.assertEqual(expected, actual['metrics'])
            for name in ['color_distance.png', 'recall.png', 'color_consistency.png', 'color_consistency_supported.png', 'manifest.json']:
                self.assertTrue((path/name).stat().st_size > 0)
            with np.load(path/'lcr_pairs.npz') as archive:
                pairs = {key: archive[key] for key in archive.files}
            src = o3d.io.read_point_cloud(str(path/'lcr_pairs_map.ply'))
            ref = o3d.io.read_point_cloud(str(path/'lcr_pairs_reference.ply'))
            np.testing.assert_allclose(np.asarray(src.points), np.asarray(source.points)[pairs['map_indices']])
            np.testing.assert_allclose(np.asarray(ref.points), np.asarray(target.points)[pairs['reference_indices']])
            self.assertTrue(np.all(pairs['spatial_distance_m'] <= .5))
            self.assertTrue(np.all(pairs['rgb_l2'] <= .3))
            with self.assertRaisesRegex(ValueError, 'completed'):
                evaluate_map(source, target, visualization_dir=tmp)

    def test_zero_recall_and_ccs_only(self):
        source = cloud([[0, 0, 0]], [[0, 0, 0]])
        target = cloud([[10, 0, 0]], [[1, 1, 1]])
        with tempfile.TemporaryDirectory() as tmp:
            report = evaluate_map(source, target, metrics=['lcr'], visualization_dir=tmp)
            self.assertEqual(report['metrics']['LCR'], 0)
            self.assertIn('element vertex 0', (Path(tmp)/'lcr_pairs_map.ply').read_text())
        with tempfile.TemporaryDirectory() as tmp:
            report = evaluate_map(source, metrics=['ccs'], visualization_dir=tmp)
            self.assertEqual(report['metrics']['CCS'], 0)

    def test_config_paths_transform_roi_and_cli_override(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            source = cloud([[.01, 0, 0], [.02, 0, 0], [10, 0, 0]], [[0, 0, 0], [.2, 0, 0], [1, 1, 1]])
            o3d.io.write_point_cloud(str(root/'map.ply'), source)
            transform = np.eye(4); transform[0, 3] = 2
            np.savetxt(root/'transform.txt', transform)
            config = {'map': 'map.ply', 'metrics': ['ccs'], 'map_transform': 'transform.txt',
                      'roi': [2, -.1, -.1, 3, .1, .1], 'ccs_voxel_size': 1,
                      'output': 'metrics.json', 'threads': 1}
            (root/'config.json').write_text(json.dumps(config))
            result = subprocess.run([sys.executable, '-I', '-m', 'color_map_evaluation', '--config',
                                     str(root/'config.json'), '--ccs-voxel-size', '.1'], cwd='/',
                                    capture_output=True, text=True, check=True)
            report = json.loads(result.stdout)
            self.assertEqual(report['map_points'], 2)
            self.assertEqual(report['preprocessing']['map_input_points'], 3)
            self.assertEqual(report['details']['ccs']['voxel_size_m'], .1)
            self.assertAlmostEqual(report['metrics']['CCS'], .02)
            transform[0, 0] = 2
            np.savetxt(root/'transform.txt', transform)
            with self.assertRaisesRegex(ValueError, 'rigid'):
                load_transform(root/'transform.txt')


if __name__ == '__main__':
    unittest.main()
