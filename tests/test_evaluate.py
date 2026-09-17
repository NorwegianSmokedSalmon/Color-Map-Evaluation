import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import open3d as o3d

from color_map_evaluation import evaluate_map
from color_map_evaluation._common import format_report

ROOT = Path(__file__).resolve().parents[1]


def make_cloud(points, colors):
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.asarray(points, dtype=float)))
    cloud.colors = o3d.utility.Vector3dVector(np.asarray(colors, dtype=float))
    return cloud


class EvaluateMapTest(unittest.TestCase):
    def test_combined_metrics_against_analytic_example(self):
        source = make_cloud([[.01, 0, 0], [.02, 0, 0]], [[0, 0, 0], [.2, 0, 0]])
        reference = make_cloud([[.01, 0, 0], [.02, 0, 0], [10, 0, 0]],
                               [[0, 0, 0], [.2, 0, 0], [0, 0, 1]])
        report = evaluate_map(source, reference, threads=1)
        values = report['metrics']
        cd = np.sqrt(1.04) / 6
        self.assertAlmostEqual(values['CD'], cd)
        self.assertAlmostEqual(values['CF_dB'], -20*np.log10(cd))
        self.assertAlmostEqual(values['LCR'], 2/3)
        self.assertAlmostEqual(values['CCS'], .02)
        self.assertEqual(report['map_points'], 2)
        self.assertEqual(report['reference_points'], 3)
        self.assertEqual(report['details']['lcr']['direction'], 'reference_to_reconstruction')
        self.assertEqual(report['details']['ccs']['singleton_voxels'], 0)

    def test_selection_and_reference_requirements(self):
        source = make_cloud([[0, 0, 0]], [[0, 0, 0]])
        report = evaluate_map(source, metrics='ccs')
        self.assertEqual(report['metrics'], {'CCS': 0.0})
        self.assertIsNone(report['reference_points'])
        fidelity = evaluate_map(source, source, metrics=['cf'])
        self.assertEqual(set(fidelity['metrics']), {'CF_dB'})
        self.assertEqual(json.loads(format_report(fidelity))['metrics']['CF_dB'], '+inf')
        with self.assertRaisesRegex(ValueError, 'reference'):
            evaluate_map(source)
        for invalid in ([], ['unknown']):
            with self.assertRaises(ValueError):
                evaluate_map(source, source, metrics=invalid)

    def test_toy_example_and_installed_cli_outside_checkout(self):
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            subprocess.run([sys.executable, str(ROOT/'examples/make_toy_clouds.py'),
                            '--output-dir', str(tmp)], check=True, capture_output=True, text=True)
            output = tmp/'nested/report.json'
            result = subprocess.run([
                sys.executable, '-I', '-m', 'color_map_evaluation',
                '--map', str(tmp/'reconstruction.ply'), '--reference', str(tmp/'reference.ply'),
                '--output', str(output), '--threads', '1',
            ], cwd=tmp, check=True, capture_output=True, text=True)
            report = json.loads(result.stdout)
            self.assertEqual(report, json.loads(output.read_text()))
            self.assertEqual(report['metrics']['LCR'], .8)
            self.assertAlmostEqual(report['metrics']['CCS'], .02)
            self.assertEqual(report['schema_version'], 1)
            self.assertEqual(report['package_version'], '0.1.0')
            missing = subprocess.run([
                sys.executable, '-I', '-m', 'color_map_evaluation',
                '--map', str(tmp/'reconstruction.ply'),
            ], cwd=tmp, capture_output=True, text=True)
            self.assertEqual(missing.returncode, 2)
            self.assertIn('--reference', missing.stderr)


if __name__ == '__main__':
    unittest.main()
