"""Generate two tiny colored maps to exercise the CLI without external data."""
import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("examples/data"))
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    points = np.array([[.01, 0, 0], [.02, 0, 0], [1.01, 0, 0], [1.02, 0, 0]])
    colors = np.array([[0, 0, 0], [.2, 0, 0], [0, .4, 0], [0, .6, 0]])
    for name, xyz, rgb in (
        ("reconstruction.ply", points, colors),
        ("reference.ply", np.vstack([points, [10, 0, 0]]), np.vstack([colors, [0, 0, 1]])),
    ):
        cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
        cloud.colors = o3d.utility.Vector3dVector(rgb)
        path = args.output_dir / name
        if not o3d.io.write_point_cloud(str(path), cloud, write_ascii=True):
            raise OSError(f"Failed to write {path}")
        print(path)


if __name__ == "__main__":
    main()
