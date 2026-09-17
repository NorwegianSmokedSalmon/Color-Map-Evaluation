# Color-Map-Evaluation

独立的彩色点云地图评估工具，依据 [LiDAR-VGGT](https://arxiv.org/abs/2511.01186)
§III-D 公式 (16)–(18)，计算 CD、CF、LCR 和 CCS。

| 指标 | 含义 | 方向 |
| --- | --- | --- |
| CD | 空间最近邻对应点的 RGB 欧氏距离，两个方向的均值各占一半 | 越小越好 |
| CF | `−20 log10(CD)`，单位 dB | 越大越好 |
| LCR | 参考点在重建地图半径内存在颜色差 `≤3τ` 的邻点，其占参考点总数的比例 | 越大越好 |
| CCS | 各体素 RGB 样本协方差的迹，再对体素等权平均 | 越小越好 |

支持彩色 PCD / PLY，使用 CPU。CD / CF / LCR 需要已对齐、坐标单位为米的参考地图；
CCS 只需重建地图。RGB 通道统一为 Open3D 的 `[0,1]` 约定。

## 安装与运行

在本仓库根目录，使用 Python 3.10–3.12：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install .

color-map-evaluate --map reconstruction.ply --reference reference.ply \
  --tau 0.1 --radius 0.5 --ccs-voxel-size 0.1 --output results/metrics.json
```

一条命令输出四项指标和参数记录。只计算 CCS 时：

```bash
color-map-evaluate --map reconstruction.ply --metrics ccs --output results/ccs.json
```

也支持 `python -m color_map_evaluation`。三个独立命令为 `color-map-cd`、
`color-map-lcr`、`color-map-ccs`，旧脚本 `cal_colordist.py`、`cal_lccr.py`、
`cal_cis.py` 保留为兼容入口。可选绘图依赖用 `python -m pip install '.[plot]'` 安装。

无需下载数据即可验证：

```bash
python examples/make_toy_clouds.py
color-map-evaluate --map examples/data/reconstruction.ply \
  --reference examples/data/reference.ply --output results/toy.json
```

合成示例默认 LCR 为 `0.8`、CCS 为 `0.02`。

## 评估约定

LCR 默认 `τ=0.1`、半径 `0.5m` 来自论文 §IV-B / Fig. 7。
CCS 的体素尺寸、原点、单点体素处理和颜色量纲等未由论文完整给出，当前工程约定为：

- CCS 体素 `0.1m`，原点 `(0,0,0)`，分别通过 `--ccs-voxel-size` 和 `--voxel-origin X Y Z` 配置。
- 默认 `--singleton-policy zero` 将单点体素的迹记为 0 并计入分母；也可 `exclude` 排除或 `error` 报错。
- 评估默认不做配准、裁剪或降采样；CLI 仅在显式配置时应用刚体变换或公共 ROI 裁剪。CCS 应使用尚未体素颜色平均的地图。
- JSON 记录点数、方向误差、LCR 参数、CCS 体素统计和所用约定；CF 的正无穷以字符串 `"+inf"` 保存。

完整定义、API 和开发命令见 [英文 README](README.md) 与 [指标说明](docs/metrics.md)。

## 三类指标的可视化与配置

```bash
color-map-evaluate --map reconstruction.ply --reference reference.ply \
  --visualization-dir results/run/visualizations --output results/run/metrics.json
# 或从 JSON 配置启动，命令行参数可以覆盖配置：
color-map-evaluate --config examples/evaluation.json
```

导出 CD 双向误差着色点云、LCR 成功/颜色失败/无几何邻居点云、LCR 配对点云及连线、
CCS 体素颜色方差图。`lcr_pairs_reference.ply` 与 `lcr_pairs_map.ply` 相同行号对应；
`lcr_pairs.npz` 保存原始索引、空间距离和颜色误差。PNG/PLY 默认最多显示 20 万个点，
指标使用全部输入点。完整参数和文件说明见 [英文 README](README.md#visualization)。

坐标系不一致时，先用 `color-map-align` 对独立 LiDAR 几何进行刚体对齐，再通过
`--map-transform` 应用同一个变换。可通过 `--roi` 指定固定公共裁剪范围。
两项预处理均显式记录；不要使用按各自重建结果自适应裁剪的真值来比较召回率。
