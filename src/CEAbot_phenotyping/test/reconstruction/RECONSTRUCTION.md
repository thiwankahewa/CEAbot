# Comparing plant-view registration methods

For soil-referenced height measurements and historical soil fallback, see
[PLANT_HEIGHT.md](../height/PLANT_HEIGHT.md). That workflow reads the original RGB-D views
with these saved registration poses, so a cropped PLY does not limit soil coverage.

Run from `/home/thiwa/CEAbot`. The original command still reconstructs using saved
camera poses, with no registration and no Open3D dependency:

```bash
python3 src/CEAbot_phenotyping/test/reconstruction/reconstruct_plant_views.py \
  /home/thiwa/scan_data/b1_r12_20260902_173149
```

Registration additionally needs Open3D (tested with 0.19.0):

```bash
python3 -m pip install 'open3d>=0.19,<0.20'
```

On Linux, `open3d-cpu==0.19.0` is a smaller alternative that provides the same
`open3d` Python module. Use the Python environment where your dependencies are
installed. Limit OpenMP threads if registration is slow on a many-core machine,
for example by prefixing a command with `OMP_NUM_THREADS=4`.

## Enable methods

All switches default to off. Enabling several produces **independent results**;
each method starts from the original saved poses and original RGB values.

| Switch | Method |
| --- | --- |
| `--robust-icp` / `--no-robust-icp` | Multiscale point-to-plane ICP with Huber or Tukey loss |
| `--colored-icp` / `--no-colored-icp` | Multiscale colored ICP |

Compare both ICP methods plus the saved-pose baseline:

```bash
python3 src/CEAbot_phenotyping/test/reconstruction/reconstruct_plant_views.py \
  /home/thiwa/scan_data/b1_r12_20260902_173149 \
  --all-methods --run-name compare_01 --voxel-size 0.002
```

Compare just robust and colored ICP, for one plant, with debug view colors:

```bash
python3 src/CEAbot_phenotyping/test/reconstruction/reconstruct_plant_views.py \
  /home/thiwa/scan_data/b1_r12_20260902_173149 \
  --plant plant_02 --robust-icp --colored-icp \
  --icp-voxels 0.008 0.004 0.002 --icp-distances 0.03 0.015 0.008 \
  --icp-iterations 50 --robust-kernel huber --robust-scale 0.01 \
  --colored-geometric-weight 0.968 \
  --color-by-view --save-debug-views --run-name icp_tuning_01
```

`--all-methods --no-colored-icp` enables only robust ICP. `--no-baseline`
suppresses the saved-pose baseline. Individual method switches override
`--all-methods`, regardless of their order.

## Output organization

With any registration method enabled, outputs are placed in a unique run folder:

```text
<scan>/reconstruction/compare_01/
  run_config.json
  baseline/
    plant_02_merged_rgbd.ply
    plant_02_registration.json
  robust_icp/
    plant_02_merged_rgbd.ply
    plant_02_registration.json
    debug_views/plant_02/...ply  # with --save-debug-views
  colored_icp/...
```

Omit `--run-name` for an automatic UTC timestamp. An existing comparison run name
is refused, so a parameter experiment cannot overwrite an earlier run.
`--output-dir /path/to/results` replaces `<scan>/reconstruction`; when processing
multiple scans, each scan gets its own subdirectory below that root.

With all methods off and no `--run-name`, the existing output location remains
`<scan>/reconstruction/plant_XX_merged_rgbd.ply`. Giving `--run-name` also works for
a baseline-only comparison. `--cloud-source original` retains the `_merged.ply`
filename suffix and loads legacy per-view `cloud_xyzrgb.npy` files. The default
RGB-D input requires `color.png`, `depth.npy`, and camera intrinsics in each
view's `meta.yaml`.

Each plant's JSON report includes all parameters, skipped input views, accepted
and rejected pairs with reasons, pairwise metrics before registration and after
registration, metrics after pose-graph optimization, saved and refined poses,
correction sizes, connected components, and point counts. A method can retain
some or all saved poses when matching fails; check `components` and
`accepted_pairs` rather than assuming every saved PLY was fully registered.

## Parameters

Distances and voxel sizes are in **metres**; angular limits are in **degrees**.
Use `--help` to list every option.

| Parameters | Default | Purpose |
| --- | --- | --- |
| `--icp-voxels` | `0.008 0.004 0.002` | Coarse-to-fine registration downsampling |
| `--icp-distances` | `0.03 0.015 0.008` | Correspondence limits, same number of entries as voxels |
| `--icp-iterations` | `40` | Maximum iterations at each scale |
| `--normal-radius-factor`, `--normal-max-nn` | `2.5`, `30` | Normal neighborhood radius / voxel size and neighbor count |
| `--robust-kernel`, `--robust-scale` | `huber`, `0.01` | Robust ICP loss and residual scale |
| `--colored-geometric-weight` | `0.968` | Geometry weight; color weight is `1 - weight` |
| `--min-fitness`, `--min-correspondences` | `0.20`, `30` | Minimum overlap fraction and match count in both directions |
| `--max-rmse` | `0.008` | Maximum geometric inlier RMSE in either direction |
| `--max-correction-translation`, `--max-correction-rotation` | `0.05`, `12` | Pairwise and final correction limits; translation is measured at the plant center |
| `--registration-margin` | `0.03` | Additional crop padding during registration only |
| `--pairing`, `--max-pair-angle` | `adjacent`, `80` | Side-neighbor pairing and maximum circular angular gap |
| `--pose-graph` / `--no-pose-graph` | on | Jointly optimize accepted pair constraints |

For parameter comparisons, keep the evaluation threshold (`--icp-distances`'s
last value), output voxel size, crop and view selection fixed. Inlier RMSE only
describes matched points; assess it together with overlap, rejected pairs, and
the actual leaf/pot alignment.

## Registration behavior

Side neighbors are selected by angle, including the ring closure. Missing views
do not force registration across gaps larger than `--max-pair-angle`. Top-to-side
pairs are also attempted. `--pairing all` attempts every pair and ignores the
angular limit, while retaining the geometric acceptance checks.

Accepted pairs form a graph. A maximum-weight spanning tree initializes each
connected component, anchored at the first available side view's saved pose.
Pose-graph optimization uses additional edges as uncertain loop constraints.
With `--no-pose-graph`, the spanning-tree corrections are used directly. Isolated
views retain their saved poses. If a component's final correction exceeds the
limits, the entire component retains its saved poses and the reason is reported.

Final corrections are applied to full depth-filtered clouds before the final
plant crop and output downsampling. `--color-by-view` is applied only afterward,
so colored ICP always sees the original RGB data.

The existing RGB-D projection convention is preserved: depth must already be
registered to the color pixel grid, with suitable pinhole intrinsics. The code
does not rectify raw images or compensate for leaf motion. Registration adjusts
rigid camera poses only, with fixed scale.

