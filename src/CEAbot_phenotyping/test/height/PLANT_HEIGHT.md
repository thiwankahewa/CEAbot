# Plant height from local soil to the highest organ

`measure_plant_height.py` measures the vertical distance from estimated soil at
the stem base to the highest **supported plant point**, including organs of any
color connected to detected plant tissue. It also exports P99 and P99.5 heights
as diagnostics; the primary result is the supported maximum. Ruler values are
used only for validation, never to set soil height or fit the estimator.

## Run the September 14 comparison

From `/home/thiwa/CEAbot`:

```bash
python3 src/CEAbot_phenotyping/test/height/measure_plant_height.py \
  /home/thiwa/scan_data \
  --date 2026-09-14 --locations b1_r16 b1_r17 b1_r18 b1_r19 \
  --config src/CEAbot_phenotyping/test/height/plant_height_config.example.json \
  --manual-csv src/CEAbot_phenotyping/test/height/manual_heights_20260914.csv \
  --manual-unit mm \
  --output-dir /home/thiwa/scan_data/plant_height_20260914_run2
```

Use a new output directory each run. The transcribed ruler values are:

| Row | Plant 1 | Plant 2 | Plant 3 |
| --- | --- | --- | --- |
| 16 | 42 | 40 | 45 |
| 17 | 26 | 44 | 35 |
| 18 | 42 | 40 | 40 |
| 19 | 43 | 36 | 43 |

**The original table did not specify units.** Its CSV `unit` cells remain blank.
The example explicitly assumes millimetres. Use `--manual-unit cm` if appropriate,
or fill each CSV unit with `mm` or `cm`. A per-row unit overrides the CLI default.
The ambiguous row-16 entry is transcribed as plant 1 = 42; review the table before
using the comparison as study evidence.

The script uses the newest matching saved `robust_icp` registration JSON by
lexicographic run path for each plant, records that exact path, and falls back to
the top view (or first available view if top is absent) with a warning when no
registration exists. This gives a visible height without fusing misaligned
leaves; verify that the highest organ is visible. Use
`--registration-run RUN_NAME` to require a particular run or `--registration none`
to measure using a single saved camera pose. Only the largest connected
registration component is fused, preferring one containing top if sizes tie.
Excluded views are recorded. `--unregistered-views all` enables diagnostic fusion
of unregistered views, which is flagged and excluded from reusable soil history.
It reads original RGB-D images and refined camera poses, **not the cropped merged
PLY**, so reconstruction crop settings do not remove the soil reference.

Dependencies are listed in [`../requirements-height.txt`](../requirements-height.txt); Open3D is not required:

```bash
python3 -m pip install -r src/CEAbot_phenotyping/test/requirements-height.txt
```

## Physical direction and the stem location

This robot's arm is mounted upside down in `fixed_structure.urdf.xacro`.
The default physical up vector is therefore `--up 0 0 -1` in `base_link`.
It is not the soil plane normal. For another setup, supply its calibrated
physical up vector; when using a reference transform, specify up in that
reference frame. Camera poses and depth must already have correct metric
calibration and aligned color/depth pixels.

Without an override, the saved scan target's X/Y is a **stem-position proxy**.
This is flagged in every measurement: a foliage centroid is not necessarily the
stem base. For study measurements, review and set `stem_xy_base_m` to the stem
location in the scan's `base_link` frame, especially as the plant grows. These
coordinates describe the vertical column through the stem on this robot; use
the calibrated target/reference convention if changing the robot orientation.

For a sloping soil surface, the script evaluates `h = a*x + b*y + c` at the
stem location. Height is `highest plant elevation - soil elevation at stem`.
It does not subtract the soil immediately under each leaf or measure
perpendicularly to the soil plane. PLY outputs use a measurement frame with
physical up as positive Z; each result saves the basis and reference transform.

## When current soil is sufficient

The initial automated substrate candidates are brown pixels in an 8–45 mm
annulus around the stem, excluding nearby plant-colored points. The algorithm:

1. Aggregates points into 2 mm horizontal cells using median XYZ per cell.
2. Fits a local plane with deterministic RANSAC and refines its inliers.
3. Requires at least 60 inlier cells, 55% inlier fraction, five of eight angular
   sectors, and a soil footprint enclosing the stem.
4. Requires soil within 25 mm of the stem, a hull area of at least 500 mm²,
   and a slope below 25 degrees. The vertical inlier tolerance is 4 mm.

One view can be sufficient if it sees enough surrounding soil. A narrow patch
on just one side is rejected because interpolation at the hidden stem would be
poorly constrained. All these settings are starting parameters, not validated
measurement accuracy limits. Plane residual roughness is reported separately;
it is **not** a confidence interval for height. A mound hidden right at the stem
can differ from the fitted nearby surface even when these checks pass.

When accepted, **current soil takes precedence** over historical soil. A material
disagreement with history is flagged for possible soil change or coordinate drift.

## Reusing soil across dates

**September 14, 2026 is the first image set of this experiment.** It is the
initial soil baseline; August scans belong outside this experiment. The supplied
configuration sets `experiment_start_date` to `2026-09-14`. This excludes earlier
raw scans and earlier observations in an imported history file. `--start-date`
can override this setting for another experiment.

As new images are added, retain the same physical plant/pot identities:

1. On September 14, save accepted visible-soil estimates as the initial baseline.
2. On each later date, use current soil when it is sufficiently visible and add
   those accepted direct observations to history.
3. If soil is insufficiently visible, use the robust daily mean of comparable
   prior observations. Initially that may be only September 14. A fallback
   estimate does not add a new soil observation.

Two ways to process the growing dataset:

- Remove `--date` from the command above to reprocess all scans from the experiment
  start in chronological order. Keep the configuration and use a new output
  directory each time. No `--history` argument is needed for a complete rerun.
- To process only a new date, use `--date YYYY-MM-DD --history
  /path/to/latest_run/soil_history.json` with the same configuration. Each output
  history contains the retained previous observations plus new accepted fits;
  pass that newest history file to the next run.

The shared-coordinate requirement below still applies. The current configuration
stores pot identities but leaves `reference_id` unset; historical fallback is
not enabled until that reference is established. New data alone cannot establish
whether the robot/pot coordinate reference stayed fixed.

Each run writes `soil_history.json`, containing only accepted, directly observed
soil fits. Supply it to a later run with `--history /path/to/soil_history.json`.
When processing several dates in one invocation, scans are processed
chronologically and the accepted earlier observations are immediately available.
`--date` restricts processing; it does not silently load older raw scans.

History is used only when both of the following are configured:

- `history_key`: the identity of the same physical plant/pot within the same
  planting cohort, independent of changing scan IDs.
- `reference_id`: a shared metric coordinate reference. Setting this asserts
  that coordinates are comparable. If the robot/pot moved, supply a measured
  rigid `base_to_reference` transform for each scan, derived from fixed scene
  features or calibration. Registration within one scan does not establish
  alignment across days.

**Do not average absolute `base_link` elevations across dates just because the
frame name matches.** In the available data, August rows 16–18 have four plant
IDs each, while September has three. These are not automatically treated as the
same plants. Change `history_key` after replanting, and reset it after soil work
that invalidates the old baseline. No history is reused by the default command.

Example configuration for two dates with independently verified shared coordinates:

`plant_height_config.example.json` provides editable settings and September pot
identities for all four rows. Its `reference_id` is deliberately null until the
common coordinate reference is established. Its experiment start date excludes
the August scans from both processing and imported history.

```json
{
  "settings": {
    "soil_outer_radius_m": 0.045,
    "support_radius_m": 0.003
  },
  "locations": {
    "b1_r16": {
      "reference_id": "bench1-row16-calibrated-reference",
      "plants": {
        "1": {"history_key": "september-cohort-row16-pot1"},
        "2": {"history_key": "september-cohort-row16-pot2"},
        "3": {"history_key": "september-cohort-row16-pot3"}
      }
    }
  },
  "scans": {
    "b1_r16_20260914_113851": {
      "base_to_reference": [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]],
      "plants": {
        "2": {"stem_xy_base_m": [0.001005, 0.086529], "roi_radius_m": 0.060}
      }
    }
  }
}
```

The stem coordinates above are the saved target, **not a measured stem annotation**;
replace them with reviewed coordinates. Identity transforms are examples, not
cross-date calibrations. Configuration precedence is defaults, location,
location plant, scan, scan plant. Use location-wide identities only for the
explicit date range of the same cohort, or assign identities per scan.

To establish a baseline, process the earlier visible-soil scans with this
configuration. Then process later scans with the same identities and
`--history` pointing to that baseline's history file. Setting identities only
on the later run cannot make earlier unlabelled records comparable: reprocess
the baseline with the correct configuration.

The historical estimate:

- Uses strictly earlier timestamps with matching plant identity, reference, and
  up direction; the stem position must agree within 30 mm in the common frame.
- Takes a median plane for repeated scans on each date, then a robust arithmetic
  mean across accepted dates. Each day gets equal weight.
- Rejects daily elevation outliers using 3 MAD with a 3 mm minimum tolerance and
  refuses retained daily levels spanning over 10 mm.
- Reports dates, observation IDs, between-day standard deviation, and spread.
  One prior day is allowed and explicitly reported; its between-day SD is unknown.
- Never stores a historical fallback as a fresh observation, never includes future
  data, and replaces repeated observations rather than counting reruns again.

If current soil and history are both insufficient, height stays blank with
`status=unmeasurable`. The script does not infer soil from a lowest leaf, invent
a default level, or derive it from the ruler measurement.

## Plant segmentation and noise removal

The default plant ROI has the detected plant radius plus 30 mm margin, at least
60 mm radius. It includes 150 mm below and 300 mm above the target in physical
vertical coordinates. Enlarge `roi_radius_m` per plant and
`roi_above_target_m` / `roi_below_target_m` as needed for larger plants; inspect
that the entire plant is present and neighbouring plants/pot rims are excluded.

Points above the local soil model by 6 mm are voxelized at 1 mm. An original
sample is retained per voxel, without surface smoothing. A point needs at least
three other spatially distinct voxel samples within 3 mm. Nearby supported points
form components; components with at least eight green/yellow seed voxels are
retained. This includes connected non-green organs such as white flowers, and
can retain separate leaves with their own plant seeds. It does **not guarantee**
recognition of a disconnected non-green organ. The color seed warning and
overlays make that limitation visible.

For plants/colors not handled by the default seeds, supply semantic masks:

```text
MASK_ROOT/SCAN_NAME/plant_02/top/plant.png
MASK_ROOT/SCAN_NAME/plant_02/top/soil.png
MASK_ROOT/SCAN_NAME/plant_02/view_1_0deg/plant.png
```

Pass `--mask-root MASK_ROOT`. These are full-resolution binary masks aligned to
the original RGB image (nonzero selects a pixel). A supplied plant mask replaces
the color seeds for that view and should include **all plant organs**. It seeds
3D components rather than acting as a strict cutout; inspect nearby structures.
A supplied soil mask replaces brown candidates before spatial/geometry checks.
An all-zero soil mask explicitly indicates no visible soil in that view. Missing
masks use automated candidates. Invalid-sized masks cause that view to be skipped
with a recorded reason.

Inspect the magenta top point against its original RGB view. Sparse real tips can
be removed by density filtering; depth-edge artifacts may survive if spatially
supported. P99/P99.5 and per-view seed maxima are diagnostic alternatives, not
automatic replacements for the highest organ. Per-view seed maxima use plant
seeds only, so they may omit connected non-green flowers. No parameter is tuned
against the provided ruler data by the script.

## Outputs and interpretation

- `heights.csv`: primary/percentile heights in mm, soil source, manual values,
  signed error (automatic minus manual), and warnings.
- `results.json`: complete measurements plus MAE, RMSE, bias and unmatched manual
  records. Results are estimates, not certified study measurements.
- `manual_comparison.png`: estimated versus manual height, when ruler values are supplied.
- `soil_history.json`: accepted directly observed soil, with identities/provenance.
- `run_config.json`: exact settings, arguments, and configuration.
- Per plant: `measurement.json`, `height_preview.png`, source-view overlays,
  `classified.ply`, `plant_cleaned.ply`, `soil_candidates.ply`, and `height_line.ply`
  when measurable. Brown marks soil candidates, green selected plant points,
  and magenta the detected top. Soil candidates include points RANSAC may reject.

`--no-artifacts` skips image/PLY outputs for fast batch checks. A failed plant is
recorded while remaining plants are processed; the process exits nonzero if any
plant failed. Insufficient evidence is an ordinary `unmeasurable` result.

Review stem position, soil surface, tip, crop boundaries, and alignment before
including an estimate in the study. The reported soil-fit roughness and
between-view spread do not capture all calibration or manual-measurement errors.
For validation, repeat ruler measurements using the same local-soil reference,
highest organ, and natural plant position.
