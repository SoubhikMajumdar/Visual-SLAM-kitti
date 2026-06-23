# Stereo Visual Odometry on KITTI

A stereo visual odometry (VO) pipeline built from scratch — ORB feature extraction, stereo triangulation, temporal PnP-RANSAC motion estimation, and trajectory evaluation against real KITTI odometry ground truth. Built and run in Google Colab. No external SLAM/VO libraries (no ORB-SLAM, no OpenVSLAM) — every stage of the geometry pipeline is implemented directly with OpenCV primitives and NumPy.

## Repository structure (what's pushed to GitHub)

```text
realtime_stereo_slam/
└── scripts/
    └── <notebook>.ipynb        # full pipeline: dataset setup, VO algorithm, evaluation, benchmarking
```

Only the notebook/script is version-controlled. Everything else described below (datasets, generated results, videos) is either pulled from Google Drive at runtime or regenerated fresh each time the notebook is run in Colab — none of it is checked into this repo, since KITTI image data is large and results are reproducible by re-running the notebook.

## What gets created at runtime (Colab session only, not in this repo)

When the notebook runs, it builds this structure on the Colab VM's local disk (wiped on session end):

```text
/content/
├── kitti_raw/                          # unzipped KITTI data, pulled from Drive
│   └── dataset/
│       ├── sequences/
│       │   ├── 00/
│       │   │   ├── image_0/            # left camera grayscale frames (.png)
│       │   │   ├── image_1/            # right camera grayscale frames (.png)
│       │   │   └── calib.txt           # stereo calibration (P0–P3 projection matrices)
│       │   ├── 02/, 05/, 08/, 09/, ... # additional sequences (00–21 available)
│       └── poses/
│           ├── 00.txt                  # ground-truth poses, sequences 00–10 only
│           ├── 02.txt
│           ├── 05.txt
│           ├── 08.txt
│           └── 09.txt
│
└── realtime_stereo_slam/
    ├── configs/
    │   └── kitti.yaml                  # dataset path, ORB params, matching/stereo thresholds
    └── results/
        ├── trajectories/
        │   ├── pred_xyz.txt             # raw estimated trajectory (unaligned)
        │   ├── pred_aligned_xyz.txt     # estimated trajectory after Umeyama alignment to GT
        │   └── gt_xyz.txt               # ground-truth trajectory (same frame count)
        ├── plots/
        │   ├── trajectory_vs_gt.png     # estimated vs ground-truth path (overhead x-z view)
        │   ├── fps_over_time.png        # per-frame FPS across the run
        │   ├── latency_breakdown.png    # mean latency by pipeline stage
        │   └── drift_over_time_seq00_full.png  # per-frame ATE deviation, full seq 00 run
        ├── runtime/
        │   ├── frame_runtime.csv        # per-frame timing + feature/match counts
        │   └── stage_runtime_summary.csv # per-stage mean/median/p95 latency
        ├── videos/
        │   ├── feature_demo.mp4         # ORB keypoint visualization (mp4v codec)
        │   └── feature_demo_h264.mp4    # re-encoded H.264 version for inline Colab playback
        ├── metrics.csv                  # single-run summary metrics
        └── benchmark_multi_sequence.csv # FPS/ATE comparison across multiple sequences
```

**Why none of this is in the repo:** the KITTI dataset itself is several GB and isn't ours to redistribute (requires registration on the [official KITTI site](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)), and the CSVs/plots/videos are fully reproducible by re-running the notebook against the same data — so they're treated as build artifacts, not source.

## Where the data actually lives (Google Drive setup)

KITTI doesn't allow direct `wget`/`curl` downloads (requires a logged-in browser session), so the data was downloaded manually once and staged in Google Drive for Colab to pull from each session:

```text
Google Drive: MyDrive/KITTI/
├── data_odometry_gray.zip      # grayscale stereo images, all sequences (image_0, image_1)
└── data_odometry_poses.zip     # ground-truth poses for sequences 00–10
```

At the start of each Colab session, the notebook:
1. Mounts Drive (`drive.mount('/content/drive')`)
2. Scans `MyDrive/KITTI/` for any `.zip` files
3. Unzips each one into `/content/kitti_raw` on the Colab VM's local disk
4. Auto-detects the dataset root by searching for a `sequences/` folder (handles minor nesting differences in how KITTI zips extract)

This means the only manual setup required to re-run this project is: download `data_odometry_gray.zip` and `data_odometry_poses.zip` from the official KITTI site, drop them in a Drive folder, and point `DRIVE_KITTI_DIR` in the notebook at that folder.

Note: ground-truth poses are only published for sequences **00–10** (training split). Sequences **11–21** are KITTI's held-out test set — `data_odometry_poses.zip` doesn't contain poses for these, so any run on those sequences will skip ATE evaluation automatically and only report estimated trajectories / runtime metrics.

## How the code works

### Pipeline overview

Each frame goes through four stages:

1. **Feature extraction** — ORB keypoints/descriptors computed independently on the left and right images
2. **Stereo triangulation** — left↔right ORB matches converted to 3D points via calibrated stereo depth
3. **Temporal motion estimation** — current-frame features matched against the previous frame's 3D points, solved as a PnP problem with RANSAC
4. **Pose chaining** — each frame-to-frame motion estimate is composed onto a running world-frame trajectory

No loop closure, no bundle adjustment, no keyframing — this is a from-scratch VO baseline, not a full SLAM system, and the results below are interpreted with that in mind.

### Stereo depth (triangulation)

In a calibrated, rectified stereo rig, a 3D point at depth `Z` projects to left/right images with a horizontal pixel offset (disparity) `d = u_left - u_right`. Depth is recovered as:

```
Z = (fx * baseline) / disparity
```

then back-projected to full 3D `(X, Y, Z)` with the pinhole camera model. Matches are filtered on vertical disagreement (`|v_left - v_right| > 2px` rejected — true rectified stereo matches should be nearly horizontal-only), and on implausible disparity/depth ranges, before being accepted.

### Feature matching

ORB descriptors are matched via brute-force Hamming distance with Lowe's ratio test: each point's best match must be meaningfully closer than its second-best match, or it's discarded as ambiguous. The same matching function is reused for both left↔right (stereo) and previous-frame↔current-frame (temporal) matching.

### Motion estimation (PnP-RANSAC)

Once 3D points are known from the previous frame's stereo triangulation, and those same features are matched into the current frame's 2D image, the frame-to-frame camera motion is solved as a Perspective-n-Point problem via `cv2.solvePnPRansac` — RANSAC makes this robust to mismatched features or moving objects in the scene, since outlier correspondences get excluded from the final pose estimate.

### Pose chaining

The relative motion estimate from PnP is inverted (it maps the *previous* frame into the *current* frame; inverting gives "how did the camera itself move") and composed onto a running world-frame pose via matrix multiplication. This frame-to-frame chaining, with no global correction step, is the direct mechanism behind the long-horizon drift shown in the full-sequence results below — small per-frame errors don't cancel, they accumulate.

### Evaluation

Estimated and ground-truth trajectories are aligned using the Umeyama method (optimal least-squares scale + rotation + translation between two point sets, solved via SVD), then compared with Absolute Trajectory Error (ATE) RMSE — the root-mean-square positional distance between the aligned estimate and ground truth.

## Results

### Short-segment accuracy (200 frames per sequence, identical config)

| Sequence | FPS | ATE RMSE (m) | Distance (m) | ATE % of distance |
|---|---:|---:|---:|---:|
| 00 | 56.3 | 0.192 | 144.9 | 0.13% |
| 02 | 56.1 | 0.755 | 230.5 | 0.33% |
| 05 | 58.9 | 0.157 | 153.7 | 0.10% |
| 08 | 54.5 | 1.437 | 168.3 | 0.85% |
| 09 | 64.4 | 0.527 | 190.3 | 0.28% |

Tracking success rate was ≥99.5% on every sequence. Accuracy differences across sequences (e.g. 08's 0.85% vs. 05's 0.10%) reflect real differences in driving environment difficulty — turn density, dynamic objects, low-texture regions — rather than pipeline instability.

### Full-sequence drift accumulation (sequence 00, all 4541 frames)

| Metric | Value |
|---|---:|
| Frames | 4541 |
| Distance traveled | 3724.19 m |
| ATE RMSE | 48.98 m |
| ATE % of distance | 1.32% |
| Tracking success rate | 99.98% |
| Overall FPS | 59.4 |

Sequence 00 contains a real loop closure — the recorded route revisits ground it already covered. This full run is the clearest demonstration of why pure VO needs loop closure: the estimated trajectory tracks the overall route shape, but small per-frame error compounds enough that the loop never actually closes (visible directly in `trajectory_vs_gt.png` — the estimated path drifts into territory the ground-truth path never visits, and ends up clearly offset from where it should reconnect).

The per-frame drift curve (`drift_over_time_seq00_full.png`) oscillates rather than increasing strictly monotonically. This is an artifact of how ATE-with-Umeyama-alignment is computed: alignment finds one *fixed* global transform that best fits the whole trajectory, so different stretches of the route naturally land closer to or farther from that single rigid+scale fit — it does not mean the estimator periodically "recovers." Raw per-frame relative error accumulates fairly steadily underneath this.

### Runtime profiling

| Stage | Mean latency (ms) |
|---|---:|
| Feature extraction (ORB) | 6.30 |
| Image load / preprocess | 5.97 |
| Pose estimation (PnP-RANSAC) | 2.34 |
| Stereo triangulation | 2.13 |

Feature extraction and image I/O dominate per-frame cost, not the motion-estimation math — the pipeline runs at roughly 55–65 FPS on CPU across all tested sequences.

### Key diagnostic: alignment scale

Recovered Umeyama alignment scale was consistently within ~0.99–1.03 of 1.0 across every sequence tested. This is expected and a useful sanity check specifically for *stereo* VO: absolute metric scale comes directly from the known camera baseline used in triangulation, unlike monocular VO, which has fundamental scale ambiguity and would show much larger, less stable scale-alignment values if it drifted.

## Possible extensions

- **Loop closure** — place recognition (e.g. bag-of-words on ORB descriptors) to detect revisited locations and trigger drift correction; directly motivated by the full sequence-00 result above
- **Bundle adjustment** — windowed optimization over several keyframes instead of strict frame-to-frame chaining, to reduce per-step error before it compounds
- **Relative Pose Error (RPE)** — complements ATE by isolating local frame-to-frame drift rate from the global-alignment artifacts visible in the oscillating drift-over-time plot
- **Keyframing** — skip low-motion frames to reduce redundant compute and improve triangulation baseline quality

## Reproducing this

1. Register at the [KITTI odometry benchmark page](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) and download `data_odometry_gray.zip` and `data_odometry_poses.zip`.
2. Upload both zips to a Google Drive folder.
3. Open `scripts/<notebook>.ipynb` in Colab, set `DRIVE_KITTI_DIR` to that folder's path, and run all cells.
4. Adjust `config["dataset"]["sequence"]` and `config["dataset"]["max_frames"]` to target different sequences or run lengths.
