# Real-Time Stereo Visual SLAM / Visual Odometry

This project implements a deployable stereo visual odometry pipeline with KITTI-format dataset loading, calibration parsing, stereo triangulation, PnP-RANSAC motion estimation, trajectory evaluation, and runtime benchmarking, run against real KITTI odometry sequence 00.

## Features

- KITTI-format stereo dataset support
- ORB feature extraction
- Stereo matching and depth triangulation
- Temporal feature tracking
- PnP-RANSAC camera motion estimation
- ATE RMSE trajectory evaluation (if ground truth available)
- FPS, median latency, p95 latency, and stage-wise runtime profiling
- Trajectory visualization
- Feature demo video generation

## Results

| Metric | Value |
|---|---:|
| Sequence | 00 |
| Frames | 200 |
| Overall FPS | 49.58 |
| Mean Frame Latency | 20.11 ms |
| Median Frame Latency | 17.61 ms |
| p95 Frame Latency | 18.72 ms |
| Tracking Success Rate | 99.50% |
| ATE RMSE | 0.192 m |

## Runtime Profiling

```text
results/runtime/stage_runtime_summary.csv
results/runtime/frame_runtime.csv
```

## Outputs

- `results/plots/trajectory_vs_gt.png` (or `trajectory_estimated_only.png` if no ground truth)
- `results/plots/fps_over_time.png`
- `results/plots/latency_breakdown.png`
- `results/videos/feature_demo.mp4`
