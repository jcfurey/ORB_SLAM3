# ORB-SLAM3 settings files

The nodes require an ORB-SLAM3 `settings_file` (`.yaml`) describing the camera
model, intrinsics, distortion, resolution/FPS, and — for the inertial modes —
the IMU noise densities and the camera↔IMU extrinsics (`Tbc`). These values are
**per-unit** (calibrate your own camera), so no ready-made file is shipped here.

Start from the upstream templates in this repo and edit the calibration:

| Camera / mode | Template to copy |
| --- | --- |
| RealSense D435i, RGB-D | `Examples/RGB-D/RealSense_D435i.yaml` |
| RealSense D435i, stereo-inertial | `Examples/Stereo-Inertial/RealSense_D435i.yaml` |
| RealSense D435i, mono-inertial | `Examples/Monocular-Inertial/RealSense_D435i.yaml` |
| EuRoC-style stereo-inertial | `Examples/Stereo-Inertial/EuRoC.yaml` |

Key fields to get right:

- `Camera.type`: `"PinHole"` or `"KannalaBrandt8"` (fisheye, e.g. OAK-D wide).
- `Camera1.fx/fy/cx/cy` and distortion `k1..k4` — from your calibration.
- `Camera.fps`, `Camera.width`, `Camera.height` — match the driver's stream.
- RGB-D `RGBD.DepthMapFactor` — 1000.0 for 16-bit millimetre depth
  (RealSense/OAK), 1.0 for 32-bit metre depth.
- Stereo `Stereo.b` / `Camera.bf` — baseline (m) and baseline×fx.
- Inertial `IMU.NoiseGyro/NoiseAcc/GyroWalk/AccWalk`, `IMU.Frequency`, and the
  `IMU.T_b_c1` 4×4 camera-from-IMU extrinsic.

Point a node at your file with `settings_file:=/abs/path/to/your.yaml`.
