# Fusing ORB-SLAM3 with robot_localization (and similar EKF/UKF filters)

This guide makes the `orb_slam3` ROS 2 node a well-behaved **map-frame pose
source** for [`robot_localization`](https://github.com/cra-ros-pkg/robot_localization)
and any similar estimator (an EKF/UKF, `fuse`, etc.). It covers the REP-105 TF
contract, the message/covariance contract those filters require, the node
parameters that satisfy it, and a turnkey launch.

> TL;DR — run the node with `base_frame_id:=base_link publish_tf:=false
> publish_odom:=true covariance_mode:=g2o`, feed `~/odom` to a **map** EKF, and
> let that EKF publish `map → odom`. Ready-made:
> `ros2 launch orb_slam3 robot_localization.launch.py settings_file:=/abs/mono.yaml`.

## 1. The REP-105 frame contract

`robot_localization` runs the standard dual-EKF layout ([REP-105]):

```
map ──(map EKF: fuses global sources incl. ORB-SLAM3)──► odom ──(odom EKF: wheel/IMU)──► base_link ──(static)──► camera
```

The **single-parent rule** of a tf tree means exactly one node may publish the
transform *into* any given frame. The map EKF owns `map → odom`; the odom EKF
owns `odom → base_link`; your robot/URDF owns `base_link → camera`. Therefore a
SLAM source must **not** broadcast `map → camera` or `map → base_link` itself —
doing so double-parents that frame and breaks tf (this was finding **H7** in
[`INVESTIGATION.md`](INVESTIGATION.md)).

So when feeding `robot_localization`, set **`publish_tf:=false`**. The node then
only *publishes the measurement* (`~/odom`), and the EKF closes the tf tree.

## 2. The message contract a filter expects

`robot_localization` consumes the node's `~/odom` (`nav_msgs/Odometry`) as an
`odom0` input. For a global pose source it must be:

| Requirement | How the node satisfies it |
| --- | --- |
| `header.frame_id` = the world frame (`map`) | `world_frame_id` (default `map`) |
| `child_frame_id` = the **robot body** frame, not the camera | set `base_frame_id:=base_link` → the node looks up the static `camera→base` extrinsic from tf and republishes **base_link-in-map** |
| pose is the body pose in the world | composed as `T_map_base = T_map_cam · T_cam_base` |
| `pose.covariance` is **SPD**, finite, metric, consistent | see §3 |
| no bogus velocity | ORB-SLAM3 has no twist → `twist.covariance` diagonal is `1e6` (the filter ignores it); leave the twist rows `false` in the EKF config |

If you leave `base_frame_id` empty (the legacy default), the node publishes the
**camera** pose with `child_frame_id = camera_frame_id`; you would then have to
tell the EKF the camera is the body frame, which is rarely what you want.

## 3. The covariance contract (why the EKF rejects or diverges)

`robot_localization` Cholesky-factorizes `pose.covariance`; a singular,
non-positive-definite, or NaN matrix is silently rejected (or NaN-propagates).
The node now **guarantees a symmetric positive-definite, finite 6×6** on every
message (eigenvalue-floored by `covariance_spd_floor`), and:

- **`covariance_mode:=g2o`** publishes ORB-SLAM3's own SE3 marginal from
  motion-only BA, correctly transformed into the world `[x y z roll pitch yaw]`
  frame. This is the most informative choice and lets the EKF weight VO per-frame.
  - The marginal is a *conditional* (map-fixed) covariance, so it is mildly
    **optimistic**; raise `covariance_g2o_scale` (e.g. `2.0`) to de-weight VO if
    the EKF over-trusts it (finding **M5**).
  - A frame with no marginal (e.g. IMU-dominated) falls back to the quality
    diagonal, **smoothed** so it never jumps orders of magnitude (finding **M20**).
  - **Monocular** maps are scale-ambiguous, so the covariance is non-metric and
    drifts with scale — the node warns once. Prefer **stereo/RGBD**, or a
    monocular-**inertial** map after IMU initialization, for a metric global
    source (findings **M19**, and **M6**: after IMU init the node currently
    serves the smoothed quality diagonal rather than an inertial marginal).
- **`covariance_mode:=quality`** (default) scales a fixed diagonal
  (`pose_covariance_diagonal`) up as the inlier count drops — a safe, always-metric
  choice when you do not want to trust the raw marginal.

Tune the base numbers with `pose_covariance_diagonal` (best-case `[x y z roll
pitch yaw]` variances) and `covariance_inlier_ref` / `covariance_max_scale`.

## 4. Turnkey launch

```bash
ros2 launch orb_slam3 robot_localization.launch.py \
     settings_file:=/abs/mono.yaml image_topic:=/camera/image_raw \
     cam_x:=0.10 cam_z:=0.20 cam_roll:=-1.5708 cam_yaw:=-1.5708   # camera in base_link
```

This starts (monocular by default): the node in base-frame mode with
`publish_tf:=false` + `covariance_mode:=g2o`, a static `base_link → camera`
transform (set the mounting to your rig), and a `robot_localization` **map** EKF
(`ekf_node`, config `config/ekf_map.yaml`) that fuses `~/odom` and publishes
`map → odom`. For rgbd/stereo/stereo-inertial, copy the launch and swap the
executable + topics (the node params are identical).

The config is deliberately just the **global** EKF. A complete robot also runs an
**odom** EKF fusing wheel odometry and/or IMU to publish `odom → base_link`;
that half is platform-specific — see the `robot_localization` docs for a
`dual_ekf_navsat`-style example and drop the ORB-SLAM3 `odom0` block from
`ekf_map.yaml` into your map EKF.

## 5. Manual bring-up (any sensor node)

```bash
# 1. camera driver (publishes /camera/image_raw, ...)
# 2. static base_link -> camera (your mounting; child == camera_frame_id)
ros2 launch orb_slam3 base_to_camera.launch.py \
     parent_frame:=base_link child_frame:=camera x:=0.10 z:=0.20 roll:=-1.5708 yaw:=-1.5708
# 3. ORB-SLAM3 as a map-frame source (no map TF; base-frame odom + marginal covariance)
ros2 run orb_slam3 mono_node --ros-args \
     -p settings_file:=/abs/mono.yaml -p image_topic:=/camera/image_raw \
     -p base_frame_id:=base_link -p publish_tf:=false -p covariance_mode:=g2o
# 4. your map EKF fusing /orb_slam3_mono/odom, publishing map -> odom
ros2 run robot_localization ekf_node --ros-args \
     --params-file $(ros2 pkg prefix orb_slam3)/share/orb_slam3/config/ekf_map.yaml
```

### Parameters that matter here

| Parameter | For fusion set to | Meaning |
| --- | --- | --- |
| `base_frame_id` | `base_link` | republish pose/odom/TF in the robot body frame (via tf `camera→base`) |
| `publish_tf` | `false` | let the map EKF own `map → odom`; do not double-parent |
| `publish_odom` | `true` | emit `~/odom` (`nav_msgs/Odometry`) for the filter |
| `covariance_mode` | `g2o` (or `quality`) | source of `pose.covariance` (see §3) |
| `covariance_g2o_scale` | `1.0`–`3.0` | de-weight the optimistic marginal |
| `covariance_spd_floor` | `1e-9` | min eigenvalue keeping the covariance PD for Cholesky |

## 6. Troubleshooting

- **EKF ignores the pose / "covariance is not positive-definite".** Fixed by the
  SPD guarantee; make sure you are on this build. Confirm with
  `ros2 topic echo /orb_slam3_mono/odom --field pose.covariance`.
- **`base_frame_id` set but pose still in the camera frame.** The static
  `camera→base` transform is not in tf yet — start `base_to_camera.launch.py`
  (or your URDF `robot_state_publisher`) first; the node warns once and switches
  automatically when it appears.
- **tf `TF_MULTIPLE_PARENT` / oscillating tree.** You left `publish_tf:=true`
  while the EKF also publishes `map → odom`. Set `publish_tf:=false`.
- **Global pose drifts / jumps in scale (monocular).** Monocular is non-metric;
  use stereo/RGBD or a post-IMU-init map, or raise `covariance_g2o_scale`.

[REP-105]: https://www.ros.org/reps/rep-0105.html
