# orb_slam3_ros — ROS 2 wrapper nodes

Managed (lifecycle) ROS 2 nodes that run ORB-SLAM3 and publish the camera pose
over **TF2** plus `PoseStamped`, `Path`, and a `PointCloud2` of tracked map
points. Built as part of the `orb_slam3` package (colcon does not discover a
package nested inside another), so `ros2 run orb_slam3 <node>` and
`ros2 launch orb_slam3 <file>` are used.

Design highlights:

- **Lifecycle** — every node is a `rclcpp_lifecycle::LifecycleNode`. The heavy
  ORB-SLAM3 system is created in `on_configure`; sensor subscriptions exist only
  in the **active** state. `autostart:=true` (default) self-drives to *active*.
- **TF2, first-class** — the camera pose is broadcast as a
  `geometry_msgs/TransformStamped` (`world_frame_id → camera_frame_id`), stamped
  with the source image time and built from the Sophus pose via `tf2_eigen`.
- **Diagnostics** — a `diagnostic_updater` task publishes `/diagnostics`
  (tracking state, frame rate, IMU-buffer depth). Green when `OK`, warn when
  initializing / recently-lost, error when lost.
- **RMW-agnostic** — no RMW-specific code; QoS chosen for cross-vendor
  compatibility (works with Fast DDS, Cyclone DDS, …).
- **Robust camera integration** — every topic is a parameter and the
  subscription **QoS is configurable** (default `sensor_data` / BEST_EFFORT to
  match the RealSense / OAK-D / LUCID drivers — a reliable subscriber would
  silently receive nothing from a best-effort publisher).

## Nodes

| Executable | ORB-SLAM3 sensor | Key topic params |
| --- | --- | --- |
| `mono_node` | MONOCULAR | `image_topic` |
| `rgbd_node` | RGBD | `rgb_topic`, `depth_topic` |
| `stereo_node` | STEREO | `left_topic`, `right_topic` |
| `stereo_inertial_node` | IMU_STEREO (VIO) | `left_topic`, `right_topic`, `imu_topic` |

### Common parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `settings_file` | *(required)* | ORB-SLAM3 `.yaml` (see `config/README.md`) |
| `voc_file` | `<share>/orb_slam3/vocabulary/ORBvoc.txt` | ORB vocabulary |
| `world_frame_id` | `map` | TF/pose parent frame |
| `camera_frame_id` | `camera` | TF child frame (the camera optical frame) |
| `qos_reliability` | `sensor_data` | `sensor_data` (best-effort) or `reliable` |
| `qos_depth` | `5` | subscription queue depth |
| `sync_queue_size` | `30` | approximate-time sync queue (rgbd/stereo) |
| `publish_tf` / `publish_pose` / `publish_path` / `publish_pointcloud` | `true` | output toggles |
| `path_max_poses` | `1000` | cap on `~/path` length (`0` = unlimited) |
| `path_min_distance` | `0.05` | metres moved before appending to `~/path` (`0` = every frame) |
| `path_publish_period` | `1.0` | seconds between `~/path` republishes (`0` = every append) |
| `use_pangolin_viewer` | `false` | open the Pangolin window (needs a viewer build) |
| `autostart` | `true` | self-configure + activate on start |

### Published topics (per node, namespaced under the node name)

- `~/pose` — `geometry_msgs/PoseStamped` (camera in `world_frame_id`)
- `~/path` — `nav_msgs/Path`
- `~/map_points` — `sensor_msgs/PointCloud2` (tracked map points, latched by demand)
- TF: `world_frame_id → camera_frame_id`
- `/diagnostics` — `diagnostic_msgs/DiagnosticArray`

> **Underwater / ROV (RTSP streams):** see `docs/UNDERWATER_ROV.md` for a
> literature-backed assessment, in-water calibration guidance, RTSP-bridge
> timestamping (vision-only vs. inertial), and the recommended
> fuse-with-DVL/depth architecture.

## Camera quick-starts

> The camera driver is separate; run it first. `settings_file` must match the
> stream (intrinsics/resolution/fps and, for VIO, IMU noise + extrinsics).

**RealSense D435i — RGB-D**
```bash
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
ros2 launch orb_slam3 realsense_rgbd.launch.py settings_file:=/abs/D435i_rgbd.yaml
```

**RealSense D435i — stereo-inertial (VIO)** (projector off, united IMU)
```bash
ros2 launch realsense2_camera rs_launch.py enable_infra1:=true enable_infra2:=true \
     enable_gyro:=true enable_accel:=true unite_imu_method:=2 depth_module.emitter_enabled:=0
ros2 launch orb_slam3 realsense_stereo_inertial.launch.py settings_file:=/abs/D435i_vio.yaml
```

**Luxonis OAK-D — stereo-inertial**
```bash
# depthai_ros_driver publishing rectified mono stereo + combined IMU
ros2 launch orb_slam3 oakd_stereo_inertial.launch.py settings_file:=/abs/oakd_vio.yaml \
     left_topic:=/oak/left/image_rect right_topic:=/oak/right/image_rect imu_topic:=/oak/imu/data
```

**LUCID (arena_camera) — monocular**
```bash
ros2 launch orb_slam3 lucid_mono.launch.py settings_file:=/abs/lucid_mono.yaml \
     image_topic:=/arena_camera_node/image_raw
# LUCID drivers commonly use RELIABLE QoS -> this launch defaults qos_reliability:=reliable
```

Any node also runs standalone, e.g.:
```bash
ros2 run orb_slam3 mono_node --ros-args -p settings_file:=/abs/mono.yaml -p image_topic:=/my/image
```

## Composable components

Each node is also registered as a `rclcpp_components` component
(`orb_slam3_ros::MonoNode`, `RgbdNode`, `StereoNode`, `StereoInertialNode`), so
it can be loaded into a shared process (zero-copy intra-process comms with the
camera driver when it is composed too):
```bash
ros2 run rclcpp_components component_container
ros2 component load /ComponentManager orb_slam3 orb_slam3_ros::MonoNode \
  -p settings_file:=/abs/mono.yaml -p image_topic:=/camera/image_raw
```

## Visualization (RViz) + connecting to your robot

```bash
ros2 launch orb_slam3 rviz.launch.py                     # opens the bundled config
```
The config's *Fixed Frame* is `map` and it shows TF, the camera pose (axes), the
path, and the map-point cloud (topics default to the mono node — change them for
rgbd/stereo).

Publish a static transform from your robot body to the camera frame so the SLAM
`map → camera` chain connects to `base_link` (set the mounting to match your rig;
`child_frame` MUST equal the node's `camera_frame_id`):
```bash
ros2 launch orb_slam3 base_to_camera.launch.py \
     parent_frame:=base_link child_frame:=camera x:=0.10 z:=0.20 roll:=-1.5708 yaw:=-1.5708
```

## Diagnosing "no data"

Ninety percent of "it launched but never tracks" is a **QoS mismatch** or a
**wrong topic**. Check:
```bash
ros2 topic list                                   # confirm the driver topics
ros2 topic hz <image_topic>                        # confirm data is flowing
ros2 topic info <image_topic> --verbose            # check the publisher's QoS
ros2 run diagnostic_updater ...                    # or: ros2 topic echo /diagnostics
```
If the publisher is `RELIABLE`, set `-p qos_reliability:=reliable`; if it is
`BEST_EFFORT` (the RealSense/OAK default), keep the `sensor_data` default.

## Lifecycle control (manual, when `autostart:=false`)

```bash
ros2 lifecycle set /orb_slam3_mono configure
ros2 lifecycle set /orb_slam3_mono activate
ros2 lifecycle set /orb_slam3_mono deactivate   # stop feeding frames, keep the map
ros2 lifecycle set /orb_slam3_mono cleanup      # destroy the SLAM system
```

## Frames

ORB-SLAM3's world frame is defined by the first keyframe (visual) or is
gravity-aligned after IMU initialization (inertial). `camera_frame_id` is the
**camera optical frame** (x-right, y-down, z-forward). Chain it to your robot's
`base_link` with a static transform for your specific mounting (REP-103/REP-105);
the per-camera launches default `camera_frame_id` to each driver's optical frame.
Visualize in RViz: set *Fixed Frame* to `map`, add TF, the `~/pose`/`~/path`, and
the `~/map_points` `PointCloud2`.
