# ORB-SLAM3 as a ROS 2 (ament) package

This fork packages ORB-SLAM3 V1.0 as a first-class **ROS 2 `ament_cmake`
library** using the `ament_cmake_auto` conventions. One `colcon build` compiles
the ORB-SLAM3 core **and** its vendored third-party dependencies (DBoW2, g2o,
Sophus) in-tree, then installs and exports everything so your own ROS 2 nodes
can depend on it with a plain `find_package(orb_slam3)`.

- **Package name:** `orb_slam3` (lowercase, REP-144 compliant, lint-clean).
- **C++ namespace / headers:** unchanged — still `ORB_SLAM3::System`, `#include "System.h"`, etc.
- **Tested on:** ROS 2 **Jazzy** (Ubuntu 24.04, GCC 13, OpenCV 4.6). Forward-compatible with **Kilted / Rolling**.

> The original standalone `build.sh` / plain-CMake flow still works if you don't
> want ROS 2 — but for ROS 2 you no longer need it. See "Standalone build" below.

---

## 1. Dependencies

Everything except Pangolin is resolvable by `rosdep`:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

This installs OpenCV (`libopencv-dev`), Eigen (`eigen`), Boost.serialization
(`libboost-serialization-dev`) and OpenSSL (`libssl-dev`).

### Pangolin (viewer only — optional)

Pangolin has **no rosdep key** and is not packaged for Ubuntu 24.04. You have two choices:

- **Headless (recommended for robots / CI):** build with `-DUSE_PANGOLIN=OFF`.
  No Pangolin needed at all; visualize the map/pose in **RViz** instead
  (publish TF + `nav_msgs/Path` + `sensor_msgs/PointCloud2` from your node).
- **With the built-in viewer:** build Pangolin from source once:

  ```bash
  sudo apt install -y libgl1-mesa-dev libglew-dev libegl1-mesa-dev \
       libepoxy-dev libeigen3-dev
  git clone --branch v0.9.3 --depth 1 https://github.com/stevenlovegrove/Pangolin.git
  cmake -S Pangolin -B Pangolin/build -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_PANGOLIN_PYTHON=OFF
  cmake --build Pangolin/build -j"$(nproc)"
  sudo cmake --install Pangolin/build && sudo ldconfig
  ```

---

## 2. Build

```bash
mkdir -p ~/ros2_ws/src
ln -s /path/to/ORB_SLAM3 ~/ros2_ws/src/orb_slam3     # or clone into src/
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash

# Headless (no Pangolin):
colcon build --packages-select orb_slam3 --cmake-args -DUSE_PANGOLIN=OFF

# …or with the Pangolin viewer (default):
colcon build --packages-select orb_slam3
```

On machines with < 16 GB RAM, cap parallelism to avoid the compiler being
OOM-killed on `Optimizer.cc`/g2o: `MAKEFLAGS="-j2" colcon build ...`.

### Build options (`--cmake-args -D<OPT>=<VAL>`)

| Option | Default | Meaning |
| --- | --- | --- |
| `USE_PANGOLIN` | `ON` | Build the Pangolin viewer. `OFF` = headless, **no Pangolin dependency**. |
| `ORB_SLAM3_INSTALL_VOCABULARY` | `ON` | Extract `ORBvoc.txt` at build time and install it to `share/orb_slam3/vocabulary/`. |
| `ORB_SLAM3_BUILD_EXAMPLES` | `OFF` | Also build the standalone dataset tools (EuRoC/KITTI/TUM…). |
| `ORB_SLAM3_NATIVE_ARCH` | `OFF` | Add `-march=native`. Faster, but **non-portable** (SIGILL on other CPUs) — do not use for CI or redistributable binaries. |

---

## 3. Use it from your own ROS 2 node

**`your_node/package.xml`**

```xml
<depend>orb_slam3</depend>
<depend>rclcpp</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
```

**`your_node/CMakeLists.txt`**

```cmake
find_package(ament_cmake REQUIRED)
find_package(orb_slam3 REQUIRED)          # pulls includes + libs + OpenCV/Eigen/Boost/OpenSSL(/Pangolin)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mono_node src/mono_node.cpp)
ament_target_dependencies(mono_node orb_slam3 rclcpp sensor_msgs cv_bridge)
install(TARGETS mono_node DESTINATION lib/${PROJECT_NAME})
```

**`your_node/src/mono_node.cpp`** (sketch)

```cpp
#include "System.h"                                   // ORB-SLAM3 public header
#include <ament_index_cpp/get_package_share_directory.hpp>

const std::string share = ament_index_cpp::get_package_share_directory("orb_slam3");
const std::string voc   = share + "/vocabulary/ORBvoc.txt";

ORB_SLAM3::System SLAM(voc, settings_yaml, ORB_SLAM3::System::MONOCULAR,
                       /*bUseViewer=*/false);         // false = headless
// ... in your image callback:
Sophus::SE3f Tcw = SLAM.TrackMonocular(cv_image, timestamp_sec);
```

`find_package(orb_slam3)` exports the include directories (including the
vendored `Thirdparty/…` headers that ORB-SLAM3's public headers pull in) and the
`orb_slam3`, `DBoW2` and `g2o` libraries, and re-finds OpenCV/Eigen/Boost/OpenSSL
(and Pangolin, if built with the viewer). You do **not** need to touch the
`Thirdparty/` layout yourself.

---

## 4. Distro compatibility (Humble / Jazzy / Lyrical)

The package targets the full ROS 2 LTS suite. Two APIs differ across it and are
shimmed; everything else was audited against the per-distro upstream branches
and is compatible unmodified:

| API | Humble | Jazzy | Lyrical | Handling |
| --- | --- | --- | --- | --- |
| `message_filters` Subscriber/QoS API | 4.x legacy | 4.x legacy | ≥6 modern | `mf_compat.hpp`, gated by `message_filters_VERSION` in CMake (`<6` → legacy) |
| `cv_bridge` header name | `.h` only | `.hpp` | `.hpp` | `distro_compat.hpp` (`__has_include`) |
| `tf2_eigen.hpp`, sensor_msgs `*.hpp`, `diagnostic_updater.hpp` | ✓ | ✓ | ✓ | none needed |
| `${pkg}_TARGETS` CMake vars (replacement for the removed `ament_target_dependencies`) | ✓ | ✓ | ✓ | none needed |
| Lifecycle `configure()`/`activate()`, `diagnostic_updater::Updater(node)`, lifecycle components via `rclcpp_components` | ✓ | ✓ | ✓ | none needed |

Verified by building on Jazzy (this repo's CI reference) and on a Lyrical host;
Humble compatibility is by API audit of the `humble` branches (message_filters,
vision_opencv, geometry2, ament_cmake, rclcpp, diagnostics, common_interfaces) —
if you build on Humble and hit anything, please open an issue.

## 5. What changed vs. upstream ORB-SLAM3

- **`CMakeLists.txt`** rewritten as an `ament_cmake` / `ament_cmake_auto` package
  that builds DBoW2 + g2o in-tree (`add_subdirectory`), links Sophus as a
  header-only include, extracts + installs the ORB vocabulary, and installs +
  exports the library and headers.
- **`package.xml`** added (`ament_cmake` build type, format 3).
- **`-march=native` removed** from the top-level, DBoW2 and g2o CMake (the most
  common cause of `SIGILL`/"illegal instruction" crashes and un-redistributable
  binaries). Re-enable with `-DORB_SLAM3_NATIVE_ARCH=ON` if you build and run on
  the same machine.
- **C++17** (ROS 2 Jazzy/Kilted default) instead of C++11.
- **Optional Pangolin** via `USE_PANGOLIN` — the viewer/`MapDrawer`/`Map`
  headers no longer force a Pangolin dependency on the whole library, enabling a
  fully headless, rosdep-only build. The SLAM algorithms are untouched.

## Standalone (non-ROS) build

The classic flow is unchanged:

```bash
./build.sh                       # builds Thirdparty + the library + examples
```

or configure the CMake directly with `-DORB_SLAM3_BUILD_EXAMPLES=ON` outside a
colcon workspace (you'll need Pangolin, or pass `-DUSE_PANGOLIN=OFF`).
