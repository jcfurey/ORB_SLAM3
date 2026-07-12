# ==============================================================================
#  ORB-SLAM3 — ROS 2 (ament_cmake / ament_cmake_auto) package
#
#  Builds the ORB-SLAM3 core shared library together with its vendored
#  Thirdparty dependencies (DBoW2, g2o) in-tree, so a single `colcon build`
#  produces a self-contained, installable, exportable package. Downstream ROS 2
#  nodes consume it with:
#
#      find_package(orb_slam3 REQUIRED)
#      ament_target_dependencies(my_node orb_slam3)   # or target_link_libraries
#
#  Tested on ROS 2 Jazzy (Ubuntu 24.04, GCC 13, OpenCV 4.6); forward-compatible
#  with Kilted / Rolling. The C++ namespace remains ORB_SLAM3.
# ==============================================================================
cmake_minimum_required(VERSION 3.10)
project(orb_slam3 VERSION 1.0.0 LANGUAGES C CXX)

# ------------------------------------------------------------------ build type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "[orb_slam3] Build type: ${CMAKE_BUILD_TYPE}")

# ----------------------------------------------------------- C++ standard (17)
# ROS 2 Jazzy/Kilted default to C++17. ORB-SLAM3 v1.0 was written for C++11;
# it compiles cleanly under C++17. COMPILEDWITHC11 keeps its std::chrono path.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------- options
option(USE_PANGOLIN
  "Build the Pangolin-based viewer. OFF = headless build with NO Pangolin dependency \
(recommended for ROS 2 robots: visualize in RViz instead)." ON)
option(ORB_SLAM3_NATIVE_ARCH
  "Add -march=native. Faster, but non-portable: binaries may SIGILL on a different \
CPU and must not be redistributed or built in CI. Keep OFF unless you build and run \
on the exact same machine." OFF)
option(ORB_SLAM3_BUILD_EXAMPLES
  "Build the standalone (non-ROS) dataset example executables (EuRoC/KITTI/TUM/...)." OFF)
option(ORB_SLAM3_INSTALL_VOCABULARY
  "Extract Vocabulary/ORBvoc.txt.tar.gz at build time and install it to share/." ON)
option(ORB_SLAM3_BUILD_ROS_NODES
  "Build the ROS 2 wrapper nodes (mono / rgbd / stereo / stereo-inertial)." ON)

# --------------------------------------------------------------- compile flags
add_compile_definitions(COMPILEDWITHC11)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-O3 -Wall)
  if(ORB_SLAM3_NATIVE_ARCH)
    message(STATUS "[orb_slam3] -march=native ENABLED (non-portable build)")
    add_compile_options(-march=native)
  endif()
endif()

# --------------------------------------------------------- ament + build deps
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)

# Non-ament system dependencies. ament_auto_find_build_dependencies() cannot
# resolve these (their rosdep keys are not CMake package-config names), so they
# are found explicitly and linked by hand below.
find_package(OpenCV REQUIRED)
# eigen3_cmake_module makes find_package(Eigen3) populate Eigen3_INCLUDE_DIRS so
# ament_export_dependencies(Eigen3) actually propagates Eigen's headers to
# downstream (Eigen3's own config only provides the Eigen3::Eigen target).
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)
find_package(OpenSSL REQUIRED)            # System.cc uses the OpenSSL MD5 API
if(USE_PANGOLIN)
  find_package(Pangolin REQUIRED)
endif()

# Resolves any ROS/ament <depend>s that DO map to CMake configs (none needed by
# the core library today, but keeps the package ament_auto-idiomatic).
ament_auto_find_build_dependencies()

message(STATUS "[orb_slam3] OpenCV ${OpenCV_VERSION}, Eigen ${Eigen3_VERSION}, "
               "Pangolin viewer: ${USE_PANGOLIN}")

# ------------------------------------------ vendored Thirdparty, built in-tree
# One colcon build compiles DBoW2 and g2o as normal CMake targets (`DBoW2`,
# `g2o`). Sophus is header-only and is consumed as an include directory only
# (do NOT add_subdirectory it — its own CMake builds -Werror tests/examples).
add_subdirectory(Thirdparty/DBoW2)
add_subdirectory(Thirdparty/g2o)

# ------------------------------------------------------- ORB-SLAM3 core library
set(ORB_SLAM3_SOURCES
  src/System.cc
  src/Tracking.cc
  src/LocalMapping.cc
  src/LoopClosing.cc
  src/ORBextractor.cc
  src/ORBmatcher.cc
  src/FrameDrawer.cc
  src/Converter.cc
  src/MapPoint.cc
  src/KeyFrame.cc
  src/Atlas.cc
  src/Map.cc
  src/MapDrawer.cc
  src/Optimizer.cc
  src/Frame.cc
  src/KeyFrameDatabase.cc
  src/Sim3Solver.cc
  src/Viewer.cc
  src/ImuTypes.cc
  src/G2oTypes.cc
  src/CameraModels/Pinhole.cpp
  src/CameraModels/KannalaBrandt8.cpp
  src/OptimizableTypes.cpp
  src/MLPnPsolver.cpp
  src/GeometricTools.cc
  src/TwoViewReconstruction.cc
  src/Config.cc
  src/Settings.cc)

ament_auto_add_library(${PROJECT_NAME} SHARED ${ORB_SLAM3_SOURCES})

if(USE_PANGOLIN)
  # PRIVATE: the viewer is an implementation detail of the library's own .cc
  # files. The public headers are pangolin-agnostic, so downstream consumers
  # neither need this definition nor a Pangolin installation.
  target_compile_definitions(${PROJECT_NAME} PRIVATE USE_PANGOLIN)
endif()

# Public include tree. ORB-SLAM3's public headers #include their vendored deps
# with "Thirdparty/..." paths (relative to the repo root) and Sophus as
# "sophus/..." (relative to Thirdparty/Sophus), so both roots are exposed.
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/CameraModels>"
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Thirdparty/Sophus>"
  "$<INSTALL_INTERFACE:include>"
  "$<INSTALL_INTERFACE:include/CameraModels>"
  "$<INSTALL_INTERFACE:include/Thirdparty/Sophus>")

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS}
  Eigen3::Eigen
  Boost::serialization
  OpenSSL::Crypto
  DBoW2
  g2o)

if(USE_PANGOLIN)
  target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${Pangolin_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} ${Pangolin_LIBRARIES})
endif()

# ------------------------------------------------ install vendored .so + headers
# ament_auto_package() installs/exports the core library and the include/ tree
# (to include); we additionally install the vendored libraries
# and the Thirdparty headers that the public ORB-SLAM3 headers pull in.
install(TARGETS DBoW2 g2o
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY Thirdparty/DBoW2/DBoW2 Thirdparty/DBoW2/DUtils
  DESTINATION include/Thirdparty/DBoW2
  FILES_MATCHING PATTERN "*.h")
install(DIRECTORY Thirdparty/g2o/g2o
  DESTINATION include/Thirdparty/g2o
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")
install(FILES Thirdparty/g2o/config.h
  DESTINATION include/Thirdparty/g2o)
install(DIRECTORY Thirdparty/Sophus/sophus
  DESTINATION include/Thirdparty/Sophus
  FILES_MATCHING PATTERN "*.hpp")

# ------------------------------------------------------------- ORB vocabulary
if(ORB_SLAM3_INSTALL_VOCABULARY)
  set(_voc_dir ${CMAKE_CURRENT_BINARY_DIR}/vocabulary)
  file(MAKE_DIRECTORY ${_voc_dir})   # must exist before the rule's WORKING_DIRECTORY is used
  add_custom_command(
    OUTPUT ${_voc_dir}/ORBvoc.txt
    COMMAND ${CMAKE_COMMAND} -E tar xzf
            ${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz
    WORKING_DIRECTORY ${_voc_dir}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/Vocabulary/ORBvoc.txt.tar.gz
    COMMENT "[orb_slam3] Extracting ORB vocabulary")
  add_custom_target(orb_slam3_extract_vocabulary ALL DEPENDS ${_voc_dir}/ORBvoc.txt)
  install(FILES ${_voc_dir}/ORBvoc.txt DESTINATION share/${PROJECT_NAME}/vocabulary)
endif()

# ------------------------------------------------------------------- exports
ament_export_include_directories(
  include
  include/CameraModels
  include/Thirdparty/Sophus)
ament_export_libraries(DBoW2 g2o)
ament_export_dependencies(eigen3_cmake_module OpenCV Eigen3 Boost OpenSSL)
if(USE_PANGOLIN)
  ament_export_dependencies(Pangolin)
endif()

# ----------------------------------------------- standalone dataset examples
if(ORB_SLAM3_BUILD_EXAMPLES)
  find_package(realsense2 QUIET)
  set(_ex_dir ${CMAKE_CURRENT_SOURCE_DIR}/Examples)
  function(orb_slam3_add_example name src)
    add_executable(${name} ${_ex_dir}/${src})
    target_link_libraries(${name} ${PROJECT_NAME})
    install(TARGETS ${name} RUNTIME DESTINATION lib/${PROJECT_NAME})
  endfunction()

  orb_slam3_add_example(rgbd_tum            RGB-D/rgbd_tum.cc)
  orb_slam3_add_example(stereo_kitti        Stereo/stereo_kitti.cc)
  orb_slam3_add_example(stereo_euroc        Stereo/stereo_euroc.cc)
  orb_slam3_add_example(stereo_tum_vi       Stereo/stereo_tum_vi.cc)
  orb_slam3_add_example(mono_tum            Monocular/mono_tum.cc)
  orb_slam3_add_example(mono_kitti          Monocular/mono_kitti.cc)
  orb_slam3_add_example(mono_euroc          Monocular/mono_euroc.cc)
  orb_slam3_add_example(mono_tum_vi         Monocular/mono_tum_vi.cc)
  orb_slam3_add_example(mono_inertial_euroc     Monocular-Inertial/mono_inertial_euroc.cc)
  orb_slam3_add_example(mono_inertial_tum_vi    Monocular-Inertial/mono_inertial_tum_vi.cc)
  orb_slam3_add_example(stereo_inertial_euroc   Stereo-Inertial/stereo_inertial_euroc.cc)
  orb_slam3_add_example(stereo_inertial_tum_vi  Stereo-Inertial/stereo_inertial_tum_vi.cc)
  message(STATUS "[orb_slam3] Building standalone examples (realsense2: ${realsense2_FOUND})")
endif()

# ---------------------------------------------------- ROS 2 wrapper nodes
# Built in the same package as the library (colcon does not discover a package
# nested inside another), so the nodes link the ORB-SLAM3 target directly. The
# node dependencies do not leak to downstream find_package(orb_slam3) consumers.
if(ORB_SLAM3_BUILD_ROS_NODES)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_lifecycle REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(diagnostic_updater REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(message_filters REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(tf2_eigen REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)

  set(_orb_ros_deps
    rclcpp rclcpp_lifecycle lifecycle_msgs ament_index_cpp
    sensor_msgs geometry_msgs nav_msgs
    diagnostic_msgs diagnostic_updater cv_bridge message_filters
    tf2 tf2_ros tf2_eigen tf2_geometry_msgs)

  find_package(rclcpp_components REQUIRED)

  # One component library holding the common impl + all four node classes.
  add_library(orb_slam3_ros SHARED
    orb_slam3_ros/src/orb_slam3_node.cpp
    orb_slam3_ros/src/mono_node.cpp
    orb_slam3_ros/src/rgbd_node.cpp
    orb_slam3_ros/src/stereo_node.cpp
    orb_slam3_ros/src/stereo_inertial_node.cpp)
  target_include_directories(orb_slam3_ros PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/orb_slam3_ros/include>")
  target_link_libraries(orb_slam3_ros ${PROJECT_NAME} ${OpenCV_LIBS})

  # message_filters dual-distro support (see orb_slam3_ros/mf_compat.hpp):
  # 4.x (Jazzy) subscribes with Subscriber<M, NodeT> + rmw_qos_profile_t;
  # >= 6 (Kilted/Lyrical) with the interface-based Subscriber<M> + rclcpp::QoS.
  if(message_filters_VERSION VERSION_LESS 6)
    target_compile_definitions(orb_slam3_ros PRIVATE ORB_SLAM3_MF_LEGACY_API=1)
    message(STATUS "[orb_slam3] message_filters ${message_filters_VERSION}: legacy (Jazzy) API")
  else()
    message(STATUS "[orb_slam3] message_filters ${message_filters_VERSION}: modern API")
  endif()
  # ament_target_dependencies() was removed in ament_cmake 2.8 (the ROS 'lyrical'
  # toolchain on this host). Link each dependency's exported IMPORTED targets
  # instead: find_package(<pkg>) populates ${<pkg>_TARGETS}, which carry that
  # package's include dirs and libraries as usage requirements — the transitive
  # propagation the old macro's ${<pkg>_INCLUDE_DIRS}/link vars used to provide.
  # The DEFINED guard mirrors resple/CMakeLists.txt and tolerates any dep that
  # exports no *_TARGETS (all node deps here do).
  foreach(_dep IN LISTS _orb_ros_deps)
    if(DEFINED ${_dep}_TARGETS)
      target_link_libraries(orb_slam3_ros ${${_dep}_TARGETS})
    endif()
  endforeach()
  target_link_libraries(orb_slam3_ros ${rclcpp_components_TARGETS})

  # Each node is registered as a composable component AND gets a standalone
  # executable (installed to lib/${PROJECT_NAME}) generated by rclcpp_components.
  # So both `ros2 run orb_slam3 mono_node` and `ros2 component load ... MonoNode` work.
  rclcpp_components_register_node(orb_slam3_ros
    PLUGIN "orb_slam3_ros::MonoNode" EXECUTABLE mono_node)
  rclcpp_components_register_node(orb_slam3_ros
    PLUGIN "orb_slam3_ros::RgbdNode" EXECUTABLE rgbd_node)
  rclcpp_components_register_node(orb_slam3_ros
    PLUGIN "orb_slam3_ros::StereoNode" EXECUTABLE stereo_node)
  rclcpp_components_register_node(orb_slam3_ros
    PLUGIN "orb_slam3_ros::StereoInertialNode" EXECUTABLE stereo_inertial_node)

  install(TARGETS orb_slam3_ros
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
  install(DIRECTORY orb_slam3_ros/launch orb_slam3_ros/config orb_slam3_ros/rviz
    DESTINATION share/${PROJECT_NAME})
  message(STATUS "[orb_slam3] Building ROS 2 wrapper nodes + composable components")
endif()

# ----------------------------------------------------------------- lint / test
if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    # The vendored Thirdparty (DBoW2/g2o/Sophus) is upstream code; don't lint it.
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    set(ament_cmake_uncrustify_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()

ament_auto_package()
