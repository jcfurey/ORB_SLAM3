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
find_package(eigen3_cmake_module QUIET)   # optional: cleaner Eigen3 export under ament
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
ament_export_dependencies(OpenCV Eigen3 Boost OpenSSL)
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
