// Cross-distro include shims for the ROS 2 LTS suite (Humble / Jazzy / Lyrical).
//
// cv_bridge: the header was renamed cv_bridge.h -> cv_bridge.hpp in Iron;
// Humble ships only the .h. Everything else this package includes
// (tf2_eigen.hpp, sensor_msgs *.hpp, diagnostic_updater.hpp,
// tf2_ros/transform_broadcaster.h) exists under the same name on all three
// distros — verified against the humble branches. message_filters has its own
// shim in mf_compat.hpp (API differences, not just the header name).
#ifndef ORB_SLAM3_ROS__DISTRO_COMPAT_HPP_
#define ORB_SLAM3_ROS__DISTRO_COMPAT_HPP_

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>   // Humble
#endif

#endif  // ORB_SLAM3_ROS__DISTRO_COMPAT_HPP_
