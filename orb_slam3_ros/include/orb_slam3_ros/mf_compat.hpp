// message_filters compatibility shim: one node source, two distro APIs.
//
// Jazzy ships message_filters 4.x, whose Subscriber is templated on the node
// type and subscribes with an rmw_qos_profile_t. Kilted/Lyrical (>= 6.x) use the
// interface-based Subscriber (no node-type template parameter) and subscribe
// with rclcpp::QoS. CMake defines ORB_SLAM3_MF_LEGACY_API=1 when
// message_filters_VERSION < 6 so the same node code builds on both stacks.
#ifndef ORB_SLAM3_ROS__MF_COMPAT_HPP_
#define ORB_SLAM3_ROS__MF_COMPAT_HPP_

#include <string>

// Current Humble/Jazzy releases ship .hpp alongside .h, but older Humble patch
// levels predate the .hpp backport — fall back to .h where needed.
#if __has_include(<message_filters/subscriber.hpp>)
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#else
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#endif
#include <rclcpp/qos.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace orb_slam3_ros
{

#if defined(ORB_SLAM3_MF_LEGACY_API) && ORB_SLAM3_MF_LEGACY_API

template<typename M>
using MfSubscriber = message_filters::Subscriber<M, rclcpp_lifecycle::LifecycleNode>;

template<typename M>
inline void mfSubscribe(
  MfSubscriber<M> & sub, rclcpp_lifecycle::LifecycleNode * node,
  const std::string & topic, const rclcpp::QoS & qos)
{
  sub.subscribe(node, topic, qos.get_rmw_qos_profile());
}

#else  // modern (Kilted/Lyrical) API

template<typename M>
using MfSubscriber = message_filters::Subscriber<M>;

template<typename M>
inline void mfSubscribe(
  MfSubscriber<M> & sub, rclcpp_lifecycle::LifecycleNode * node,
  const std::string & topic, const rclcpp::QoS & qos)
{
  sub.subscribe(node, topic, qos);
}

#endif

}  // namespace orb_slam3_ros

#endif  // ORB_SLAM3_ROS__MF_COMPAT_HPP_
