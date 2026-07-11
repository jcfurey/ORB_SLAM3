// Managed (lifecycle) base for the ORB-SLAM3 ROS 2 wrapper nodes.
//
// Design goals:
//  * Lifecycle: rclcpp_lifecycle::LifecycleNode. The heavy ORB-SLAM3 system is
//    created in on_configure, sensor subscriptions live only in the ACTIVE state.
//  * TF2 first-class: the camera pose is broadcast as a geometry_msgs/TransformStamped
//    (world_frame_id -> camera_frame_id) stamped with the source image time, built
//    from the Sophus pose via tf2_eigen.
//  * Diagnostics: a diagnostic_updater task reports tracking state and rate.
//  * Camera-agnostic + robust: every topic is a parameter and the subscription QoS
//    is configurable (default SensorData/BEST_EFFORT to match RealSense / OAK-D /
//    LUCID drivers). RMW-agnostic — no rmw-implementation-specific code.
#ifndef ORB_SLAM3_ROS__ORB_SLAM3_NODE_HPP_
#define ORB_SLAM3_ROS__ORB_SLAM3_NODE_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <opencv2/core.hpp>
#include <sophus/se3.hpp>

#include "System.h"
#include "ImuTypes.h"

namespace orb_slam3_ros
{

class OrbSlam3LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  OrbSlam3LifecycleNode(
    const std::string & node_name,
    ORB_SLAM3::System::eSensor sensor,
    const rclcpp::NodeOptions & options);

  ~OrbSlam3LifecycleNode() override;

  // Managed-node transitions.
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

protected:
  // Each sensor node creates/destroys its own subscriptions (topics are its params).
  virtual void createSubscriptions() = 0;
  virtual void destroySubscriptions() = 0;

  // Publish TF2 + pose + path + map cloud for one tracking result.
  // Tcw = world->camera as returned by System::Track* (empty/identity if lost).
  void publishTracking(const Sophus::SE3f & Tcw, const rclcpp::Time & stamp);

  // --- helpers for subclasses ---
  rclcpp::QoS sensorQoS() const;                 // configurable image/IMU QoS
  rmw_qos_profile_t sensorQoSProfile() const;    // same, for message_filters
  cv::Mat toMono(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const;
  cv::Mat toDepth(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const;
  void pushImu(const sensor_msgs::msg::Imu & imu);
  // IMU measurements in (last_frame, t_frame], consumed (drained) from the buffer.
  std::vector<ORB_SLAM3::IMU::Point> drainImu(double t_frame);
  bool active() const {return active_.load();}

  ORB_SLAM3::System::eSensor sensor_type_;
  std::shared_ptr<ORB_SLAM3::System> slam_;

private:
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  // parameters
  std::string voc_file_;
  std::string settings_file_;
  std::string world_frame_id_;
  std::string camera_frame_id_;
  std::string qos_reliability_;
  int qos_depth_{5};
  bool use_viewer_{false};
  bool publish_tf_{true};
  bool publish_pose_{true};
  bool publish_path_{true};
  bool publish_pointcloud_{true};
  bool autostart_{true};

  // publishers / tf / diagnostics
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<diagnostic_updater::Updater> diag_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;

  nav_msgs::msg::Path path_;

  // IMU buffer
  std::mutex imu_mutex_;
  std::deque<ORB_SLAM3::IMU::Point> imu_buffer_;
  double last_imu_drain_t_{-1.0};

  // diagnostics state
  std::atomic<bool> active_{false};
  std::atomic<int> last_tracking_state_{-1};
  std::atomic<uint64_t> frames_tracked_{0};
  rclcpp::Time last_track_time_;
  double track_rate_hz_{0.0};
};

}  // namespace orb_slam3_ros

#endif  // ORB_SLAM3_ROS__ORB_SLAM3_NODE_HPP_
