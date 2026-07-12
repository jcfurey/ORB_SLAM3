#include "orb_slam3_ros/orb_slam3_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Geometry>

#include "MapPoint.h"

using namespace std::chrono_literals;

namespace orb_slam3_ros
{
namespace
{
// ORB_SLAM3::Tracking::eTrackingState values (kept as literals to avoid pulling
// in the internal Tracking.h enum): OK == 2, RECENTLY_LOST == 3, LOST == 4.
constexpr int kOk = 2;
}  // namespace

OrbSlam3LifecycleNode::OrbSlam3LifecycleNode(
  const std::string & node_name,
  ORB_SLAM3::System::eSensor sensor,
  const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, options),
  sensor_type_(sensor),
  last_track_time_(0, 0, RCL_ROS_TIME)
{
  std::string default_voc;
  try {
    default_voc =
      ament_index_cpp::get_package_share_directory("orb_slam3") + "/vocabulary/ORBvoc.txt";
  } catch (const std::exception &) {
    default_voc = "";
  }

  declare_parameter<std::string>("voc_file", default_voc);
  declare_parameter<std::string>("settings_file", "");
  declare_parameter<bool>("use_pangolin_viewer", false);
  declare_parameter<std::string>("world_frame_id", "map");
  declare_parameter<std::string>("camera_frame_id", "camera");
  // Child frame stamped on ~/odom; empty -> reuse camera_frame_id.
  declare_parameter<std::string>("odom_child_frame_id", "");
  declare_parameter<std::string>("qos_reliability", "sensor_data");  // "sensor_data"|"reliable"
  declare_parameter<int>("qos_depth", 5);
  declare_parameter<bool>("publish_tf", true);
  declare_parameter<bool>("publish_pose", true);
  declare_parameter<bool>("publish_odom", true);  // nav_msgs/Odometry (pose + covariance) for fusion
  declare_parameter<bool>("publish_path", true);
  declare_parameter<bool>("publish_pointcloud", true);

  // Covariance model for ~/odom. "quality" (default) scales the base diagonal up
  // as tracking degrades; "static" emits the base diagonal unchanged; "g2o" uses
  // the true SE3 marginal from ORB-SLAM3's motion-only BA (falls back to quality
  // when the estimator reports none, e.g. IMU-dominated frames).
  declare_parameter<std::string>("covariance_mode", "quality");
  // Best-case (well-tracked) variance on [x, y, z, roll, pitch, yaw] (m^2, rad^2).
  declare_parameter<std::vector<double>>(
    "pose_covariance_diagonal", {0.01, 0.01, 0.01, 0.0025, 0.0025, 0.0025});
  // quality mode: inlier count at which the scale is ~1; scale = ref/inliers.
  declare_parameter<int>("covariance_inlier_ref", 100);
  // quality mode: clamp on the per-axis sigma scale (variance scales by its square).
  declare_parameter<double>("covariance_max_scale", 10.0);
  // quality mode: extra sigma multiplier while tracking state is RECENTLY_LOST.
  declare_parameter<double>("covariance_recently_lost_scale", 5.0);
  // g2o mode: empirical variance multiplier (the raw reprojection marginal is
  // optimistic; >1 de-weights VO in the fusion). Applied after the frame transform.
  declare_parameter<double>("covariance_g2o_scale", 1.0);
  // g2o mode: include the SE3 adjoint lever-arm term (true = full world-frame
  // marginal; false = body-frame covariance with axes rotated into the world).
  declare_parameter<bool>("covariance_g2o_lever_arm", true);

  autostart_ = declare_parameter<bool>("autostart", true);

  if (autostart_) {
    // Self-drive the lifecycle to ACTIVE shortly after the executor starts.
    autostart_timer_ = create_wall_timer(200ms, [this]() {
      autostart_timer_->cancel();
      RCLCPP_INFO(get_logger(), "autostart: configure + activate");
      if (configure().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
      {
        activate();
      }
    });
  }
}

OrbSlam3LifecycleNode::~OrbSlam3LifecycleNode()
{
  if (slam_) {
    slam_->Shutdown();
    slam_.reset();
  }
}

OrbSlam3LifecycleNode::CallbackReturn
OrbSlam3LifecycleNode::on_configure(const rclcpp_lifecycle::State &)
{
  voc_file_ = get_parameter("voc_file").as_string();
  settings_file_ = get_parameter("settings_file").as_string();
  use_viewer_ = get_parameter("use_pangolin_viewer").as_bool();
  world_frame_id_ = get_parameter("world_frame_id").as_string();
  camera_frame_id_ = get_parameter("camera_frame_id").as_string();
  odom_child_frame_id_ = get_parameter("odom_child_frame_id").as_string();
  if (odom_child_frame_id_.empty()) {
    odom_child_frame_id_ = camera_frame_id_;
  }
  qos_reliability_ = get_parameter("qos_reliability").as_string();
  qos_depth_ = get_parameter("qos_depth").as_int();
  publish_tf_ = get_parameter("publish_tf").as_bool();
  publish_pose_ = get_parameter("publish_pose").as_bool();
  publish_odom_ = get_parameter("publish_odom").as_bool();
  publish_path_ = get_parameter("publish_path").as_bool();
  publish_pointcloud_ = get_parameter("publish_pointcloud").as_bool();

  const std::string cov_mode = get_parameter("covariance_mode").as_string();
  if (cov_mode == "static") {
    covariance_mode_ = CovarianceMode::kStatic;
  } else if (cov_mode == "g2o") {
    covariance_mode_ = CovarianceMode::kG2o;
  } else {
    if (cov_mode != "quality") {
      RCLCPP_WARN(get_logger(),
        "covariance_mode '%s' unknown; using 'quality'", cov_mode.c_str());
    }
    covariance_mode_ = CovarianceMode::kQuality;
  }
  {
    const std::vector<double> d = get_parameter("pose_covariance_diagonal").as_double_array();
    if (d.size() == 6) {
      for (size_t i = 0; i < 6; ++i) {pose_cov_diagonal_[i] = d[i];}
    } else {
      RCLCPP_WARN(get_logger(),
        "pose_covariance_diagonal must have 6 elements (got %zu); using defaults", d.size());
    }
  }
  cov_inlier_ref_ = std::max<int>(1, get_parameter("covariance_inlier_ref").as_int());
  cov_max_scale_ = std::max(1.0, get_parameter("covariance_max_scale").as_double());
  cov_recently_lost_scale_ = std::max(1.0, get_parameter("covariance_recently_lost_scale").as_double());
  cov_g2o_scale_ = std::max(1e-9, get_parameter("covariance_g2o_scale").as_double());
  cov_g2o_lever_arm_ = get_parameter("covariance_g2o_lever_arm").as_bool();

  if (settings_file_.empty()) {
    RCLCPP_ERROR(get_logger(), "Parameter 'settings_file' (ORB-SLAM3 .yaml) is required.");
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "Vocabulary : %s", voc_file_.c_str());
  RCLCPP_INFO(get_logger(), "Settings   : %s", settings_file_.c_str());
  RCLCPP_INFO(get_logger(), "TF frames  : %s -> %s",
    world_frame_id_.c_str(), camera_frame_id_.c_str());
  RCLCPP_INFO(get_logger(), "Sub QoS    : %s (depth %d)", qos_reliability_.c_str(), qos_depth_);

  try {
    slam_ = std::make_shared<ORB_SLAM3::System>(
      voc_file_, settings_file_, sensor_type_, use_viewer_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to construct ORB-SLAM3 System: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/pose", rclcpp::QoS(10));
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", rclcpp::QoS(10));
  path_pub_ = create_publisher<nav_msgs::msg::Path>("~/path", rclcpp::QoS(10));
  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/map_points", rclcpp::QoS(1));
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  diag_ = std::make_shared<diagnostic_updater::Updater>(this);
  diag_->setHardwareID("orb_slam3");
  diag_->add("tracking", this, &OrbSlam3LifecycleNode::produceDiagnostics);

  path_ = nav_msgs::msg::Path();
  path_.header.frame_id = world_frame_id_;
  return CallbackReturn::SUCCESS;
}

OrbSlam3LifecycleNode::CallbackReturn
OrbSlam3LifecycleNode::on_activate(const rclcpp_lifecycle::State &)
{
  pose_pub_->on_activate();
  odom_pub_->on_activate();
  path_pub_->on_activate();
  cloud_pub_->on_activate();
  createSubscriptions();
  active_.store(true);
  RCLCPP_INFO(get_logger(), "activated");
  return CallbackReturn::SUCCESS;
}

OrbSlam3LifecycleNode::CallbackReturn
OrbSlam3LifecycleNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_.store(false);
  destroySubscriptions();
  pose_pub_->on_deactivate();
  odom_pub_->on_deactivate();
  path_pub_->on_deactivate();
  cloud_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "deactivated");
  return CallbackReturn::SUCCESS;
}

OrbSlam3LifecycleNode::CallbackReturn
OrbSlam3LifecycleNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  destroySubscriptions();
  diag_.reset();
  tf_broadcaster_.reset();
  pose_pub_.reset();
  odom_pub_.reset();
  path_pub_.reset();
  cloud_pub_.reset();
  if (slam_) {
    slam_->Shutdown();
    slam_.reset();
  }
  {
    std::lock_guard<std::mutex> lk(imu_mutex_);
    imu_buffer_.clear();
    last_imu_drain_t_ = -1.0;
  }
  return CallbackReturn::SUCCESS;
}

OrbSlam3LifecycleNode::CallbackReturn
OrbSlam3LifecycleNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  return on_cleanup(state);
}

rclcpp::QoS OrbSlam3LifecycleNode::sensorQoS() const
{
  rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(std::max(1, qos_depth_))));
  if (qos_reliability_ == "reliable") {
    qos.reliable();
  } else {
    qos.best_effort();  // matches RealSense / OAK-D / LUCID drivers by default
  }
  qos.durability_volatile();
  return qos;
}

cv::Mat OrbSlam3LifecycleNode::toMono(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
{
  // cv_bridge converts bgr8/rgb8/bgra8/rgba8/mono16/mono8 -> mono8 robustly, so
  // the node is agnostic to whether the driver publishes colour or gray.
  return cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
}

cv::Mat OrbSlam3LifecycleNode::toDepth(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
{
  // Keep the native depth encoding (16UC1 mm for RealSense/OAK, or 32FC1 m);
  // ORB-SLAM3 applies DepthMapFactor from the settings .yaml.
  return cv_bridge::toCvCopy(msg, msg->encoding)->image;
}

void OrbSlam3LifecycleNode::pushImu(const sensor_msgs::msg::Imu & imu)
{
  const cv::Point3f acc(
    imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z);
  const cv::Point3f gyr(
    imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z);
  const double t = rclcpp::Time(imu.header.stamp).seconds();
  std::lock_guard<std::mutex> lk(imu_mutex_);
  imu_buffer_.emplace_back(acc, gyr, t);
}

std::vector<ORB_SLAM3::IMU::Point> OrbSlam3LifecycleNode::drainImu(double t_frame)
{
  std::vector<ORB_SLAM3::IMU::Point> out;
  std::lock_guard<std::mutex> lk(imu_mutex_);
  while (!imu_buffer_.empty() && imu_buffer_.front().t <= t_frame) {
    out.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }
  last_imu_drain_t_ = t_frame;
  return out;
}

void OrbSlam3LifecycleNode::publishTracking(const Sophus::SE3f & Tcw, const rclcpp::Time & stamp)
{
  if (!active_.load() || !slam_) {
    return;
  }

  const int state = slam_->GetTrackingState();
  last_tracking_state_.store(state);

  // rate estimate (single-threaded executor: no lock needed)
  const rclcpp::Time now = this->now();
  if (last_track_time_.nanoseconds() > 0) {
    const double dt = (now - last_track_time_).seconds();
    if (dt > 1e-6) {
      track_rate_hz_ = 0.9 * track_rate_hz_ + 0.1 * (1.0 / dt);
    }
  }
  last_track_time_ = now;

  if (state != kOk) {
    return;  // don't emit TF/pose from a lost/uninitialized state
  }
  frames_tracked_.fetch_add(1);

  // System::Track* returns Tcw (world->camera); publish the camera-in-world pose.
  const Sophus::SE3f Twc = Tcw.inverse();

  if (publish_tf_) {
    Eigen::Isometry3d iso = Eigen::Isometry3d::Identity();
    iso.linear() = Twc.rotationMatrix().cast<double>();
    iso.translation() = Twc.translation().cast<double>();
    geometry_msgs::msg::TransformStamped tf = tf2::eigenToTransform(iso);
    tf.header.stamp = stamp;
    tf.header.frame_id = world_frame_id_;
    tf.child_frame_id = camera_frame_id_;
    tf_broadcaster_->sendTransform(tf);
  }

  if (publish_pose_ || publish_path_ || publish_odom_) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = world_frame_id_;
    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Quaternionf q = Twc.unit_quaternion();
    ps.pose.position.x = t.x();
    ps.pose.position.y = t.y();
    ps.pose.position.z = t.z();
    ps.pose.orientation.x = q.x();
    ps.pose.orientation.y = q.y();
    ps.pose.orientation.z = q.z();
    ps.pose.orientation.w = q.w();
    if (publish_pose_) {
      pose_pub_->publish(ps);
    }
    if (publish_path_) {
      path_.header.stamp = stamp;
      path_.poses.push_back(ps);
      path_pub_->publish(path_);
    }
    if (publish_odom_) {
      nav_msgs::msg::Odometry odom;
      odom.header = ps.header;                 // world_frame_id_ + source-image stamp
      odom.child_frame_id = odom_child_frame_id_;
      odom.pose.pose = ps.pose;
      odom.pose.covariance = computePoseCovariance(Twc);
      // ORB-SLAM3 has no velocity estimate in non-inertial modes; leave twist zero
      // and flag it unknown (large variance) so a fusion filter ignores it.
      for (size_t i = 0; i < 6; ++i) {odom.twist.covariance[i * 6 + i] = 1e6;}
      odom_pub_->publish(odom);
    }
  }

  if (publish_pointcloud_ && cloud_pub_->get_subscription_count() > 0) {
    const std::vector<ORB_SLAM3::MapPoint *> mps = slam_->GetTrackedMapPoints();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = world_frame_id_;
    cloud.height = 1;
    cloud.is_dense = false;
    cloud.is_bigendian = false;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(mps.size());
    sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x"), iy(cloud, "y"), iz(cloud, "z");
    size_t n = 0;
    for (ORB_SLAM3::MapPoint * mp : mps) {
      if (mp == nullptr || mp->isBad()) {continue;}
      const Eigen::Vector3f p = mp->GetWorldPos();
      *ix = p.x(); *iy = p.y(); *iz = p.z();
      ++ix; ++iy; ++iz; ++n;
    }
    mod.resize(n);
    cloud_pub_->publish(cloud);
  }
}

std::array<double, 36> OrbSlam3LifecycleNode::computePoseCovariance(const Sophus::SE3f & Twc)
{
  std::array<double, 36> cov{};  // zero-initialized row-major 6x6

  if (covariance_mode_ == CovarianceMode::kG2o) {
    bool valid = false;
    const Eigen::Matrix<double, 6, 6> Sigma_cw = slam_->GetTrackedPoseCovariance(valid);
    if (valid) {
      // Sigma_cw = covariance of a left-perturbation of Tcw, in g2o's SE3 tangent
      // ordering [omega(rot); upsilon(trans)]. A left-pert of Tcw is a body(right)
      // pert of Twc (same covariance), so map it to a world(left) pert of Twc with
      // the SE3 adjoint of Twc:  Ad = [[R, 0], [ [t]x R, R ]] (rotation-first),
      // then reorder to ROS [trans, rot] and apply the empirical scale.
      const Eigen::Matrix3d R = Twc.rotationMatrix().cast<double>();
      const Eigen::Vector3d p = Twc.translation().cast<double>();
      Eigen::Matrix<double, 6, 6> Ad = Eigen::Matrix<double, 6, 6>::Zero();
      Ad.topLeftCorner<3, 3>() = R;
      Ad.bottomRightCorner<3, 3>() = R;
      if (cov_g2o_lever_arm_) {
        Eigen::Matrix3d tx;
        tx <<      0.0, -p.z(),  p.y(),
              p.z(),      0.0, -p.x(),
             -p.y(),  p.x(),      0.0;
        Ad.bottomLeftCorner<3, 3>() = tx * R;   // lever-arm coupling rot -> trans
      }
      const Eigen::Matrix<double, 6, 6> Sw = Ad * Sigma_cw * Ad.transpose();  // [rot,trans]
      Eigen::Matrix<double, 6, 6> Sr;                                          // ROS [trans,rot]
      Sr.topLeftCorner<3, 3>() = Sw.bottomRightCorner<3, 3>();      // trans-trans
      Sr.bottomRightCorner<3, 3>() = Sw.topLeftCorner<3, 3>();      // rot-rot
      Sr.topRightCorner<3, 3>() = Sw.bottomLeftCorner<3, 3>();      // trans-rot
      Sr.bottomLeftCorner<3, 3>() = Sw.topRightCorner<3, 3>();      // rot-trans
      Sr *= cov_g2o_scale_;
      for (int r = 0; r < 6; ++r) {
        for (int c = 0; c < 6; ++c) {cov[r * 6 + c] = Sr(r, c);}
      }
      last_cov_scale_ = 1.0;  // scale is carried inside the marginal
      return cov;
    }
    // no marginal this frame (e.g. IMU-dominated) -> fall through to quality
  }

  // static / quality (and g2o fallback): diagonal from the base variances.
  double scale = 1.0;  // multiplies sigma; variance scales by scale^2
  if (covariance_mode_ != CovarianceMode::kStatic) {
    const int inliers = slam_->GetTrackedInliers();
    const double ratio = static_cast<double>(cov_inlier_ref_) / std::max(1, inliers);
    scale = std::clamp(ratio, 1.0, cov_max_scale_);
    // Only reached if ~/odom is ever emitted outside OK tracking; harmless today
    // (publishTracking returns early unless state == OK) but keeps intent explicit.
    if (last_tracking_state_.load() == 3 /* RECENTLY_LOST */) {
      scale *= cov_recently_lost_scale_;
    }
  }
  last_cov_scale_ = scale;
  const double var_scale = scale * scale;
  for (size_t i = 0; i < 6; ++i) {
    cov[i * 6 + i] = pose_cov_diagonal_[i] * var_scale;
  }
  return cov;
}

void OrbSlam3LifecycleNode::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  const int s = last_tracking_state_.load();
  const char * name = "UNKNOWN";
  switch (s) {
    case -1: name = "SYSTEM_NOT_READY"; break;
    case 0: name = "NO_IMAGES_YET"; break;
    case 1: name = "NOT_INITIALIZED"; break;
    case 2: name = "OK"; break;
    case 3: name = "RECENTLY_LOST"; break;
    case 4: name = "LOST"; break;
  }
  if (!active_.load()) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "not active");
  } else if (s == kOk) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "tracking OK");
  } else if (s == 3 || s == 1 || s == 0) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, name);
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, name);
  }
  stat.add("tracking_state", name);
  stat.add("frames_tracked", static_cast<int>(frames_tracked_.load()));
  stat.add("track_rate_hz", track_rate_hz_);
  stat.add("sensor_type", static_cast<int>(sensor_type_));
  if (slam_) {
    stat.add("track_inliers", slam_->GetTrackedInliers());
  }
  const char * cov_name = "quality";
  if (covariance_mode_ == CovarianceMode::kStatic) {cov_name = "static";}
  else if (covariance_mode_ == CovarianceMode::kG2o) {cov_name = "g2o";}
  stat.add("covariance_mode", cov_name);
  stat.add("covariance_scale", last_cov_scale_);
  {
    std::lock_guard<std::mutex> lk(imu_mutex_);
    stat.add("imu_buffer", static_cast<int>(imu_buffer_.size()));
  }
}

}  // namespace orb_slam3_ros
