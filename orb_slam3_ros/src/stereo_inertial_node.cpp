// Stereo-inertial (VIO) ORB-SLAM3 node. Approximate-time syncs a rectified
// left/right pair and buffers a single combined sensor_msgs/Imu stream, passing
// the IMU samples that precede each frame into ORB-SLAM3.
//
// RealSense D435i: set unite_imu_method:=linear_interpolation (or copy) so the
// driver publishes a combined /camera/imu; enable_gyro/enable_accel:=true.
// OAK-D: depthai driver publishes a combined /imu. Remap imu_topic accordingly.
#include "orb_slam3_ros/orb_slam3_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

namespace orb_slam3_ros
{

class StereoInertialNode : public OrbSlam3LifecycleNode
{
public:
  explicit StereoInertialNode(const rclcpp::NodeOptions & options)
  : OrbSlam3LifecycleNode("orb_slam3_stereo_inertial", ORB_SLAM3::System::IMU_STEREO, options)
  {
    left_topic_ = declare_parameter<std::string>("left_topic", "/camera/infra1/image_rect_raw");
    right_topic_ = declare_parameter<std::string>("right_topic", "/camera/infra2/image_rect_raw");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/camera/imu");
    sync_queue_ = declare_parameter<int>("sync_queue_size", 30);
  }

protected:
  using ImageMsg = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

  void createSubscriptions() override
  {
    left_sub_.subscribe(this, left_topic_, sensorQoS());
    right_sub_.subscribe(this, right_topic_, sensorQoS());
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_), left_sub_, right_sub_);
    sync_->registerCallback(
      std::bind(&StereoInertialNode::stereoCb, this, std::placeholders::_1, std::placeholders::_2));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensorQoS(),
      std::bind(&StereoInertialNode::imuCb, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "left: %s  right: %s  imu: %s",
      left_topic_.c_str(), right_topic_.c_str(), imu_topic_.c_str());
  }

  void destroySubscriptions() override
  {
    sync_.reset();
    left_sub_.unsubscribe();
    right_sub_.unsubscribe();
    imu_sub_.reset();
  }

private:
  void imuCb(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {pushImu(*msg);}

  void stereoCb(const ImageMsg::ConstSharedPtr & left, const ImageMsg::ConstSharedPtr & right)
  {
    cv::Mat gl, gr;
    try {
      gl = toMono(left);
      gr = toMono(right);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }
    const rclcpp::Time stamp(left->header.stamp);
    std::vector<ORB_SLAM3::IMU::Point> imu = drainImu(stamp.seconds());
    const Sophus::SE3f Tcw = slam_->TrackStereo(gl, gr, stamp.seconds(), imu);
    publishTracking(Tcw, stamp);
  }

  std::string left_topic_, right_topic_, imu_topic_;
  int sync_queue_{30};
  message_filters::Subscriber<ImageMsg> left_sub_, right_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

}  // namespace orb_slam3_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(orb_slam3_ros::StereoInertialNode)
