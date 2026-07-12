// Stereo (non-inertial) ORB-SLAM3 node. Approximate-time syncs a rectified
// left/right pair. Defaults target RealSense IR (infra1/infra2); works with
// OAK-D and LUCID stereo by remapping left_topic/right_topic. For RealSense,
// disable the IR projector (emitter_enabled:=false) so the pattern does not
// corrupt feature matching.
#include "orb_slam3_ros/orb_slam3_node.hpp"

#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include "orb_slam3_ros/mf_compat.hpp"

namespace orb_slam3_ros
{

class StereoNode : public OrbSlam3LifecycleNode
{
public:
  explicit StereoNode(const rclcpp::NodeOptions & options)
  : OrbSlam3LifecycleNode("orb_slam3_stereo", ORB_SLAM3::System::STEREO, options)
  {
    left_topic_ = declare_parameter<std::string>("left_topic", "/camera/infra1/image_rect_raw");
    right_topic_ = declare_parameter<std::string>("right_topic", "/camera/infra2/image_rect_raw");
    sync_queue_ = declare_parameter<int>("sync_queue_size", 30);
  }

protected:
  using ImageMsg = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

  void createSubscriptions() override
  {
    mfSubscribe(left_sub_, this, left_topic_, sensorQoS());
    mfSubscribe(right_sub_, this, right_topic_, sensorQoS());
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_), left_sub_, right_sub_);
    sync_->registerCallback(
      std::bind(&StereoNode::cb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "left: %s  right: %s", left_topic_.c_str(), right_topic_.c_str());
  }

  void destroySubscriptions() override
  {
    sync_.reset();
    left_sub_.unsubscribe();
    right_sub_.unsubscribe();
  }

private:
  void cb(const ImageMsg::ConstSharedPtr & left, const ImageMsg::ConstSharedPtr & right)
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
    const Sophus::SE3f Tcw = slam_->TrackStereo(gl, gr, stamp.seconds());
    publishTracking(Tcw, stamp);
  }

  std::string left_topic_, right_topic_;
  int sync_queue_{30};
  MfSubscriber<ImageMsg> left_sub_, right_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

}  // namespace orb_slam3_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(orb_slam3_ros::StereoNode)
