// RGB-D ORB-SLAM3 node. Approximate-time syncs a colour/mono image with a depth
// image. Defaults target RealSense (aligned depth); works with OAK-D and LUCID
// Helios by remapping rgb_topic/depth_topic. Depth scale is handled by the
// DepthMapFactor in the ORB-SLAM3 settings .yaml.
#include "orb_slam3_ros/orb_slam3_node.hpp"

#include <memory>
#include <string>

#include "orb_slam3_ros/distro_compat.hpp"
#include "orb_slam3_ros/mf_compat.hpp"

namespace orb_slam3_ros
{

class RgbdNode : public OrbSlam3LifecycleNode
{
public:
  explicit RgbdNode(const rclcpp::NodeOptions & options)
  : OrbSlam3LifecycleNode("orb_slam3_rgbd", ORB_SLAM3::System::RGBD, options)
  {
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera/color/image_raw");
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/camera/aligned_depth_to_color/image_raw");
    sync_queue_ = declare_parameter<int>("sync_queue_size", 30);
  }

protected:
  using ImageMsg = sensor_msgs::msg::Image;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;

  void createSubscriptions() override
  {
    mfSubscribe(rgb_sub_, this, rgb_topic_, sensorQoS());
    mfSubscribe(depth_sub_, this, depth_topic_, sensorQoS());
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(sync_queue_), rgb_sub_, depth_sub_);
    sync_->registerCallback(
      std::bind(&RgbdNode::cb, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "rgb: %s  depth: %s", rgb_topic_.c_str(), depth_topic_.c_str());
  }

  void destroySubscriptions() override
  {
    sync_.reset();
    rgb_sub_.unsubscribe();
    depth_sub_.unsubscribe();
  }

private:
  void cb(const ImageMsg::ConstSharedPtr & rgb, const ImageMsg::ConstSharedPtr & depth)
  {
    cv::Mat gray, dep;
    try {
      gray = toMono(rgb);
      dep = toDepth(depth);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }
    const rclcpp::Time stamp(rgb->header.stamp);
    const Sophus::SE3f Tcw = slam_->TrackRGBD(gray, dep, stamp.seconds());
    publishTracking(Tcw, stamp);
  }

  std::string rgb_topic_, depth_topic_;
  int sync_queue_{30};
  MfSubscriber<ImageMsg> rgb_sub_, depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

}  // namespace orb_slam3_ros

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(orb_slam3_ros::RgbdNode)
