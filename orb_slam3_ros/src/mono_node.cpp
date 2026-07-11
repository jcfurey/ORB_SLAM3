// Monocular ORB-SLAM3 node. Works with any driver publishing a single
// sensor_msgs/Image (RealSense color/IR, OAK-D RGB, LUCID Triton, ...).
#include "orb_slam3_ros/orb_slam3_node.hpp"

#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>

namespace orb_slam3_ros
{

class MonoNode : public OrbSlam3LifecycleNode
{
public:
  explicit MonoNode(const rclcpp::NodeOptions & options)
  : OrbSlam3LifecycleNode("orb_slam3_mono", ORB_SLAM3::System::MONOCULAR, options)
  {
    image_topic_ = declare_parameter<std::string>("image_topic", "/camera/image_raw");
  }

protected:
  void createSubscriptions() override
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, sensorQoS(),
      std::bind(&MonoNode::imageCb, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "image: %s", image_topic_.c_str());
  }

  void destroySubscriptions() override {sub_.reset();}

private:
  void imageCb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    cv::Mat gray;
    try {
      gray = toMono(msg);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }
    const rclcpp::Time stamp(msg->header.stamp);
    const Sophus::SE3f Tcw = slam_->TrackMonocular(gray, stamp.seconds());
    publishTracking(Tcw, stamp);
  }

  std::string image_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

}  // namespace orb_slam3_ros

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<orb_slam3_ros::MonoNode>(rclcpp::NodeOptions());
  exec.add_node(node->get_node_base_interface());
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
