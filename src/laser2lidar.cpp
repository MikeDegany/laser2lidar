#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>

class Laser2LidarNode : public rclcpp::Node {
public:
  Laser2LidarNode() : Node("laser2lidar_node") {
    // Declare parameters without hard-coded topic names.
    // (The empty string tells us “nothing was provided”.)
    this->declare_parameter<std::string>("input_topic", "");
    this->declare_parameter<std::string>("output_topic", "");

    std::string input_topic, output_topic;
    this->get_parameter("input_topic", input_topic);
    this->get_parameter("output_topic", output_topic);

    // If the parameters are empty, something’s wrong:
    if (input_topic.empty() || output_topic.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Parameters 'input_topic' and 'output_topic' must be set (via a config file or command-line override)!");
      rclcpp::shutdown();
      return;
    }

    // Create subscriber and publisher using the parameters.
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      input_topic, 10,
      std::bind(&Laser2LidarNode::scanCallback, this, std::placeholders::_1));
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    projector_.projectLaser(*scan_msg, cloud_msg);
    point_cloud_pub_->publish(cloud_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  laser_geometry::LaserProjection projector_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Laser2LidarNode>());
  rclcpp::shutdown();
  return 0;
}

