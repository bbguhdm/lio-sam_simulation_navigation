#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pointcloud_downsampling
{
class PointcloudDownsampling : public rclcpp::Node
{
public:
  explicit PointcloudDownsampling(const rclcpp::NodeOptions & options);

  ~PointcloudDownsampling() override;

private:
  void getParams();

  void voxelFiltering(const sensor_msgs::msg::PointCloud2::UniquePtr msg);

  void publishResult(sensor_msgs::msg::PointCloud2 msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};
}  // namespace pointcloud_downsampling
