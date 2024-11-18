#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "cstring"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std::placeholders;

struct LivoxPointXYZITL {
    PCL_ADD_POINT4D;
    float intensity;
    uint8_t tag;
    uint8_t line;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPointXYZITL,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint8_t, tag, tag) (uint8_t, line, line) 
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

pcl::PointCloud<VelodynePointXYZIRT>::Ptr cloud (new pcl::PointCloud<VelodynePointXYZIRT>);
class LaserPublisher: public rclcpp::Node{
public:
    LaserPublisher():Node("laser_publisher_node_cpp"){
        point_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_out",10);
        point_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("/pointcloud",10,std::bind(&LaserPublisher::point_cb,this,_1));
    }
private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_pub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub;
    rclcpp::TimerBase::SharedPtr timer;
    double current_time = 0.0;
    void point_cb(const sensor_msgs::msg::PointCloud2::SharedPtr point){
        pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*point, *input_cloud);
        int num_lines=16;
        pcl::PointCloud<VelodynePointXYZIRT>::Ptr output_cloud(new pcl::PointCloud<VelodynePointXYZIRT>);
        for (const auto& pt : input_cloud->points)
        {
            VelodynePointXYZIRT pt_rt;
            pt_rt.x = pt.x;
            pt_rt.y = pt.y;
            pt_rt.z = pt.z;
            pt_rt.intensity = pt.intensity;
            double vertical_angle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y));
            pt_rt.ring = static_cast<int>((vertical_angle + M_PI / 2) / M_PI * num_lines);
            output_cloud->points.push_back(pt_rt);
        }
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*output_cloud, output_msg);
        output_msg.header.frame_id=point->header.frame_id;
        output_msg.header.stamp = point->header.stamp;
        point_pub->publish(output_msg);
    }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<LaserPublisher>());
    rclcpp::shutdown();
    return 0;
}