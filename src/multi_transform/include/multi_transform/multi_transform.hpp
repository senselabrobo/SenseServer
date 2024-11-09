#ifndef MULTI_TRANSFORM
#define MULTI_TRANSFORM

#include <arpa/inet.h>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>

#include <netinet/in.h>
#include <queue>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <tf2/LinearMath/Transform.h>
#include <thread>

#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#define MAX_ROBOT_COUNT 5

namespace multi_transform
{
class MultiTransformNode : public rclcpp::Node
{
public:
  MultiTransformNode(const rclcpp::NodeOptions & options);
  ~MultiTransformNode() override;

private:
  std::thread send_thread_;
  std::thread recv_thread_[MAX_ROBOT_COUNT];

  struct send_buffer{
    int id;
    std::vector<uint8_t> buffer;
  };
  std::queue<send_buffer> send_buffer_queue;

  void NetworkSendThread();
  void NetworkRecvThread(const int robot_id);

  void WayPoint0CallBack(const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg);
  void WayPoint1CallBack(const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg);
  void WayPoint2CallBack(const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg);
  void WayPoint3CallBack(const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg);
  void WayPoint4CallBack(const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg);

  void SendData(const std::vector<uint8_t> & data_buffer, const int robot_id, const int msg_type);

  template <class T> std::vector<uint8_t> SerializeMsg(const T &msg);
  template <class T> T DeserializeMsg(const std::vector<uint8_t> &data);

  int port;
  std::string ip;
  int sockfd;
  struct sockaddr_in server_addr, client_addr, saved_client_addr[MAX_ROBOT_COUNT];

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_pub_[MAX_ROBOT_COUNT];
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr realsense_image_pub_[MAX_ROBOT_COUNT];

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr way_point_sub_[MAX_ROBOT_COUNT];
};
} // namespace multi_transform

#endif