#include <arpa/inet.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <netinet/in.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/utilities.hpp>
#include <rmw/qos_profiles.h>
#include <rmw/types.h>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <string>
#include <sys/socket.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <vector>

#include "multi_transform/multi_transform.hpp"

#define MAX_PACKET_SIZE 64000
#define BUFFER_SIZE 65535

namespace multi_transform
{
MultiTransformNode::MultiTransformNode(const rclcpp::NodeOptions & options)
  : Node("multi_transform", options)
{
  this->declare_parameter<int>("network_port", 12130);
  this->declare_parameter<std::string>("network_ip", "192.168.31.207");

  this->get_parameter("network_port", port);
  this->get_parameter("network_ip", ip);

  for (int i = 0; i < MAX_ROBOT_COUNT; i++) {
    registered_scan_pub_[i] = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/robot_" + std::to_string(i) + "/total_registered_scan", 5);
    realsense_image_pub_[i] = this->create_publisher<sensor_msgs::msg::Image>(
      "/robot_" + std::to_string(i) + "/image_raw", 5);
  }

  way_point_sub_[0] = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/robot_0/way_point",
    2,
    std::bind(&MultiTransformNode::WayPoint0CallBack, this, std::placeholders::_1));
  way_point_sub_[1] = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/robot_1/way_point",
    2,
    std::bind(&MultiTransformNode::WayPoint1CallBack, this, std::placeholders::_1));
  way_point_sub_[2] = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/robot_2/way_point",
    2,
    std::bind(&MultiTransformNode::WayPoint2CallBack, this, std::placeholders::_1));
  way_point_sub_[3] = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/robot_3/way_point",
    2,
    std::bind(&MultiTransformNode::WayPoint3CallBack, this, std::placeholders::_1));
  way_point_sub_[4] = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/robot_4/way_point",
    2,
    std::bind(&MultiTransformNode::WayPoint4CallBack, this, std::placeholders::_1));

  // UDP
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Socket creation failed!");
    return;
  }

  memset(&server_addr, 0, sizeof(server_addr));
  memset(&client_addr, 0, sizeof(client_addr));
  memset(&saved_client_addr, 0, sizeof(saved_client_addr));

  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = inet_addr(ip.c_str());

  if (bind(sockfd, (const struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Bind failed!");
    close(sockfd);
    return;
  }

  send_thread_ = std::thread(&MultiTransformNode::NetworkSendThread, this);
  for (int i = 0; i < MAX_ROBOT_COUNT; i++){
    recv_thread_[i] = std::thread(&MultiTransformNode::NetworkRecvThread, this, i);
  }
  RCLCPP_INFO(this->get_logger(), "Server start at ip: %s, port: %d", ip.c_str(), port);
}

MultiTransformNode::~MultiTransformNode()
{
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
  for (int i = 0; i < MAX_ROBOT_COUNT; i++){
    if (recv_thread_[i].joinable()) {
      recv_thread_[i].join();
    }
  }
  close(sockfd);
}

void MultiTransformNode::NetworkSendThread()
{
  while (rclcpp::ok()) {
    rclcpp::sleep_for(std::chrono::nanoseconds(10));
    if (!send_buffer_queue.empty()) {
      send_buffer s_buffer = send_buffer_queue.front();
      send_buffer_queue.pop();
      sendto(sockfd,
             s_buffer.buffer.data(),
             s_buffer.buffer.size(),
             MSG_CONFIRM,
             (const struct sockaddr *)&saved_client_addr[s_buffer.id],
             sizeof(saved_client_addr[s_buffer.id]));
    }
  }
}

void MultiTransformNode::NetworkRecvThread(const int robot_id)
{
  int n, len = sizeof(client_addr);
  int packet_idx = 0;
  int packet_type = -1;
  std::vector<uint8_t> buffer;
  while (rclcpp::ok()) {
    std::vector<uint8_t> buffer_tmp(BUFFER_SIZE);
    n = recvfrom(sockfd,
                 buffer_tmp.data(),
                 BUFFER_SIZE,
                 MSG_WAITALL,
                 (struct sockaddr *)&client_addr,
                 (socklen_t *)&len);
    if (n < 0) {
      continue;
    }
    buffer_tmp.resize(n);
    uint8_t id, type, max_idx;
    uint16_t idx;
    std::memcpy(&id, buffer_tmp.data(), sizeof(id));
    std::memcpy(&type, buffer_tmp.data() + sizeof(uint8_t), sizeof(type));
    std::memcpy(&idx, buffer_tmp.data() + sizeof(uint16_t), sizeof(idx));
    std::memcpy(&max_idx, buffer_tmp.data() + sizeof(uint32_t), sizeof(max_idx));

    // register client
    saved_client_addr[id] = client_addr;

    if (packet_type == -1) {
      packet_type = type;
    } else if (packet_type < type) {
      continue;
    } else if (packet_type > type) {
      packet_type = type;
      packet_idx = 0;
      buffer = std::vector<uint8_t>(0);
    }
    if (idx == 0) {
      packet_idx = 0;
      buffer = std::vector<uint8_t>(0);
    } else if (packet_idx != idx) {
      packet_idx = 0;
      packet_type = -1;
      buffer = std::vector<uint8_t>(0);
      continue;
    }

    packet_idx++;
    if (packet_idx == 1) {
      buffer.insert(buffer.begin(),
                        buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t),
                        buffer_tmp.end());
    } else {
      buffer.insert(buffer.end(),
                        buffer_tmp.begin() + sizeof(uint32_t) + sizeof(uint8_t),
                        buffer_tmp.end());
    }

    if (packet_idx != max_idx) {
      continue;
    }
    try {
      if (type == 0) { // PointCloud2
        std::shared_ptr<sensor_msgs::msg::PointCloud2> totalRegisteredScan =
          std::make_shared<sensor_msgs::msg::PointCloud2>(
            MultiTransformNode::DeserializeMsg<sensor_msgs::msg::PointCloud2>(buffer));
        registered_scan_pub_[id]->publish(*totalRegisteredScan);
      } else if (type == 1) { // Image
        std::shared_ptr<sensor_msgs::msg::Image> realsense_image =
          std::make_shared<sensor_msgs::msg::Image>(
            MultiTransformNode::DeserializeMsg<sensor_msgs::msg::Image>(buffer));
        realsense_image_pub_[id]->publish(*realsense_image);
      } else if (type == 2) { // Transform
        std::shared_ptr<geometry_msgs::msg::TransformStamped> transformStamped =
          std::make_shared<geometry_msgs::msg::TransformStamped>(
            MultiTransformNode::DeserializeMsg<geometry_msgs::msg::TransformStamped>(buffer));
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ =
          std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tf_broadcaster_->sendTransform(*transformStamped);
      }
    } catch (...) {
    }

    packet_idx = 0;
    packet_type = -1;
    buffer = std::vector<uint8_t>(0);
  }
}

void MultiTransformNode::WayPoint0CallBack(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg)
{
  std::vector<uint8_t> data_buffer =
    MultiTransformNode::SerializeMsg<geometry_msgs::msg::PointStamped>(*way_point_msg);
  MultiTransformNode::SendData(data_buffer, 0, 0);
}

void MultiTransformNode::WayPoint1CallBack(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg)
{
  std::vector<uint8_t> data_buffer =
    MultiTransformNode::SerializeMsg<geometry_msgs::msg::PointStamped>(*way_point_msg);
  MultiTransformNode::SendData(data_buffer, 1, 0);
}

void MultiTransformNode::WayPoint2CallBack(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg)
{
  std::vector<uint8_t> data_buffer =
    MultiTransformNode::SerializeMsg<geometry_msgs::msg::PointStamped>(*way_point_msg);
  MultiTransformNode::SendData(data_buffer, 2, 0);
}

void MultiTransformNode::WayPoint3CallBack(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg)
{
  std::vector<uint8_t> data_buffer =
    MultiTransformNode::SerializeMsg<geometry_msgs::msg::PointStamped>(*way_point_msg);
  MultiTransformNode::SendData(data_buffer, 3, 0);
}

void MultiTransformNode::WayPoint4CallBack(
  const geometry_msgs::msg::PointStamped::ConstSharedPtr way_point_msg)
{
  std::vector<uint8_t> data_buffer =
    MultiTransformNode::SerializeMsg<geometry_msgs::msg::PointStamped>(*way_point_msg);
  MultiTransformNode::SendData(data_buffer, 4, 0);
}

void MultiTransformNode::SendData(const std::vector<uint8_t> & data_buffer,
                                  const int robot_id,
                                  const int msg_type)
{
  const int total_packet = (data_buffer.size() + MAX_PACKET_SIZE - 1) / MAX_PACKET_SIZE;
  for (int i = 0; i < total_packet; i++) {
    uint8_t id = robot_id;
    uint8_t type = msg_type;
    uint16_t idx = i;
    uint8_t max_idx = total_packet;
    std::vector<uint8_t> header(sizeof(uint32_t) + sizeof(uint8_t));
    std::memcpy(header.data(), &id, sizeof(id));
    std::memcpy(header.data() + sizeof(uint8_t), &type, sizeof(type));
    std::memcpy(header.data() + sizeof(uint16_t), &idx, sizeof(idx));
    std::memcpy(header.data() + sizeof(uint32_t), &max_idx, sizeof(max_idx));
    std::vector<uint8_t> packet;
    packet.insert(packet.end(), header.begin(), header.end());
    packet.insert(packet.end(),
                  data_buffer.begin() + i * MAX_PACKET_SIZE,
                  i == total_packet - 1 ? data_buffer.end()
                                        : data_buffer.begin() + (i + 1) * MAX_PACKET_SIZE);
    send_buffer s_buffer;
    s_buffer.buffer = packet;
    s_buffer.id = robot_id;
    send_buffer_queue.push(s_buffer);
  }
}

// Serialization
template <class T> std::vector<uint8_t> MultiTransformNode::SerializeMsg(const T & msg)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<T> serializer;
  serializer.serialize_message(&msg, &serialized_msg);

  std::vector<uint8_t> buffer_tmp(serialized_msg.size());
  std::memcpy(
    buffer_tmp.data(), serialized_msg.get_rcl_serialized_message().buffer, serialized_msg.size());

  return buffer_tmp;
}

// Deserialization
template <class T> T MultiTransformNode::DeserializeMsg(const std::vector<uint8_t> & data)
{
  rclcpp::SerializedMessage serialized_msg;
  rclcpp::Serialization<T> serializer;

  serialized_msg.reserve(data.size());
  std::memcpy(serialized_msg.get_rcl_serialized_message().buffer, data.data(), data.size());
  serialized_msg.get_rcl_serialized_message().buffer_length = data.size();

  T msg;
  serializer.deserialize_message(&serialized_msg, &msg);

  return msg;
}
} // namespace multi_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(multi_transform::MultiTransformNode)