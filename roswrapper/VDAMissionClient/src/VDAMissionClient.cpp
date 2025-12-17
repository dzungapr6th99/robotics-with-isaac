
#include "VDAMissionClient/VDAMissionClient.hpp"

VDAMissionClient::VDAMissionClient() : Node("vda_miss_client_node")
{
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS(),
                                                                       std::bind(&VDAMissionClient::OdometryCallback, this,
                                                                                 std::placeholders::_1));
    client_ptr_ = rclcpp_action::create_client<NavThroughPoses>(this, "navigate_through_poses");
    agv_state_ = std::make_shared<vda5050_msgs::msg::AGVState>();
}

extern "C"
{
    RCLCPP_EXPORT void InitEnviroment()
    {
        rclcpp::init(0, nullptr);
    }
}