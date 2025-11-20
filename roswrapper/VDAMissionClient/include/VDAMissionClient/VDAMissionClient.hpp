#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <memory>
#include <vector>
#include <cmath>
#include <vda5050_msgs/msg/agv_state.h>
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using GoalHandleNavThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

class VDAMissionClient : public rclcpp::Node
{
public:
    VDAMissionClient();
    ~VDAMissionClient();

public:
    bool RunThroughPoses(const std::vector<geometry_msgs::msg::PoseStamped> &poses);
    
private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr _client;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
};

extern "C"
{
    RCLCPP_EXPORT void InitEnviroment();
    RCLCPP_EXPORT bool RunThroughPoses(VDAMissionClient *nodePtr, const double *xs,
                                             const double *ys,
                                             int32_t n,
                                             double theta_final_rad,
                                             const char *order_id);
    RCLCPP_EXPORT void SpinNode(VDAMissionClient *node);
    RCLCPP_EXPORT VDAMissionClient* CreateVDANode();
    
}