#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <memory>
#include <vector>
#include <cmath>
#include <pluginlib/class_loader.hpp>
#include <logging.hpp>
#include "VDAMissionClient/Vda5050ActionHandlerBase.hpp"
#include "vda5050_msgs/msg/error.hpp"
#include "vda5050_msgs/msg/order.hpp"
#include "vda5050_msgs/msg/action.hpp"
#include "vda5050_msgs/msg/action_parameter.hpp"
#include "vda5050_msgs/msg/agv_state.hpp"
#include "vda5050_msgs/msg/action_state.hpp"
#include "vda5050_msgs/msg/node_state.hpp"
#include "vda5050_msgs/msg/node.hpp"
#include "vda5050_msgs/msg/edge_state.hpp"
#include "vda5050_msgs/msg/instant_actions.hpp"
#include "vda5050_msgs/msg/factsheet.hpp"
class Vda5050ActionHandlerBase;
class VDAMissionClient : public rclcpp::Node
{
public:
    using NavThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavThroughPoses = rclcpp_action::ClientGoalHandle<NavThroughPoses>;
    using NavClient = rclcpp_action::Client<NavThroughPoses>;
    using NavCancelResponse = NavClient::CancelResponse;

    using VDAActionState = vda5050_msgs::msg::ActionState;
    enum ErrorLevel
    {
        WARNING,
        FATAL
    };
    enum ClientState
    {
        IDLE,
        RUNNING,
        PAUSED
    };

    /* public function*/
public:
    VDAMissionClient();
    ~VDAMissionClient();
    bool RunThroughPoses(const std::vector<geometry_msgs::msg::PoseStamped> &poses);
    void UpdateActionState(
        const vda5050_msgs::msg::Action &action,
        const std::string &status,
        const std::string &result_description = "");
    void UpdateActionStateById(
        const std::string &action_id,
        const std::string &status,
        const std::string &result_description = "");
    template <typename ActionType>
    void ActionResponseCallback(
        const typename rclcpp_action::ClientGoalHandle<ActionType>::SharedPtr &goal,
        const vda5050_msgs::msg::Action &action);
    template <typename ResultType>
    void ActionResultCallback(
        const vda5050_msgs::msg::Action &action,
        const ResultType &result,
        const bool &success,
        const std::string &description);
    void AddError(const vda5050_msgs::msg::Error &error);

    /*public parameters*/
public:
    vda5050_msgs::msg::Error CreateError(
        ErrorLevel level, const std::string &error_msg,
        const std::vector<std::pair<std::string, std::string>> &error_refs,
        const std::string &error_type = "");

    /* private parameters */
private:
    // Node parameters
    // The period to update the robot state and mission status messages (in seconds)
    double update_feedback_period_{};
    // The topic to get robot velocity
    std::string odom_topic_ = "odom";
    // The topic to get robot battery state
    std::string battery_state_topic_;
    std::string robot_type_;
    std::string status_check_service_;
    std::string config_file_;
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr status_check_client_;
    // Action clients
    rclcpp_action::Client<NavThroughPoses>::SharedPtr client_ptr_;
    GoalHandleNavThroughPoses::SharedPtr nav_goal_handle_;
    rclcpp_action::Client<NavThroughPoses>::SendGoalOptions send_goal_options;
    // Mutex to protect private member variables when they are read or written to
    std::mutex state_mutex_;

    vda5050_msgs::msg::Order::ConstSharedPtr current_order_;
    // Order information for feedback of the mission
    vda5050_msgs::msg::AGVState::SharedPtr agv_state_;
    // Factsheet information for robot
    vda5050_msgs::msg::Factsheet::SharedPtr factsheet_;
    // Number of actions in the current order
    size_t num_actions_;
    // Cancel action
    vda5050_msgs::msg::Action::SharedPtr cancel_action_;
    // Pause action
    vda5050_msgs::msg::Action::SharedPtr pause_action_;
    // Action ids that action handler Cancel() is called
    // This is used to avoid calling Cancel() multiple times for the same action
    std::set<std::string> canceled_action_ids_;
    // Reached current waypoint flag
    bool reached_waypoint_;
    // Pause order
    bool pause_order_;
    std::string pause_order_action_id_ = "";
    // Current node the robot is working on
    size_t current_node_{};
    // Then last node of navigate_through_poses action
    size_t next_stop_{};
    // Current action the robot is working on
    size_t current_node_action_{};
    // Current action state to update
    size_t current_action_state_{};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    ClientState client_state_;

    std::unordered_map<std::string, std::shared_ptr<Vda5050ActionHandlerBase>> action_handler_map_;
    pluginlib::ClassLoader<Vda5050ActionHandlerBase> action_handler_loader_;

private:
    // Publish a vda5050_msgs/AGVState based on the current state of the robot
    void PublishRobotState();
    // Publish a vda5050_msgs/Factsheet based on the robot's sepcifications
    void PublishRobotFactsheet();
    // Timer callback function to publish a vda5050_msgs/AGVState message
    void StateTimerCallback();
    // The callback function when the node receives a vda5050_msgs/Order message and processes it
    void OrderCallback(const vda5050_msgs::msg::Order::ConstSharedPtr msg);
    // Execute order callback
    void ExecuteOrderCallback();
    // Function that creates the NavigateThroughPoses goal message for Nav2 and sends that goal
    // asynchronously
    void NavigateThroughPoses();
    // Vda5050 action handler: check actions in the current node and send requests to trigger
    // different servers based on the action type
    void ExecuteAction(const vda5050_msgs::msg::Action &vda5050_action);
    // Initialization the order state once received a new order
    void InitAGVState();
    // The callback function when the node receives a sensor_msgs/BatteryState message and processes
    // it into a VDA5050 BatteryState message
    void BatteryStateCallback(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg);
    // The callback function when the node receives a nav_msgs/Odometry message and appends it's
    // velotity to the status message's velocity that gets published
    void OdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    // The callback function when the node receives a std_msgs/String info message and appends it to
    // the status message that gets published
    void InfoCallback(const std_msgs::msg::String::ConstSharedPtr msg);
    // Timer callback function to publish a std_msgs/String message containing the order_id
    void OrderIdCallback();
    // Goal response callback for NavigateThroughPoses goal message
    void NavGoalResponseCallback(
        const rclcpp_action::ClientGoalHandle<NavThroughPoses>::SharedPtr &goal);
    // Feedback callback for NavigateThroughPoses goal message
    void NavFeedbackCallback(
        GoalHandleNavThroughPoses::SharedPtr,
        const NavThroughPoses::Feedback::ConstSharedPtr);
    // Result callback for NavigateThroughPoses goal message
    void NavResultCallback(const GoalHandleNavThroughPoses::WrappedResult &result);

    void CancelOrder();
    void InstantActionsCallback(const vda5050_msgs::msg::InstantActions::ConstSharedPtr msg);
    // Handle teleop instant actions
    void TeleopActionHandler(const vda5050_msgs::msg::Action &teleop_action);
    // Handle factsheet instant actions
    void FactsheetRequestHandler(const vda5050_msgs::msg::Action &factsheet_request);
    // The callback function when the node receives an order error message.
    void OrderValidErrorCallback(const std_msgs::msg::String::ConstSharedPtr msg);
    // Sync service request. Return request result
    template <typename ServiceT>
    typename ServiceT::Response::SharedPtr SendServiceRequest(
        typename rclcpp::Client<ServiceT>::SharedPtr client,
        typename ServiceT::Request::SharedPtr request);

    std::string GetActionState(const std::string &action_id);
    const vda5050_msgs::msg::Action &GetAction(const std::string &action_id);

    vda5050_msgs::msg::ErrorReference CreateErrorReference(
        const std::string &reference_key, const std::string &reference_value);

    bool CanAcceptOrder();

    void PauseOrder();
    void ResumeOrder(const vda5050_msgs::msg::Action &action);
};

extern "C"
{
    RCLCPP_EXPORT void InitEnviroment();
}