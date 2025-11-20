
#include "VDAMissionClient/VDAMissionClient.hpp"

using GoalHandleNTP = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
VDAMissionClient::VDAMissionClient() : Node("vda_miss_client_node")
{
    _client = rclcpp_action::create_client<NavigateThroughPoses>(this, "/navigate_through_poses");
    //_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(this, "/odom", rclcpp::SensorDataQoS(), std::bind);
    
}

bool VDAMissionClient::RunThroughPoses(const std::vector<geometry_msgs::msg::PoseStamped> &poses)
{
    NavigateThroughPoses::Goal goalMsg;
    goalMsg.poses = poses;
    if (!_client->wait_for_action_server(std::chrono::seconds(10)))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server 'navigate_through_poses' not available after waiting");
        return false;
    }

    NavigateThroughPoses::Goal goal;
    goal.poses = poses; // chỉ cần mảng PoseStamped

    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions opts;
    opts.goal_response_callback =
        [this](std::shared_ptr<GoalHandleNavThroughPoses> handle)
    {
        if (!handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted");
        }
    };

    opts.feedback_callback =
        [this](std::shared_ptr<GoalHandleNavThroughPoses> /*handle*/,
               const std::shared_ptr<const NavigateThroughPoses::Feedback> /*fb*/)
    {
        // Có thể in thêm thông tin nếu muốn (tránh lệ thuộc trường cụ thể để code gọn/ổn định)
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Navigating through poses...");
    };

    opts.result_callback =
        [this](const GoalHandleNavThroughPoses::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Navigation succeeded");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Navigation aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Navigation canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
    };

    RCLCPP_INFO(get_logger(), "Sending goal with %zu poses...", poses.size());
    _client->async_send_goal(goal, opts);
    return true;
}

extern "C"
{
    RCLCPP_EXPORT void InitEnviroment()
    {
        rclcpp::init(0, nullptr);
    }

    RCLCPP_EXPORT VDAMissionClient *CreateVDANode()
    {
        return new VDAMissionClient();
    }

    RCLCPP_EXPORT void SpinNode(VDAMissionClient *node)
    {
        rclcpp::spin_some(node->get_node_base_interface());
    }
    RCLCPP_EXPORT bool RunThroughPoses(VDAMissionClient *nodePtr, const double *xs,
                                             const double *ys,
                                             int32_t n,
                                             double theta_final_rad,
                                             const char *order_id)
    {
        std::vector<geometry_msgs::msg::PoseStamped> poses;
        for (int i = 0; i < n; i++)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = *(xs + i);
            pose.pose.position.y = (*ys + i);
            if (i == n - 1)
            {
                pose.pose.position.z = theta_final_rad;
            }
            poses.push_back(pose);
        }
        return nodePtr->RunThroughPoses(poses); 
        
    }


}