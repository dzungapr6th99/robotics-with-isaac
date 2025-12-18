
#include "VDAMissionClient/VDAMissionClient.hpp"

VDAMissionClient::VDAMissionClient() : Node("vda_miss_client_node")
{
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS(),
                                                                       std::bind(&VDAMissionClient::OdometryCallback, this,
                                                                                 std::placeholders::_1));
    client_ptr_ = rclcpp_action::create_client<NavThroughPoses>(this, "navigate_through_poses");
    agv_state_ = std::make_shared<vda5050_msgs::msg::AGVState>();
    send_goal_options = rclcpp_action::Client<NavThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(
            &VDAMissionClient::NavGoalResponseCallback, this,
            std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(
            &VDAMissionClient::NavFeedbackCallback, this,
            std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(
            &VDAMissionClient::NavResultCallback, this,
            std::placeholders::_1);
}

void VDAMissionClient::NavigateThroughPoses()
{

    RCLCPP_DEBUG(get_logger(), "Navigating through poses");
    if (status_check_service_ != "" && current_node_ == 0)
    {
        try
        {
            auto status_check_request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
            auto result = SendServiceRequest<lifecycle_msgs::srv::GetState>(
                status_check_client_, status_check_request);
            if (result->current_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
            {
                RCLCPP_ERROR(get_logger(), "The status check node is not active");
                // Reset states to retry in next cycle
                next_stop_ = current_node_;
                reached_waypoint_ = true;
                return;
            }
            else
            {
                RCLCPP_DEBUG(get_logger(), "The status check node is active");
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Failed to check status: %s", e.what());
            // Reset states to retry in next cycle
            next_stop_ = current_node_;
            reached_waypoint_ = true;
            return;
        }
    }
    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(3)))
    {
        RCLCPP_ERROR(this->get_logger(), "Navigation server not available");
        return;
    }
    if (!current_order_)
    {
        RCLCPP_ERROR(this->get_logger(), "Navigation was called when no order exists.");
    }
    if (next_stop_ >= current_order_->nodes.size())
    {
        RCLCPP_INFO(this->get_logger(), "Navigation completed");
        return;
    }
    auto goal_msg = NavThroughPoses::Goal();
    for (size_t i = current_node_ + 1; i < current_order_->nodes.size(); i++)
    {
        auto pose_stamped = geometry_msgs::msg::PoseStamped();

        pose_stamped.pose.position.x =
            current_order_->nodes[i].node_position.x;
        pose_stamped.pose.position.y =
            current_order_->nodes[i].node_position.y;
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.header.frame_id = "map";
        // Convert theta into a quaternion for goal pose's orientation
        tf2::Quaternion orientation;
        orientation.setRPY(
            0, 0,
            current_order_->nodes[i].node_position.theta);
        pose_stamped.pose.orientation = tf2::toMsg(orientation);
        pose_stamped.header.stamp = rclcpp::Clock().now();
        goal_msg.poses.push_back(pose_stamped);
        if (current_order_->nodes[i].actions.size() > 0 ||
            i == current_order_->nodes.size() - 1 ||
            current_order_->nodes[i].node_position.allowed_deviation_x_y == 0)
        {
            next_stop_ = i;
            break;
        }
    }
    RCLCPP_INFO(
        get_logger(), "Sending goal for (x: %f, y: %f, t: %f)",
        current_order_->nodes[next_stop_].node_position.x,
        current_order_->nodes[next_stop_].node_position.y,
        current_order_->nodes[next_stop_].node_position.theta);
    agv_state_->driving = true;
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void VDAMissionClient::ExecuteAction(const vda5050_msgs::msg::Action &vda5050_action)
{
    RCLCPP_DEBUG(get_logger(), "Executing action: %s", vda5050_action.action_id.c_str());
    UpdateActionState(vda5050_action, VDAActionState::RUNNING);
    if (vda5050_action.action_type == "pause_order")
    {
        // Set the pause_order to running untill stopTeleop action is triggered
        pause_order_ = true;
        UpdateActionStateById(vda5050_action.action_id, VDAActionState::RUNNING);
        pause_order_action_id_ = vda5050_action.action_id;
        return;
    }
    auto handler = action_handler_map_.find(vda5050_action.action_type);
    if (handler != action_handler_map_.end())
    {
        handler->second->Execute(vda5050_action);
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Action handler not found for action type: %s",
                     vda5050_action.action_type.c_str());
        UpdateActionState(vda5050_action, VDAActionState::FAILED, "Action handler not found");
    }
    // Continue process action here
}

bool VDAMissionClient::CanAcceptOrder()
{
    if (client_state_ == ClientState::IDLE)
    {
        return true;
    }
    else if (client_state_ == ClientState::PAUSED)
    {
        return false;
    }
    else if (client_state_ == ClientState::RUNNING)
    {
        // Check if there is a running action
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < num_actions_; i++)
        {
            if (agv_state_->action_states[i].action_status == VDAActionState::RUNNING ||
                agv_state_->action_states[i].action_status == VDAActionState::INITIALIZING)
            {
                return false;
            }
        }
        // No action is running.
        // Check if there is a fatal error
        if (agv_state_->errors.size() > 0 &&
            agv_state_->errors[0].error_level == vda5050_msgs::msg::Error::FATAL)
        {
            // Return false if there is a running action
            RCLCPP_INFO(get_logger(), "Order %s is completed with fatal error. Mission Client is IDLE.",
                        current_order_->order_id.c_str());
            client_state_ = ClientState::IDLE;
            return true;
        }
        // Check if the robot is at the last node and no waiting action
        if (current_node_ == current_order_->nodes.size() - 1)
        {
            for (size_t i = 0; i < num_actions_; i++)
            {
                if (agv_state_->action_states[i].action_status == VDAActionState::WAITING)
                {
                    return false;
                }
            }
            return true;
        }
        return false;
    }
    return false;
}

void VDAMissionClient::PauseOrder()
{
    RCLCPP_INFO(get_logger(), "Pausing order");
    static std::set<std::string> paused_action_ids;
    if (client_state_ == ClientState::IDLE)
    {
        RCLCPP_ERROR(get_logger(), "No active order to pause");
        UpdateActionState(*pause_action_, VDAActionState::FAILED, "No active order to pause");
        pause_action_.reset();
        return;
    }
    else if (client_state_ == ClientState::PAUSED)
    {
        RCLCPP_ERROR(get_logger(), "Order is already paused");
        UpdateActionState(*pause_action_, VDAActionState::FINISHED);
        pause_action_.reset();
        return;
    }
    const std::string pause_action_state = GetActionState(pause_action_->action_id);
    if (pause_action_state == VDAActionState::WAITING)
    {
        UpdateActionState(*pause_action_, VDAActionState::RUNNING);
        if (nav_goal_handle_)
        {
            client_ptr_->async_cancel_goal(nav_goal_handle_);
        }
    }
    // pause all actions in current order
    bool all_actions_paused = true;
    for (size_t i = 0; i < num_actions_; i++)
    {
        const std::string action_id = agv_state_->action_states[i].action_id;
        const std::string action_state = GetActionState(action_id);
        if (action_state == VDAActionState::RUNNING ||
            action_state == VDAActionState::INITIALIZING)
        {
            all_actions_paused = false;
            if (paused_action_ids.find(action_id) == paused_action_ids.end())
            {
                paused_action_ids.insert(action_id);
                auto handler = action_handler_map_[GetAction(action_id).action_type];
                handler->Pause(action_id);
            }
        }
    }
    if (all_actions_paused && nav_goal_handle_ == nullptr)
    {
        UpdateActionState(*pause_action_, VDAActionState::FINISHED);
        pause_action_.reset();
        paused_action_ids.clear();
        client_state_ = ClientState::PAUSED;
        RCLCPP_INFO(get_logger(), "Finished pauseOrder.");
    }
}

void VDAMissionClient::ResumeOrder(const vda5050_msgs::msg::Action &action)
{
    RCLCPP_INFO(get_logger(), "Resuming order");
    if (client_state_ != ClientState::PAUSED)
    {
        RCLCPP_ERROR(get_logger(), "Order is not paused");
        UpdateActionState(action, VDAActionState::FAILED, "Order is not paused");
        return;
    }
    UpdateActionState(action, VDAActionState::RUNNING);
    for (size_t i = 0; i < num_actions_; i++)
    {
        const std::string action_id = agv_state_->action_states[i].action_id;
        const std::string action_state = GetActionState(action_id);
        if (action_state == VDAActionState::PAUSED)
        {
            UpdateActionStateById(agv_state_->action_states[i].action_id, VDAActionState::RUNNING);
            const std::string action_type = GetAction(action_id).action_type;
            auto handler = action_handler_map_[action_type];
            handler->Resume(action_id);
        }
    }
    next_stop_ = current_node_;
    reached_waypoint_ = true;
    client_state_ = ClientState::RUNNING;
    UpdateActionState(action, VDAActionState::FINISHED);
}

extern "C"
{
    RCLCPP_EXPORT void InitEnviroment()
    {
        rclcpp::init(0, nullptr);
    }
}