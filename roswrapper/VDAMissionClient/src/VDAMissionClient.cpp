
#include "VDAMissionClient/VDAMissionClient.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <pluginlib/class_loader.hpp>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
VDAMissionClient::VDAMissionClient(std::string ns) : rclcpp::Node("vda_miss_client_node", ns),
                                                     action_handler_loader_("VDAMissionClient",
                                                                            "Vda5050ActionHandlerBase")
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, rclcpp::SensorDataQoS(),
                                                                       std::bind(&VDAMissionClient::OdometryCallback, this,
                                                                                 std::placeholders::_1));
    client_ptr_ = rclcpp_action::create_client<NavThroughPoses>(this, "navigate_through_poses");
    agv_state_ = std::make_shared<vda5050_msgs::msg::AGVState>();
    factsheet_ = std::make_shared<vda5050_msgs::msg::Factsheet>();
    visualization_ = std::make_shared<vda5050_msgs::msg::Visualization>();
    send_goal_options = rclcpp_action::Client<NavThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback = std::bind(&VDAMissionClient::NavGoalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&VDAMissionClient::NavFeedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&VDAMissionClient::NavResultCallback, this, std::placeholders::_1);
    handler_list = {
        {"lift", "VDAMissionClient/VDAActionAMRHandler"},
        {"dock", "VDAMissionClient/VDAActionAMRHandler"},
    };

    for (const auto &item : handler_list)
    {
        const std::string &action_type = item.first;
        const std::string &plugin = item.second;
        try
        {
            auto handler = action_handler_loader_.createSharedInstance(plugin);
            handler->Initialize(this);
            action_handler_map_[action_type] = handler;
        }
        catch (const pluginlib::PluginlibException &e)
        {
            RCLCPP_WARN(get_logger(), "Skip '%s' (%s): %s",
                        action_type.c_str(), plugin.c_str(), e.what());
        }
    }

    state_update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]()
        {
            (void)UpdateRobotPoseFromTf();
            if (client_state_ == ClientState::RUNNING)
            {
                ExecuteOrderCallback();
            }
        });
}
VDAMissionClient::~VDAMissionClient()
{
}
#pragma region Error Callback
std::vector<vda5050_msgs::msg::ErrorReference> VDAMissionClient::CreateErrorReferenceList(
    const std::vector<std::pair<std::string, std::string>> &error_refs)
{
    std::vector<vda5050_msgs::msg::ErrorReference> error_references;
    for (const auto &error_ref : error_refs)
    {
        error_references.push_back(CreateErrorReference(error_ref.first, error_ref.second));
    }
    return error_references;
}

void VDAMissionClient::AddError(const vda5050_msgs::msg::Error &error)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    agv_state_->errors.push_back(error);
}

vda5050_msgs::msg::Error VDAMissionClient::CreateError(
    ErrorLevel level, const std::string &error_msg,
    const std::vector<std::pair<std::string, std::string>> &error_refs,
    const std::string &error_type)
{
    auto error = vda5050_msgs::msg::Error();
    switch (level)
    {
    case ErrorLevel::WARNING:
        error.error_level = error.WARNING;
        break;
    case ErrorLevel::FATAL:
        error.error_level = error.FATAL;
        break;
    default:
        error.error_level = error.FATAL;
        break;
    }
    error.error_description = error_msg;
    error.error_references = CreateErrorReferenceList(error_refs);
    error.error_type = error_type;
    return error;
}
vda5050_msgs::msg::ErrorReference VDAMissionClient::CreateErrorReference(
    const std::string &reference_key, const std::string &reference_value)
{
    auto error_reference = vda5050_msgs::msg::ErrorReference();
    error_reference.reference_key = reference_key;
    error_reference.reference_value = reference_value;
    return error_reference;
}
#pragma endregion

#pragma region Navigation function
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
void VDAMissionClient::NavGoalResponseCallback(const rclcpp_action::ClientGoalHandle<NavThroughPoses>::SharedPtr &goal_handle)
{
    if (!goal_handle)
    {
        RCLCPP_WARN(get_logger(), "Goal was rejected by server");
        auto error = vda5050_msgs::msg::Error();
        error = CreateError(
            ErrorLevel::FATAL, "Goal rejected",
            {{"node_id", current_order_->nodes[current_node_].node_id}});
        agv_state_->driving = false;
        agv_state_->errors.push_back(error);
        nav_goal_handle_.reset();
        PublishRobotState();
        client_state_ = ClientState::IDLE;
    }
    else
    {
        nav_goal_handle_ = goal_handle;
        agv_state_->driving = true;
        RCLCPP_DEBUG(get_logger(), "Goal accepted by server, waiting for result");
    }
}
void VDAMissionClient::NavFeedbackCallback(GoalHandleNavThroughPoses::SharedPtr, const NavThroughPoses::Feedback::ConstSharedPtr feedback)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_node_ = next_stop_ - feedback->number_of_poses_remaining;
    agv_state_->last_node_id =
        current_order_->nodes[current_node_].node_id;
    agv_state_->last_node_sequence_id =
        current_order_->nodes[current_node_].sequence_id;
    agv_state_->driving = true;
    while (agv_state_->node_states.size() > current_order_->nodes.size() - current_node_ - 1)
    {
        agv_state_->node_states.erase(
            agv_state_->node_states.begin());
    }
}
void VDAMissionClient::NavResultCallback(const GoalHandleNavThroughPoses::WrappedResult &result)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto error = vda5050_msgs::msg::Error();
    nav_goal_handle_.reset();
    agv_state_->driving = false;
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(
            get_logger(), "Nav goal was aborted at node_id %s",
            agv_state_->last_node_id.c_str());
        current_node_ = 1;
        error = CreateError(
            ErrorLevel::FATAL, "Nav goal aborted",
            {{"node_id", current_order_->nodes[current_node_].node_id}});
        agv_state_->errors.push_back(error);
        PublishRobotState();
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(
            get_logger(), "Nav goal was canceled at node_id %s",
            agv_state_->last_node_id.c_str());
        // Report error if the order is not paused and cancel action is null
        if (pause_order_ != true && cancel_action_ == nullptr && pause_action_ == nullptr)
        {
            RCLCPP_ERROR(get_logger(), "Navigation goal canceled unexpectedly");
            error = CreateError(
                ErrorLevel::FATAL, "Navigation goal canceled",
                {{"node_id", current_order_->nodes[current_node_].node_id}});
            agv_state_->errors.push_back(error);
        }
        nav_goal_handle_.reset();
        PublishRobotState();
        return;
    default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
    // Logic if navigation was successful
    RCLCPP_INFO(get_logger(), "Reached order node: %ld", next_stop_);
    current_node_ = next_stop_;
    while (agv_state_->node_states.size() > current_order_->nodes.size() - current_node_ - 1)
    {
        agv_state_->node_states.erase(
            agv_state_->node_states.begin());
    }
    agv_state_->last_node_id =
        current_order_->nodes[current_node_].node_id;
    agv_state_->last_node_sequence_id =
        current_order_->nodes[current_node_].sequence_id;
    reached_waypoint_ = true;
}

#pragma endregion

#pragma region process callback function of ROS
void VDAMissionClient::BatteryStateCallback(const sensor_msgs::msg::BatteryState::ConstSharedPtr msg)
{
    agv_state_->battery_state.battery_charge = msg->percentage * 100;
    agv_state_->battery_state.battery_voltage = msg->voltage;
    // POWER_SUPPLY_STATUS_CHARGING = 1
    agv_state_->battery_state.charging = (msg->power_supply_status == 1) ? true : false;

    // battery_health and reach are currently not supported
    agv_state_->battery_state.battery_health = 0;
    agv_state_->battery_state.reach = 0;
}
void VDAMissionClient::OdometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    agv_state_->velocity.vx = msg->twist.twist.linear.x;
    agv_state_->velocity.vy = msg->twist.twist.linear.y;
    agv_state_->velocity.omega = msg->twist.twist.angular.z;
    visualization_->velocity = agv_state_->velocity;
}

#pragma endregion

#pragma region Robot Status

void VDAMissionClient::UpdateActionStateById(
    const std::string &action_id,
    const std::string &status,
    const std::string &result_description)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    auto it = std::find_if(
        agv_state_->action_states.begin(),
        agv_state_->action_states.end(),
        [&action_id](const VDAActionState &action_state)
        {
            return action_state.action_id == action_id;
        });
    if (it == agv_state_->action_states.end())
    {
        RCLCPP_ERROR(get_logger(), "Error while processing action state. Couldn't find action with id: %s", action_id.c_str());
    }
    else
    {
        it->action_status = status;
        RCLCPP_DEBUG(get_logger(), "Updated action state for action %s to %s", action_id.c_str(), status.c_str());
        it->result_description = result_description;
        PublishRobotState();
    }
}

void VDAMissionClient::UpdateActionState(const vda5050_msgs::msg::Action &action, const std::string &status, const std::string &result_description)
{
    UpdateActionStateById(action.action_id, status, result_description);
}

#pragma endregion

#pragma region process with vda
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

void VDAMissionClient::ProcessOrder(const vda5050_msgs::msg::Order::ConstSharedPtr msg)
{
    RCLCPP_INFO(
        get_logger(), "Order with order_id %s received",
        msg->order_id.c_str());
    if (!CanAcceptOrder())
    {
        if (msg->order_id != current_order_->order_id)
        {
            RCLCPP_INFO(
                get_logger(), "One order is running. Order %s is ignored",
                msg->order_id.c_str());
            vda5050_msgs::msg::Error error = CreateError(
                ErrorLevel::WARNING, "An order is running",
                {{"orderId", msg->order_id}},
                kOrderUpdateError);
            AddError(error);
        }
    }
    else if (!msg->nodes.empty())
    {
        current_order_ = msg;
        current_node_ = 0;
        next_stop_ = 0;
        current_node_action_ = 0, current_action_state_ = 0;
        InitAGVState();
    }
}

void VDAMissionClient::ExecuteOrderCallback()
{
    if (cancel_action_)
    {
        CancelOrder();
    }
    if (pause_action_)
    {
        PauseOrder();
    }
    // For a navigatable robot, it has to get position initialized
    // Get robot position
    if (!UpdateRobotPoseFromTf())
    {
        return;
    }
    if (client_state_ == ClientState::RUNNING)
    {
        RCLCPP_DEBUG(get_logger(), "Executing order");
        // Stop executing order if there is a fatal error
        std::unique_lock<std::mutex> lock(state_mutex_);
        if (agv_state_->errors.size() > 0 &&
            agv_state_->errors[0].error_level == vda5050_msgs::msg::Error::FATAL)
        {
            RCLCPP_ERROR(get_logger(), "Fatal error detected, stopping order execution");
            // Set client state to IDLE if no running actions
            for (size_t i = 0; i < num_actions_; i++)
            {
                if (agv_state_->action_states[i].action_status == VDAActionState::RUNNING ||
                    agv_state_->action_states[i].action_status == VDAActionState::INITIALIZING)
                {
                    return;
                }
            }
            RCLCPP_INFO(get_logger(), "Order %s is completed with fatal error. Mission Client is IDLE.",
                        current_order_->order_id.c_str());
            client_state_ = ClientState::IDLE;
            return;
        }
        lock.unlock();
        // Check if the robot has reached the current node
        if (reached_waypoint_)
        {
            // Go through the actions in the current node
            bool stop_driving = false;
            bool has_running_action = false;
            for (size_t action_idx = 0; action_idx < current_order_->nodes[current_node_].actions.size();
                 action_idx++)
            {
                std::string action_id =
                    current_order_->nodes[current_node_].actions[action_idx].action_id;
                std::string action_status = GetActionState(action_id);
                std::string blocking_type =
                    current_order_->nodes[current_node_].actions[action_idx].blocking_type;
                // If blocking type is empty, it is regarded as HARD blocking
                bool running_or_initializing = action_status == VDAActionState::RUNNING ||
                                               action_status == VDAActionState::INITIALIZING;
                if (blocking_type.empty())
                {
                    blocking_type = vda5050_msgs::msg::Action::HARD;
                }
                if (action_status == VDAActionState::FINISHED || action_status == VDAActionState::FAILED)
                {
                    continue;
                }
                else if (running_or_initializing)
                {
                    has_running_action = true;
                    if (blocking_type == vda5050_msgs::msg::Action::HARD)
                    {
                        // Hard blocking action is running. Wait for it to finish.
                        return;
                    }
                    if (blocking_type == vda5050_msgs::msg::Action::SOFT)
                    {
                        stop_driving = true;
                    }
                }
                else if (action_status == VDAActionState::WAITING)
                {
                    if (blocking_type == vda5050_msgs::msg::Action::HARD)
                    {
                        // Hard blocking action is waiting. Execute it if there is no running action.
                        if (!has_running_action)
                        {
                            ExecuteAction(
                                current_order_->nodes[current_node_].actions[action_idx]);
                        }
                        // Wait for the hard blocking action to finish
                        return;
                    }
                    else
                    {
                        if (blocking_type == vda5050_msgs::msg::Action::SOFT)
                        {
                            stop_driving = true;
                        }
                        // Execute soft/none blocking action
                        ExecuteAction(
                            current_order_->nodes[current_node_].actions[action_idx]);
                        has_running_action = true;
                    }
                }
            }
            // SOFT or HARD blocking action is running, stop driving
            if (stop_driving)
            {
                RCLCPP_DEBUG(get_logger(), "SOFT or HARD blocking action is running, stop driving");
                return;
            }
            // SOFT and HARD actions are all done, go to next node
            next_stop_++;
            // Check if the order is completed.
            if (next_stop_ >= current_order_->nodes.size())
            {
                RCLCPP_INFO(
                    get_logger(), "Order %s is completed.", current_order_->order_id.c_str());
                PublishRobotState();
                client_state_ = ClientState::IDLE;
            }
            else
            {
                reached_waypoint_ = false;
                NavigateThroughPoses();
            }
        }
    }
}

bool VDAMissionClient::UpdateRobotPoseFromTf()
{
    if (!tf_buffer_ || !agv_state_ || !visualization_)
    {
        return true;
    }

    try
    {
        geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
            "map", "base_link", tf2::TimePointZero);

        std::lock_guard<std::mutex> lock(state_mutex_);
        agv_state_->agv_position.x = t.transform.translation.x;
        agv_state_->agv_position.y = t.transform.translation.y;
        visualization_->agv_position.x = t.transform.translation.x;
        visualization_->agv_position.y = t.transform.translation.y;

        tf2::Quaternion quaternion;
        tf2::fromMsg(t.transform.rotation, quaternion);
        tf2::Matrix3x3 matrix(quaternion);
        double roll, pitch, yaw;
        matrix.getEulerYPR(yaw, pitch, roll);
        agv_state_->agv_position.theta = yaw;
        agv_state_->agv_position.position_initialized = true;
        visualization_->agv_position.theta = yaw;
        RCLCPP_INFO_ONCE(this->get_logger(), "Robot position initialized");
        return true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 120000,
            "Could not get robot position: %s", ex.what());
        return false;
    }
}

void VDAMissionClient::OrderValidErrorCallback(const std_msgs::msg::String::ConstSharedPtr msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    vda5050_msgs::msg::Error error = CreateError(
        ErrorLevel::WARNING, "Malformed order",
        {{"Error", msg->data}},
        kValidationError);
    agv_state_->errors.push_back(error);
}

void VDAMissionClient::ProcessInstantActions(const vda5050_msgs::msg::InstantActions::ConstSharedPtr msg)
{
    RCLCPP_INFO(
        get_logger(), "Instant actions with header_id %d received",
        msg->header_id);
    std::unique_lock<std::mutex> lock(state_mutex_);

    for (const vda5050_msgs::msg::Action &action : msg->instant_actions)
    {
        // Get action state from current state
        auto it = std::find_if(
            agv_state_->action_states.begin(),
            agv_state_->action_states.end(),
            [&action](const VDAActionState &action_state)
            {
                return action_state.action_id == action.action_id;
            });
        if (it != agv_state_->action_states.end())
        {
            continue;
        }
        RCLCPP_INFO(
            this->get_logger(), "Processing action %s of type %s.",
            action.action_id.c_str(), action.action_type.c_str());
        // Add action to action_states
        auto action_state = VDAActionState();
        action_state.action_id = action.action_id;
        action_state.action_description = action.action_description;
        action_state.action_status = VDAActionState::WAITING;
        action_state.action_type = action.action_type;
        agv_state_->action_states.push_back(action_state);
        lock.unlock();

        if (action.action_type == "cancelOrder")
        {
            if (cancel_action_ != nullptr)
            {
                RCLCPP_INFO(get_logger(), "Cancel order action is already running");
                UpdateActionState(action, VDAActionState::FAILED);
            }
            cancel_action_ = std::make_shared<vda5050_msgs::msg::Action>(action);
        }
        else if (action.action_type == "startPause")
        {
            pause_action_ = std::make_shared<vda5050_msgs::msg::Action>(action);
            PauseOrder();
        }
        else if (action.action_type == "stopPause")
        {
            ResumeOrder(action);
        }
        else
        {
            auto handler = action_handler_map_.find(action.action_type);
            if (handler != action_handler_map_.end())
            {
                handler->second->Execute(action);
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "Action handler not found for action type: %s",
                             action.action_type.c_str());
            }
        }
    }
}

void VDAMissionClient::PublishRobotState()
{
    // Do nothing
}

#pragma endregion

#pragma Spin node
bool VDAMissionClient::SpinOnce()
{
    try
    {
        //_executor.spin_node_some(this->get_node_base_interface());
        rclcpp::spin_some(this->get_node_base_interface());
        return true;
    }
    catch (const std::exception &ex)
    {
        RCLCPP_ERROR(this->get_logger(), "Except when spin node %s", ex.what());
        return false;
    }
}

#pragma endregion
extern "C"
{
    RCLCPP_EXPORT void InitEnviroment()
    {
        rclcpp::init(0, nullptr);
    }

    RCLCPP_EXPORT VDAMissionClient *CreateVDAMissionClient(const char *ns)
    {
        std::string name_space = ns;
        return new VDAMissionClient(name_space);
    }

    RCLCPP_EXPORT bool SpinNode(VDAMissionClient *nodePtr) noexcept
    {
        return nodePtr->SpinOnce();
    }

    RCLCPP_EXPORT void ExecuteOrder(VDAMissionClient *client, OrderWrapper *orderWrapper)
    {
        auto order_msg = orderWrapper->entity;
        auto order_ptr = std::make_shared<vda5050_msgs::msg::Order>(order_msg);
        client->ProcessOrder(order_ptr);
    }

    RCLCPP_EXPORT void ExecuteInstantActions(VDAMissionClient *client, InstantActionsWrapper *instantActionsWrapper)
    {
        auto instant_actions_msg = instantActionsWrapper->entity;
        auto instant_actions_ptr = std::make_shared<vda5050_msgs::msg::InstantActions>(instant_actions_msg);
        client->ProcessInstantActions(instant_actions_ptr);
    }

    RCLCPP_EXPORT AGVStateWrapper *GetAGVState(VDAMissionClient *client)
    {

        auto agv_state_msg = client->agv_state_;
        AGVStateWrapper* agv_state_wrapper = new AGVStateWrapper();
        agv_state_wrapper->entity = *agv_state_msg;
        return agv_state_wrapper;
    }

    RCLCPP_EXPORT FactsheetWrapper *GetFactsheet(VDAMissionClient *client)
    {
        auto factsheet_msg = client->factsheet_;
        auto factsheet_wrapper = new FactsheetWrapper();
        factsheet_wrapper->entity = *factsheet_msg;
        return factsheet_wrapper;
    }

    RCLCPP_EXPORT VisualizationWrapper *GetVisualization(VDAMissionClient *client)
    {

        auto visualization_msg = client->visualization_;
        VisualizationWrapper *visualization_wrapper = new VisualizationWrapper();
        visualization_wrapper->entity = *visualization_msg;
        return visualization_wrapper;
    }
}
