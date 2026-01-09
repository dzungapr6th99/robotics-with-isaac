#include "VDAMissionClient/VDAActionHandlerInitialPose.hpp"
#include "vda5050_msgs/msg/action_parameter.hpp"
#include <string>
#include <thread>
void VDAActionHandlerInitialPose::Initialize(VDAMissionClient *client_node)
{
    client_node_ = client_node;
}

void VDAActionHandlerInitialPose::Execute(const vda5050_msgs::msg::Action &vda5050_action)
{

    client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::RUNNING);

    bool isSet = false;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    for (const auto &param : vda5050_action.action_parameters)
    {
        if (param.key == "x")
        {
            isSet = true;
            x = std::stod(param.value);
        }
        else if (param.key == "y")
        {
            isSet = true;
            y = std::stod(param.value);
        }
        else if (param.key == "theta")
        {
            isSet = true;
            theta = std::stod(param.value);
        }
    }
    if (isSet)
    {
        client_node_->InitialPose(x, y, theta);
        client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::FINISHED);
        RCLCPP_INFO(rclcpp::get_logger("ExampleActionHandler"), "Example action %s finished",
                    vda5050_action.action_id.c_str());
    }
    else
    {
        client_node_->UpdateActionState(
            vda5050_action, vda5050_msgs::msg::ActionState::FAILED);
        RCLCPP_INFO(rclcpp::get_logger("ExampleActionHandler"), "Example action %s failed",
                    vda5050_action.action_id.c_str());
    }
}

void VDAActionHandlerInitialPose::Cancel(const std::string &action_id)
{
    client_node_->UpdateActionStateById(action_id, vda5050_msgs::msg::ActionState::FAILED);
}

void VDAActionHandlerInitialPose::HandleInitPosition(vda5050_msgs::msg::Action action)
{
    auto parameters = action.action_parameters;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    for (int i = 0; i < parameters.size(); i++)
    {
        auto param = parameters[i];
        if (param.key == "x")
        {
            x = std::stod(param.value);
        }
        else if (param.key == "y")
        {
            y = std::stod(param.value);
        }
        else if (param.key == "theta")
        {
            theta = std::stod(param.value);
        }
    }
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(VDAActionHandlerInitialPose, Vda5050ActionHandlerBase)
