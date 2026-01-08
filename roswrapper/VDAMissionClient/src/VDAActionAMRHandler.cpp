#include "VDAMissionClient/VDAActionAMRHandler.hpp"
#include "vda5050_msgs/msg/action_parameter.hpp"
#include <string>
#include <thread>
void VDAActionAMRHandler::Initialize(VDAMissionClient *client_node)
{
    client_node_ = client_node;
}

void VDAActionAMRHandler::Execute(const vda5050_msgs::msg::Action &vda5050_action)
{
    client_node_->UpdateActionState(vda5050_action, vda5050_msgs::msg::ActionState::RUNNING);
    std::thread(
        [this, vda5050_action]()
        {
            if (vda5050_action.action_type == "PICK")
            {
                HandlePick(vda5050_action);
            }
            else if (vda5050_action.action_type == "DROP")
            {
                HandleDrop(vda5050_action);
            }
        })
        .detach();
}

void VDAActionAMRHandler::Cancel(const std::string &action_id)
{
    client_node_->UpdateActionStateById(action_id, vda5050_msgs::msg::ActionState::FAILED);
}

void VDAActionAMRHandler::HandlePick(vda5050_msgs::msg::Action action)
{
    std::string currentParameterName;
    std::string currentParameterValue;
    try
    {
        double theta, height = 0;
        for (int i = 0; i < action.action_parameters.size(); i++)
        {
            vda5050_msgs::msg::ActionParameter actionParameter = action.action_parameters[i];
            currentParameterName = actionParameter.key;
            currentParameterValue = actionParameter.value;
            if (actionParameter.key == "theta")
            {
                theta = std::stod(actionParameter.value);
            }
            if (actionParameter.key == "height")
            {
                height = std::stod(actionParameter.value);
            }
            // Continue code here to control IO to lift up / down
        }
    }
    catch (const std::exception &e)
    {
        vda5050_msgs::msg::Error error;
        error.error_description = e.what();
        vda5050_msgs::msg::ErrorReference errorReference;
        errorReference.reference_key = currentParameterName;
        errorReference.reference_value = currentParameterValue;
        error.error_references.push_back(errorReference);
        client_node_->AddError(error);
    }
}

void VDAActionAMRHandler::HandleDrop(vda5050_msgs::msg::Action action)
{
    std::string currentParameterName;
    std::string currentParameterValue;
    try
    {
        double theta, height = 0;
        for (int i = 0; i < action.action_parameters.size(); i++)
        {
            vda5050_msgs::msg::ActionParameter actionParameter = action.action_parameters[i];
            currentParameterName = actionParameter.key;
            currentParameterValue = actionParameter.value;
            if (actionParameter.key == "theta")
            {
                theta = std::stod(actionParameter.value);
            }
            if (actionParameter.key == "height")
            {
                height = std::stod(actionParameter.value);
            }
            // Continue code here to control IO to lift up / down
        }
    }
    catch (const std::exception &e)
    {
        vda5050_msgs::msg::Error error;
        error.error_description = e.what();
        vda5050_msgs::msg::ErrorReference errorReference;
        errorReference.reference_key = currentParameterName;
        errorReference.reference_value = currentParameterValue;
        error.error_references.push_back(errorReference);
        client_node_->AddError(error);
    }
}
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(VDAActionAMRHandler, Vda5050ActionHandlerBase)
