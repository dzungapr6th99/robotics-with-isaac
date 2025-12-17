#include "VDA5050Wrapper/ActionWrapper.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <vda5050_msgs/msg/action_parameter.hpp>
using ActionParameter = vda5050_msgs::msg::ActionParameter;
ActionWrapper::ActionWrapper()
{

}

ActionWrapper::~ActionWrapper()
{
    
}

extern "C"
{
    RCLCPP_EXPORT void Action_ActionType(ActionWrapper *actionWrapper, const char *data)
    {
        actionWrapper->entity.action_type = data ? data : "";
    }

    RCLCPP_EXPORT void Action_SetActionDescription(ActionWrapper *actionWrapper, const char *data)
    {
        actionWrapper->entity.action_description  = data ? data : "";
    }


    RCLCPP_EXPORT void Action_SetActionId(ActionWrapper *actionWrapper, char *data)
    {
        actionWrapper->entity.action_id  = data ? data : "";
    }

    RCLCPP_EXPORT void Action_SetBlockingType(ActionWrapper *actionWrapper, char *data)
    {
        actionWrapper->entity.blocking_type = data ? data : "";
    }

    RCLCPP_EXPORT void Action_SetActionParameters(ActionWrapper *actionWrapper, const ActionParameter *parameters, int length)
    {
        actionWrapper->entity.action_parameters = std::vector<ActionParameter>(parameters, parameters +length);
    }

}
