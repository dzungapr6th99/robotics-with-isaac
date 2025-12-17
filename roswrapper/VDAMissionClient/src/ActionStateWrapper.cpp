#include "VDA5050Wrapper/ActionStateWrapper.hpp"
#include <rclcpp/rclcpp.hpp>

ActionStateWrapper::ActionStateWrapper()
{
}

ActionStateWrapper::~ActionStateWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT const char *ActionState_GetActionId(ActionStateWrapper *wrapper)
    {
        return wrapper->entity.action_id.c_str();
    }

    RCLCPP_EXPORT const char *ActionState_GetActionType(ActionStateWrapper *wrapper)
    {
        return wrapper->entity.action_type.c_str();
    }

    RCLCPP_EXPORT const char *ActionState_GetActionDescription(ActionStateWrapper *wrapper)
    {
        return wrapper->entity.action_description.c_str();
    }

    RCLCPP_EXPORT const char *ActionState_GetActionStatus(ActionStateWrapper *wrapper)
    {
        return wrapper->entity.action_status.c_str();
    }

    RCLCPP_EXPORT const char *ActionState_GetResultDescription(ActionStateWrapper *wrapper)
    {
        return wrapper->entity.result_description.c_str();
    }
}
