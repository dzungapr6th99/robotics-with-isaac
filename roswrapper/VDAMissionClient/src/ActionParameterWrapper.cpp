#include "VDA5050Wrapper/ActionParameterWrapper.hpp"
#include <rclcpp/rclcpp.hpp>

ActionParameterWrapper::ActionParameterWrapper()
{
}

ActionParameterWrapper::~ActionParameterWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT void ActionParameter_SetKey(ActionParameterWrapper *wrapper, const char *data)
    {
        wrapper->entity.key = data ? data : "";
    }

    RCLCPP_EXPORT void ActionParameter_SetValue(ActionParameterWrapper *wrapper, const char *data)
    {
        wrapper->entity.value = data ? data : "";
    }
}
