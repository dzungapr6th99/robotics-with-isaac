#include "VDA5050Wrapper/SafetyStateWrapper.hpp"
#include <rclcpp/rclcpp.hpp>

SafetyStateWrapper::SafetyStateWrapper()
{
}

SafetyStateWrapper::~SafetyStateWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT const char *SafetyState_GetEStop(SafetyStateWrapper *wrapper)
    {
        return wrapper->entity.e_stop.c_str();
    }

    RCLCPP_EXPORT bool SafetyState_GetFieldViolation(SafetyStateWrapper *wrapper)
    {
        return wrapper->entity.field_violation;
    }
}
