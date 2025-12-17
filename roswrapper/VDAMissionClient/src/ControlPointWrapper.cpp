#include "VDA5050Wrapper/ControlPointWrapper.hpp"
#include <rclcpp/rclcpp.hpp>

ControlPointWrapper::ControlPointWrapper()
{
}

ControlPointWrapper::~ControlPointWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT void ControlPoint_SetX(ControlPointWrapper *wrapper, double data)
    {
        wrapper->entity.x = data;
    }

    RCLCPP_EXPORT void ControlPoint_SetY(ControlPointWrapper *wrapper, double data)
    {
        wrapper->entity.y = data;
    }

    RCLCPP_EXPORT void ControlPoint_SetWeight(ControlPointWrapper *wrapper, double data)
    {
        wrapper->entity.weight = data;
    }
}
