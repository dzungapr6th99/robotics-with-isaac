#include "VDA5050Wrapper/ControlPointWrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/control_point.hpp>

ControlPointWrapper::ControlPointWrapper()
{
}

ControlPointWrapper::~ControlPointWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT ControlPointWrapper *ControlPoint_Create()
    {
        return new ControlPointWrapper();
    }

    RCLCPP_EXPORT void ControlPoint_Destroy(ControlPointWrapper *wrapper)
    {
        delete wrapper;
    }

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

    // ---- Getters for ControlPoint message (used in Trajectory.control_points) ----
    RCLCPP_EXPORT double ControlPoint_GetX(const vda5050_msgs::msg::ControlPoint *cp)
    {
        return cp ? cp->x : 0.0;
    }

    RCLCPP_EXPORT double ControlPoint_GetY(const vda5050_msgs::msg::ControlPoint *cp)
    {
        return cp ? cp->y : 0.0;
    }

    RCLCPP_EXPORT double ControlPoint_GetWeight(const vda5050_msgs::msg::ControlPoint *cp)
    {
        return cp ? cp->weight : 0.0;
    }
}
