#include "VDA5050Wrapper/VelocityWrapper.hpp"
#include <rclcpp/rclcpp.hpp>

VelocityWrapper::VelocityWrapper()
{
}

VelocityWrapper::~VelocityWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT double Velocity_GetVx(VelocityWrapper *wrapper)
    {
        return wrapper->entity.vx;
    }

    RCLCPP_EXPORT double Velocity_GetVy(VelocityWrapper *wrapper)
    {
        return wrapper->entity.vy;
    }

    RCLCPP_EXPORT double Velocity_GetOmega(VelocityWrapper *wrapper)
    {
        return wrapper->entity.omega;
    }
}
