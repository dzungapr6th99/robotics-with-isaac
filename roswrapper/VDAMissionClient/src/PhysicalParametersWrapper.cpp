#include "VDA5050Wrapper/PhysicalParametersWrapper.hpp"
#include <rclcpp/rclcpp.hpp>

PhysicalParametersWrapper::PhysicalParametersWrapper()
{
}

PhysicalParametersWrapper::~PhysicalParametersWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT PhysicalParametersWrapper *PhysicalParameters_Create()
    {
        return new PhysicalParametersWrapper();
    }

    RCLCPP_EXPORT void PhysicalParameters_Destroy(PhysicalParametersWrapper *wrapper)
    {
        delete wrapper;
    }

    RCLCPP_EXPORT double PhysicalParameters_GetSpeedMin(PhysicalParametersWrapper *wrapper)
    {
        return wrapper->entity.speed_min;
    }

    RCLCPP_EXPORT double PhysicalParameters_GetSpeedMax(PhysicalParametersWrapper *wrapper)
    {
        return wrapper->entity.speed_max;
    }

    RCLCPP_EXPORT double PhysicalParameters_GetAccelerationMax(PhysicalParametersWrapper *wrapper)
    {
        return wrapper->entity.acceleration_max;
    }

    RCLCPP_EXPORT double PhysicalParameters_GetDecelerationMax(PhysicalParametersWrapper *wrapper)
    {
        return wrapper->entity.deceleration_max;
    }

    RCLCPP_EXPORT double PhysicalParameters_GetHeightMin(PhysicalParametersWrapper *wrapper)
    {
        return wrapper->entity.height_min;
    }

    RCLCPP_EXPORT double PhysicalParameters_GetHeightMax(PhysicalParametersWrapper *wrapper)
    {
        return wrapper->entity.height_max;
    }

    RCLCPP_EXPORT double PhysicalParameters_GetWidth(PhysicalParametersWrapper *wrapper)
    {
        return wrapper->entity.width;
    }

    RCLCPP_EXPORT double PhysicalParameters_GetLength(PhysicalParametersWrapper *wrapper)
    {
        return wrapper->entity.length;
    }
}
