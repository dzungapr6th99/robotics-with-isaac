#include "VDA5050Wrapper/VisualizationWrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/agv_position.hpp>
#include <vda5050_msgs/msg/velocity.hpp>

using AGVPosition = vda5050_msgs::msg::AGVPosition;
using Velocity = vda5050_msgs::msg::Velocity;

VisualizationWrapper::VisualizationWrapper()
{
}

VisualizationWrapper::~VisualizationWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT VisualizationWrapper *Visualization_Create()
    {
        return new VisualizationWrapper();
    }

    RCLCPP_EXPORT void Visualization_Destroy(VisualizationWrapper *wrapper)
    {
        delete wrapper;
    }

    RCLCPP_EXPORT int32_t Visualization_GetHeaderId(VisualizationWrapper *wrapper)
    {
        return wrapper->entity.header_id;
    }

    RCLCPP_EXPORT const char *Visualization_GetTimestamp(VisualizationWrapper *wrapper)
    {
        return wrapper->entity.timestamp.c_str();
    }

    RCLCPP_EXPORT const char *Visualization_GetVersion(VisualizationWrapper *wrapper)
    {
        return wrapper->entity.version.c_str();
    }

    RCLCPP_EXPORT const char *Visualization_GetManufacturer(VisualizationWrapper *wrapper)
    {
        return wrapper->entity.manufacturer.c_str();
    }

    RCLCPP_EXPORT const char *Visualization_GetSerialNumber(VisualizationWrapper *wrapper)
    {
        return wrapper->entity.serial_number.c_str();
    }

    RCLCPP_EXPORT const AGVPosition *Visualization_GetAGVPosition(VisualizationWrapper *wrapper)
    {
        return &wrapper->entity.agv_position;
    }

    RCLCPP_EXPORT const Velocity *Visualization_GetVelocity(VisualizationWrapper *wrapper)
    {
        return &wrapper->entity.velocity;
    }
}
