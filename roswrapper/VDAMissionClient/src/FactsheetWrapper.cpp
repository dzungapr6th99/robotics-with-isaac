#include "VDA5050Wrapper/FactsheetWrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/type_specification.hpp>
#include <vda5050_msgs/msg/physical_parameters.hpp>

using TypeSpecification = vda5050_msgs::msg::TypeSpecification;
using PhysicalParameters = vda5050_msgs::msg::PhysicalParameters;

FactsheetWrapper::FactsheetWrapper()
{
}

FactsheetWrapper::~FactsheetWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT FactsheetWrapper *Factsheet_Create()
    {
        return new FactsheetWrapper();
    }

    RCLCPP_EXPORT void Factsheet_Destroy(FactsheetWrapper *wrapper)
    {
        delete wrapper;
    }

    RCLCPP_EXPORT int32_t Factsheet_GetHeaderId(FactsheetWrapper *wrapper)
    {
        return wrapper->entity.header_id;
    }

    RCLCPP_EXPORT const char *Factsheet_GetTimestamp(FactsheetWrapper *wrapper)
    {
        return wrapper->entity.timestamp.c_str();
    }

    RCLCPP_EXPORT const char *Factsheet_GetVersion(FactsheetWrapper *wrapper)
    {
        return wrapper->entity.version.c_str();
    }

    RCLCPP_EXPORT const char *Factsheet_GetManufacturer(FactsheetWrapper *wrapper)
    {
        return wrapper->entity.manufacturer.c_str();
    }

    RCLCPP_EXPORT const char *Factsheet_GetSerialNumber(FactsheetWrapper *wrapper)
    {
        return wrapper->entity.serial_number.c_str();
    }

    RCLCPP_EXPORT const TypeSpecification *Factsheet_GetTypeSpecification(FactsheetWrapper *wrapper)
    {
        return &wrapper->entity.type_specification;
    }

    RCLCPP_EXPORT const PhysicalParameters *Factsheet_GetPhysicalParameters(FactsheetWrapper *wrapper)
    {
        return &wrapper->entity.physical_parameters;
    }
}
