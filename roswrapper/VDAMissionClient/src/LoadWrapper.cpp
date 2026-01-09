#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/load.hpp>
#include <vda5050_msgs/msg/bounding_box_reference.hpp>
#include <vda5050_msgs/msg/load_dimensions.hpp>

using Load = vda5050_msgs::msg::Load;
using BoundingBoxReference = vda5050_msgs::msg::BoundingBoxReference;
using LoadDimensions = vda5050_msgs::msg::LoadDimensions;

extern "C"
{
    RCLCPP_EXPORT const char *Load_GetLoadId(const Load *load)
    {
        return load ? load->load_id.c_str() : "";
    }

    RCLCPP_EXPORT const char *Load_GetLoadType(const Load *load)
    {
        return load ? load->load_type.c_str() : "";
    }

    RCLCPP_EXPORT const char *Load_GetLoadPosition(const Load *load)
    {
        return load ? load->load_position.c_str() : "";
    }

    RCLCPP_EXPORT uint32_t Load_GetWeight(const Load *load)
    {
        return load ? load->weight : 0;
    }

    RCLCPP_EXPORT const BoundingBoxReference *Load_GetBoundingBoxReference(const Load *load)
    {
        return load ? &load->bounding_box_reference : nullptr;
    }

    RCLCPP_EXPORT const LoadDimensions *Load_GetLoadDimensions(const Load *load)
    {
        return load ? &load->load_dimensions : nullptr;
    }

    RCLCPP_EXPORT double BoundingBoxReference_GetX(const BoundingBoxReference *bb)
    {
        return bb ? bb->x : 0.0;
    }

    RCLCPP_EXPORT double BoundingBoxReference_GetY(const BoundingBoxReference *bb)
    {
        return bb ? bb->y : 0.0;
    }

    RCLCPP_EXPORT double BoundingBoxReference_GetZ(const BoundingBoxReference *bb)
    {
        return bb ? bb->z : 0.0;
    }

    RCLCPP_EXPORT double BoundingBoxReference_GetTheta(const BoundingBoxReference *bb)
    {
        return bb ? bb->theta : 0.0;
    }

    RCLCPP_EXPORT double LoadDimensions_GetLength(const LoadDimensions *dim)
    {
        return dim ? dim->length : 0.0;
    }

    RCLCPP_EXPORT double LoadDimensions_GetWidth(const LoadDimensions *dim)
    {
        return dim ? dim->width : 0.0;
    }

    RCLCPP_EXPORT double LoadDimensions_GetHeight(const LoadDimensions *dim)
    {
        return dim ? dim->height : 0.0;
    }
}

