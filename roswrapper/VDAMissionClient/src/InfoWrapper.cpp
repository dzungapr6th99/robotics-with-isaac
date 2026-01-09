#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/info.hpp>
#include <vda5050_msgs/msg/info_reference.hpp>

using Info = vda5050_msgs::msg::Info;
using InfoReference = vda5050_msgs::msg::InfoReference;

extern "C"
{
    RCLCPP_EXPORT const char *Info_GetInfoType(const Info *info)
    {
        return info ? info->info_type.c_str() : "";
    }

    RCLCPP_EXPORT const char *Info_GetInfoDescription(const Info *info)
    {
        return info ? info->info_description.c_str() : "";
    }

    RCLCPP_EXPORT const char *Info_GetInfoLevel(const Info *info)
    {
        return info ? info->info_level.c_str() : "";
    }

    RCLCPP_EXPORT int32_t Info_GetInfoReferencesCount(const Info *info)
    {
        return info ? static_cast<int32_t>(info->info_references.size()) : 0;
    }

    RCLCPP_EXPORT const InfoReference *Info_GetInfoReferenceAt(const Info *info, int index)
    {
        if (!info || index < 0 || index >= static_cast<int>(info->info_references.size()))
        {
            return nullptr;
        }
        return &info->info_references[static_cast<size_t>(index)];
    }

    RCLCPP_EXPORT const char *InfoReference_GetReferenceKey(const InfoReference *reference)
    {
        return reference ? reference->reference_key.c_str() : "";
    }

    RCLCPP_EXPORT const char *InfoReference_GetReferenceValue(const InfoReference *reference)
    {
        return reference ? reference->reference_value.c_str() : "";
    }
}

