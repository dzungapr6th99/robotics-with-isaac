#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/error.hpp>
#include <vda5050_msgs/msg/error_reference.hpp>

using Error = vda5050_msgs::msg::Error;
using ErrorReference = vda5050_msgs::msg::ErrorReference;

extern "C"
{
    RCLCPP_EXPORT const char *Error_GetErrorType(const Error *error)
    {
        return error ? error->error_type.c_str() : "";
    }

    RCLCPP_EXPORT const char *Error_GetErrorDescription(const Error *error)
    {
        return error ? error->error_description.c_str() : "";
    }

    RCLCPP_EXPORT const char *Error_GetErrorLevel(const Error *error)
    {
        return error ? error->error_level.c_str() : "";
    }

    RCLCPP_EXPORT int32_t Error_GetErrorReferencesCount(const Error *error)
    {
        return error ? static_cast<int32_t>(error->error_references.size()) : 0;
    }

    RCLCPP_EXPORT const ErrorReference *Error_GetErrorReferenceAt(const Error *error, int index)
    {
        if (!error || index < 0 || index >= static_cast<int>(error->error_references.size()))
        {
            return nullptr;
        }
        return &error->error_references[static_cast<size_t>(index)];
    }

    RCLCPP_EXPORT const char *ErrorReference_GetReferenceKey(const ErrorReference *reference)
    {
        return reference ? reference->reference_key.c_str() : "";
    }

    RCLCPP_EXPORT const char *ErrorReference_GetReferenceValue(const ErrorReference *reference)
    {
        return reference ? reference->reference_value.c_str() : "";
    }
}
