#include "VDA5050Wrapper/TypeSpecificationWrapper.hpp"
#include <rclcpp/rclcpp.hpp>

TypeSpecificationWrapper::TypeSpecificationWrapper()
{
}

TypeSpecificationWrapper::~TypeSpecificationWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT TypeSpecificationWrapper *TypeSpecification_Create()
    {
        return new TypeSpecificationWrapper();
    }

    RCLCPP_EXPORT void TypeSpecification_Destroy(TypeSpecificationWrapper *wrapper)
    {
        delete wrapper;
    }

    RCLCPP_EXPORT const char *TypeSpecification_GetSeriesName(TypeSpecificationWrapper *wrapper)
    {
        return wrapper->entity.series_name.c_str();
    }

    RCLCPP_EXPORT const char *TypeSpecification_GetSeriesDescription(TypeSpecificationWrapper *wrapper)
    {
        return wrapper->entity.series_description.c_str();
    }

    RCLCPP_EXPORT const char *TypeSpecification_GetAgvKinematics(TypeSpecificationWrapper *wrapper)
    {
        return wrapper->entity.agv_kinematics.c_str();
    }

    RCLCPP_EXPORT const char *TypeSpecification_GetAgvClass(TypeSpecificationWrapper *wrapper)
    {
        return wrapper->entity.agv_class.c_str();
    }

    RCLCPP_EXPORT double TypeSpecification_GetMaxLoadMass(TypeSpecificationWrapper *wrapper)
    {
        return wrapper->entity.max_load_mass;
    }

    RCLCPP_EXPORT const std::string *TypeSpecification_GetLocalizationTypes(TypeSpecificationWrapper *wrapper, int *length)
    {
        if (length)
        {
            *length = static_cast<int>(wrapper->entity.localization_types.size());
        }
        return wrapper->entity.localization_types.data();
    }

    RCLCPP_EXPORT const std::string *TypeSpecification_GetNavigationTypes(TypeSpecificationWrapper *wrapper, int *length)
    {
        if (length)
        {
            *length = static_cast<int>(wrapper->entity.navigation_types.size());
        }
        return wrapper->entity.navigation_types.data();
    }

    // Helper to fetch C-string data from a std::string array returned above.
    RCLCPP_EXPORT const char *TypeSpecification_GetStringData(const std::string *data, int index)
    {
        if (!data || index < 0)
        {
            return nullptr;
        }
        return (data + index)->c_str();
    }
}
