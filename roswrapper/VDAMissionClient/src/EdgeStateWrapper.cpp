#include "VDA5050Wrapper/EdgeStateWrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/trajectory.hpp>

using Trajectory = vda5050_msgs::msg::Trajectory;

EdgeStateWrapper::EdgeStateWrapper()
{
}

EdgeStateWrapper::~EdgeStateWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT EdgeStateWrapper *EdgeState_Create()
    {
        return new EdgeStateWrapper();
    }

    RCLCPP_EXPORT void EdgeState_Destroy(EdgeStateWrapper *wrapper)
    {
        delete wrapper;
    }

    RCLCPP_EXPORT const char *EdgeState_GetEdgeId(EdgeStateWrapper *wrapper)
    {
        return wrapper->entity.edge_id.c_str();
    }

    RCLCPP_EXPORT int32_t EdgeState_GetSequenceId(EdgeStateWrapper *wrapper)
    {
        return wrapper->entity.sequence_id;
    }

    RCLCPP_EXPORT const char *EdgeState_GetEdgeDescription(EdgeStateWrapper *wrapper)
    {
        return wrapper->entity.edge_description.c_str();
    }

    RCLCPP_EXPORT bool EdgeState_GetReleased(EdgeStateWrapper *wrapper)
    {
        return wrapper->entity.released;
    }

    RCLCPP_EXPORT const Trajectory *EdgeState_GetTrajectory(EdgeStateWrapper *wrapper)
    {
        return &wrapper->entity.trajectory;
    }
}
