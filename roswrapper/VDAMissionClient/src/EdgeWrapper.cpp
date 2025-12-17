#include "VDA5050Wrapper/EdgeWrapper.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <vda5050_msgs/msg/trajectory.hpp>
using Action = vda5050_msgs::msg::Action;
using Trajectory = vda5050_msgs::msg::Trajectory;
EdgeWrapper::EdgeWrapper()
{
}

EdgeWrapper::~EdgeWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT void Edge_SetEdgeId(EdgeWrapper *edgeWrapper, const char *data)
    {
        edgeWrapper->entity.edge_id = data ? data : "";
    }

    RCLCPP_EXPORT void Edge_SetSequenceId(EdgeWrapper *edgeWrapper, uint32_t sequenceId)
    {
        edgeWrapper->entity.sequence_id = sequenceId;
    }

    RCLCPP_EXPORT void Edge_SetDescription(EdgeWrapper *edgeWrapper, const char *data)
    {
        edgeWrapper->entity.edge_description  = data ? data : "";
    }

    RCLCPP_EXPORT void Edge_SetRelease(EdgeWrapper *edgeWrapper, bool data)
    {
        edgeWrapper->entity.released = data;
    }

    RCLCPP_EXPORT void Edge_SetStartNodeId(EdgeWrapper *edgeWrapper, char *data)
    {
        edgeWrapper->entity.start_node_id  = data ? data : "";
    }

    RCLCPP_EXPORT void Edge_SetEndNodeId(EdgeWrapper *edgeWrapper, char *data)
    {
        edgeWrapper->entity.end_node_id = data ? data : "";
    }

    RCLCPP_EXPORT void Edge_SetMaxSpeed(EdgeWrapper *edgeWrapper, double data)
    {
        edgeWrapper->entity.max_speed = data;
    }

    RCLCPP_EXPORT void Edge_SetMaxHeight(EdgeWrapper *edgeWrapper, double data)
    {
        edgeWrapper->entity.max_height = data;
    }

    RCLCPP_EXPORT void Edge_SetMinHeight(EdgeWrapper *edgeWrapper, double minHeight)
    {
        edgeWrapper->entity.min_height = minHeight;
    }

    RCLCPP_EXPORT void Edge_SetOrentation(EdgeWrapper *edgeWrapper, double data)
    {
        edgeWrapper->entity.orientation = data;
    }

    RCLCPP_EXPORT void Edge_SetOrientiationType(EdgeWrapper *edgeWrapper, char *data)
    {
        edgeWrapper->entity.orientation_type = data ? data : "";
    }

    RCLCPP_EXPORT void Edge_SetDirection(EdgeWrapper *edgeWrapper, char *data)
    {
        edgeWrapper->entity.direction  = data ? data : "";
    }

    RCLCPP_EXPORT void Edge_SetRotationAllowed(EdgeWrapper *edgeWrapper, bool data)
    {
        edgeWrapper->entity.rotation_allowed = data;
    }

    RCLCPP_EXPORT void Edge_SetMaxRotationSpeed(EdgeWrapper *edgeWrapper, double data)
    {
        edgeWrapper->entity.max_rotation_speed = data;
    }

    RCLCPP_EXPORT void Edge_SetLength(EdgeWrapper *edgeWrapper, double data)
    {
        edgeWrapper->entity.length = data;
    }

    RCLCPP_EXPORT void Edge_SetTrajectory(EdgeWrapper *edgeWrapper, const Trajectory *trajectory)
    {
        edgeWrapper->entity.trajectory = trajectory ? *trajectory : Trajectory();
    }

    RCLCPP_EXPORT void Edge_SetAction(EdgeWrapper *edgeWrapper, const Action *actions, int length)
    {
        edgeWrapper->entity.actions = std::vector<Action>(actions, actions+ length);
    }
}
