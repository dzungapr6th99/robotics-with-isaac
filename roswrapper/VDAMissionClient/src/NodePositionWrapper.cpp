#include "VDA5050Wrapper/NodePositionWrapper.hpp"
#include <rclcpp/rclcpp.hpp>

NodePositionWrapper::NodePositionWrapper()
{
}

NodePositionWrapper::~NodePositionWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT NodePositionWrapper *NodePosition_Create()
    {
        return new NodePositionWrapper();
    }

    RCLCPP_EXPORT void NodePosition_Destroy(NodePositionWrapper *nodePositionWrapper)
    {
        delete nodePositionWrapper;
    }

    RCLCPP_EXPORT void NodePosition_SetX(NodePositionWrapper *nodePositionWrapper, double data)
    {
        nodePositionWrapper->entity.x = data;
    }

    RCLCPP_EXPORT void NodePosition_SetY(NodePositionWrapper *nodePositionWrapper, double data)
    {
        nodePositionWrapper->entity.y = data;
    }

    RCLCPP_EXPORT void NodePosition_SetTheta(NodePositionWrapper *nodePositionWrapper, double data)
    {
        nodePositionWrapper->entity.theta = data;
    }

    RCLCPP_EXPORT void NodePosition_SetAllowedDeviationXY(NodePositionWrapper *nodePositionWrapper, float data)
    {
        nodePositionWrapper->entity.allowed_deviation_x_y = data;
    }

    RCLCPP_EXPORT void NodePosition_SetAllowedDeviationTheta(NodePositionWrapper *nodePositionWrapper, float data)
    {
        nodePositionWrapper->entity.allowed_deviation_theta = data;
    }

    RCLCPP_EXPORT void NodePosition_SetMapId(NodePositionWrapper *nodePositionWrapper, const char *data)
    {
        nodePositionWrapper->entity.map_id = data ? data : "";
    }

    RCLCPP_EXPORT void NodePosition_SetMapDescription(NodePositionWrapper *nodePositionWrapper, const char *data)
    {
        nodePositionWrapper->entity.map_description = data ? data : "";
    }
}
