#include "VDA5050Wrapper/TrajectoryWrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/control_point.hpp>

using ControlPoint = vda5050_msgs::msg::ControlPoint;

TrajectoryWrapper::TrajectoryWrapper()
{
}

TrajectoryWrapper::~TrajectoryWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT void Trajectory_SetDegree(TrajectoryWrapper *wrapper, int32_t data)
    {
        wrapper->entity.degree = data;
    }

    RCLCPP_EXPORT void Trajectory_SetKnotVector(TrajectoryWrapper *wrapper, const double *data, int length)
    {
        wrapper->entity.knot_vector = std::vector<double>(data, data + length);
    }

    RCLCPP_EXPORT void Trajectory_SetControlPoints(TrajectoryWrapper *wrapper, const ControlPoint *data, int length)
    {
        wrapper->entity.control_points = std::vector<ControlPoint>(data, data+length);
    }
}
