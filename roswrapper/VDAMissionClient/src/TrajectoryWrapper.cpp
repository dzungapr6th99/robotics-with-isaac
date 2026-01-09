#include "VDA5050Wrapper/TrajectoryWrapper.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vda5050_msgs/msg/control_point.hpp>
#include <vda5050_msgs/msg/trajectory.hpp>

using ControlPoint = vda5050_msgs::msg::ControlPoint;

TrajectoryWrapper::TrajectoryWrapper()
{
}

TrajectoryWrapper::~TrajectoryWrapper()
{
}

extern "C"
{
    RCLCPP_EXPORT TrajectoryWrapper *Trajectory_Create()
    {
        return new TrajectoryWrapper();
    }

    RCLCPP_EXPORT void Trajectory_Destroy(TrajectoryWrapper *wrapper)
    {
        delete wrapper;
    }

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

    RCLCPP_EXPORT void Trajectory_ClearControlPoints(TrajectoryWrapper *wrapper)
    {
        wrapper->entity.control_points.clear();
    }

    RCLCPP_EXPORT void Trajectory_AddControlPoint(TrajectoryWrapper *wrapper, const ControlPoint *controlPoint)
    {
        if (!controlPoint)
        {
            return;
        }
        wrapper->entity.control_points.push_back(*controlPoint);
    }

    // ---- Getters for Trajectory message (used in EdgeState.trajectory) ----
    RCLCPP_EXPORT int32_t Trajectory_GetDegree(const vda5050_msgs::msg::Trajectory *trajectory)
    {
        return trajectory ? trajectory->degree : 0;
    }

    RCLCPP_EXPORT int32_t Trajectory_GetKnotVectorCount(const vda5050_msgs::msg::Trajectory *trajectory)
    {
        return trajectory ? static_cast<int32_t>(trajectory->knot_vector.size()) : 0;
    }

    RCLCPP_EXPORT double Trajectory_GetKnotVectorAt(const vda5050_msgs::msg::Trajectory *trajectory, int index)
    {
        if (!trajectory || index < 0 || index >= static_cast<int>(trajectory->knot_vector.size()))
        {
            return 0.0;
        }
        return trajectory->knot_vector[static_cast<size_t>(index)];
    }

    RCLCPP_EXPORT int32_t Trajectory_GetControlPointsCount(const vda5050_msgs::msg::Trajectory *trajectory)
    {
        return trajectory ? static_cast<int32_t>(trajectory->control_points.size()) : 0;
    }

    RCLCPP_EXPORT const vda5050_msgs::msg::ControlPoint *Trajectory_GetControlPointAt(
        const vda5050_msgs::msg::Trajectory *trajectory, int index)
    {
        if (!trajectory || index < 0 || index >= static_cast<int>(trajectory->control_points.size()))
        {
            return nullptr;
        }
        return &trajectory->control_points[static_cast<size_t>(index)];
    }
}
