//
// Created by yuong on 23-11-14.
//

#include "controller/pure_controller.h"

namespace laiseern
{
namespace control
{

bool PurePursuitController::UpdatePath(const std::vector<Point>& trajectory)
{
    if (trajectory.empty())
    {
        std::cout << "path update failed!" << std::endl;
        return false;
    }
    _trajectory = trajectory;
    return true;
}

ControlCommand PurePursuitController::CalculateControl(const State& current_state)
{
    ControlCommand command{};
    if (_trajectory.empty())
    {
        std::cout << "Pp path is empty!" << std::endl;
        return command;
    }
    double closest_distance = std::numeric_limits<double>::max();

    // Find the closest point on the path
    for (size_t i = 0; i < _trajectory.size(); i++)
    {
        double distance =
            std::hypot(_trajectory[i].x - current_state.point.x, _trajectory[i].y - current_state.point.y);
        if (distance < closest_distance)
        {
            closest_distance = distance;
            _closest_index = i;
        }
    }

    Point target{};
    //     Find the point on the path that is look ahead distance away

    if (std::hypot(_trajectory.back().x - current_state.point.x, _trajectory.back().y - current_state.point.y) <
        _look_ahead_distance)
    {
        target.x = _trajectory.back().x;
        target.y = _trajectory.back().y;
    }
    else
    {
        double look_s = 0;
        for (size_t i = _closest_index; i < _trajectory.size() - 1; ++i)
        {
            double distance =
                std::hypot(_trajectory[i].x - _trajectory[i + 1].x, _trajectory[i].y - _trajectory[i + 1].y);
            look_s += distance;
            if (look_s > _look_ahead_distance)
            {
                target.x = _trajectory[i].x;
                target.y = _trajectory[i].y;
                _target_index = i;
                break;
            }
        }
    }

    target = _trajectory[_target_index];

    // 向量求解
#if 0
    double mat_AC_x = target.x - current_state.point.x;
    double mat_AC_y = target.y - current_state.point.y;

    double mat_AB_x = ParamParser::GetInstance()->GetVehicleParam()._wheel_base * cos(current_state.point.yaw);
    double mat_AB_y = ParamParser::GetInstance()->GetVehicleParam()._wheel_base * sin(current_state.point.yaw);

    double sin_alpha =
        (mat_AB_y * mat_AC_x - mat_AC_y * mat_AB_x) / (std::hypot(mat_AC_x, mat_AC_y) * std::hypot(mat_AB_x, mat_AB_y));

    command.steer =
        -atan((2 * ParamParser::GetInstance()->GetVehicleParam()._wheel_base * sin_alpha) / _look_ahead_distance);

#endif

    // 几何求解
#if 1
    double alpha =
        atan2(target.y - current_state.point.y, target.x - current_state.point.x) - (current_state.point.yaw);
    command.steer = std::min(
        atan((2 * ParamParser::GetInstance()->GetVehicleParam()._wheel_base * std::sin(alpha)) / _look_ahead_distance),
        ParamParser::GetInstance()->GetVehicleParam()._max_steering_angle);

#endif
    return command;
}

}  // namespace control

}  // namespace laiseern