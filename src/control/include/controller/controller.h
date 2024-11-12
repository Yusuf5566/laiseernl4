/***
 *  @brief      Project
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2024-05-16
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_CONTROLLER_H
#define LAISEERNL4_CONTROLLER_H

#include "common/reflect.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include "common/vehicle_param.h"
#include "common/param_parser.h"
#include "common/utils.h"

namespace laiseern
{
namespace control
{

class Controller
{
public:
    virtual bool UpdatePath(const std::vector<Point>& trajectory)
    {
        if (trajectory.size() < 2)
        {
            return false;
        }
        _trajectory = trajectory;
        return true;
    };

    virtual ControlCommand CalculateControl(const State& current_state) = 0;

    virtual double LookAheadDistance(double kv, double v, double l0)
    {
        _look_ahead_distance = kv * v + l0;
        return _look_ahead_distance;
    }

    virtual ~Controller() = default;

protected:
    double _look_ahead_distance{};
    std::vector<Point> _trajectory{};
};

}  // namespace control
}  // namespace laiseern

#endif  // LAISEERNL4_CONTROLLER_H
