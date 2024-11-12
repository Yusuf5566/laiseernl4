//
// Created by yuong on 23-11-14.
//

#ifndef LAISEERNL4_PURE_CONTROLLER_H
#define LAISEERNL4_PURE_CONTROLLER_H

#include "controller.h"

namespace laiseern
{
namespace control
{

class PurePursuitController : public Controller
{
public:
    PurePursuitController() = default;
    ControlCommand CalculateControl(const State& current_state) override;
    bool UpdatePath(const std::vector<Point>& trajectory) override;
    double LookAheadDistance(double kv, double v, double l0) override
    {
        _look_ahead_distance = kv * v + l0;
        return _look_ahead_distance;
    }

private:
    size_t _closest_index = 0;
    size_t _target_index{};
};

RegisterClass(Controller, PurePursuitController);

}  // namespace control
}  // namespace laiseern

#endif  // LAISEERNL4_PURE_CONTROLLER_H
