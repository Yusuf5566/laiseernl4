/***
 *  @brief      control
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2024-04-29
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_VEHICLE_PARAM_H
#define LAISEERNL4_VEHICLE_PARAM_H
#include "control_point_types.h"

namespace laiseern
{
namespace control
{

struct VehicleParam
{
    double _wheel_base = 2.5;                       // 轴距
    double _max_steering_angle = (70 / 180) * M_PI;  // 最大转角限制
};

struct ControllerParam
{
    double _dt;
};

}  // namespace control
}  // namespace laiseern
#endif  // LAISEERNL4_VEHICLE_PARAM_H