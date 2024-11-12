/***
 *  @brief      Project
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2024-06-17
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_POINT_TYPE_H
#define LAISEERNL4_POINT_TYPE_H
#include <glog/logging.h>

namespace laiseern
{
namespace control
{

struct Point
{
    double x;  // x坐标
    double y;  // y坐标
    double z;
    double yaw;  // 航向角
    double cur;
};

struct State
{
    Point point;

    double v;
    double vx;
    double vy;
    double acc;
    double ax;
    double ay;
    double jerk;
};

struct ControlCommand
{
    double steer;  // front wheel angle
    double acc;
    double target_v;
};

}  // namespace control
}  // namespace laiseern

#endif  // LAISEERNL4_POINT_TYPE_H
