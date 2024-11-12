/***
 *  @brief      carla_bridge
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2023-11-30
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_CONTROLLER_COMMAND_H
#define LAISEERNL4_CONTROLLER_COMMAND_H

#include <cstdint>
namespace laiseern
{
namespace simulator
{

struct ControllerCommand
{
    float _throttle = 0.0f;
    float _steer = 0.0f;
    float _brake = 0.0f;
    bool _hand_brake = false;
    bool _reverse = false;
    bool _manual_gear_shift = false;
    int32_t _gear = 0;
};

struct AckerControllerCommand
{
    float _steer = 0.0f;
    float _steer_speed = 0.0f;
    float _speed = 0.0f;
    float _acceleration = 0.0f;
    float _jerk = 0.0f;
};

}  // namespace simulator
}  // namespace laiseern

#endif  // LAISEERNL4_CONTROLLER_COMMAND_H
