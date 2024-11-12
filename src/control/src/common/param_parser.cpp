/***
 *  @brief      control
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2024-04-29
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#include "common/param_parser.h"

namespace laiseern
{
namespace control
{
ParamParser* ParamParser::_control_param_parser;

ParamParser::ParamParser()
{
    Init();
}

void ParamParser::Init()
{
    ros::param::get("~/vehicle/wheelbase", this->_vehicle_param._wheel_base);
    ros::param::get("~/vehicle/maxSteeringAngle", this->_vehicle_param._max_steering_angle);

    ros::param::get("~/controller/dt", this->_controller_param._dt);
}

ParamParser* ParamParser::GetInstance()
{
    if (_control_param_parser == nullptr)
    {
        _control_param_parser = new ParamParser();
    }
    return _control_param_parser;
}

}  // namespace control
}  // namespace laiseern
