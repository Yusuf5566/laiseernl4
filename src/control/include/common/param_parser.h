/***
 *  @brief      control
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2024-04-29
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_PARAM_PARSER_H
#define LAISEERNL4_PARAM_PARSER_H
#include "ros/ros.h"
#include "vehicle_param.h"

namespace laiseern
{
namespace control
{
class ParamParser
{
public:
public:
    static ParamParser* GetInstance();
    VehicleParam GetVehicleParam() { return _vehicle_param; }
    ControllerParam GetControllerParam() { return _controller_param; }
    template <class T>
    static T ParamParse(const std::string name, T& value)
    {
        ros::NodeHandle nh("~");
        nh.getParam(name, value);
        return value;
    }

private:
    void Init();
    ParamParser();
    static ParamParser* _control_param_parser;
    VehicleParam _vehicle_param;
    ControllerParam _controller_param{};
};

}  // namespace control
}  // namespace laiseern
#endif  // LAISEERNL4_PARAM_PARSER_H
