/***
 *  @brief      carla_bridge
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2023-11-29
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_PARAM_PARSER_H
#define LAISEERNL4_PARAM_PARSER_H

#include <cstdint>
#include <string>
#include <vector>
#include "ros/ros.h"

namespace laiseern
{
namespace simulator
{

struct ParamEgoVehicle
{
    int _start_point_index;
    std::string _vehicle_blueprint;
    std::string _color;
    float _target_speed;
    float _speed_Kp;
    float _speed_Ki;
    float _speed_Kd;
    float _accel_Kp;
    float _accel_Ki;
    float _accel_Kd;
    float _control_time_step;
};

struct ParamCarla
{
    std::string _town_name;
    std::string _host;
    int _port;
    float _timeout = 40;
    bool _synchronous_mode = false;
};

struct ParamTraffic
{
    bool _creat_traffic_flow = false;
    int _object_count = 0;
    bool _auto_pilot = false;
};

struct ParamMap
{
    int _route_size;
    double _route_dis;
    int _route_start_point_index;
    bool _get_route_plot;
    bool _tailoring;
    int _tailoring_point;
};

class ParamParser
{
public:
    static ParamParser* GetInstance();
    void Init();
    ParamCarla GetParamCarla() { return _client_param; }
    ParamEgoVehicle GetParamEgoVehicle() { return _ego_vehicle_param; }
    ParamMap GetParamMap() { return _map_param; }
    ParamTraffic GetParamTraffic() { return _traffic_param; }

    template <class T>
    static T ParamParse(const std::string name, T& value)
    {
        ros::NodeHandle nh("~");
        nh.getParam(name, value);
        return value;
    }

private:
    ParamParser();
    static ParamParser* _carla_param_parser;
    ParamEgoVehicle _ego_vehicle_param{};
    ParamCarla _client_param{};
    ParamMap _map_param{};
    ParamTraffic _traffic_param{};
    ros::NodeHandle _nh;
};

}  // namespace simulator
}  // namespace laiseern

#endif  // LAISEERNL4_PARAM_PARSER_H
