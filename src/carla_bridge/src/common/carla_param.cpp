/***
 *  @brief      carla_bridge
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2024-04-28
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#include "common/carla_param.h"

namespace laiseern
{
namespace simulator
{
ParamParser* ParamParser::_carla_param_parser;

ParamParser::ParamParser()
    : _nh("~")
{
    Init();
}

void ParamParser::Init()
{
    // ego vehicle
    ros::param::get("~/map/route_start_point_index", this->_ego_vehicle_param._start_point_index);
    ros::param::get("~/ego_vehicle/vehicle_blueprint", this->_ego_vehicle_param._vehicle_blueprint);
    ros::param::get("~/ego_vehicle/color", this->_ego_vehicle_param._color);
    ros::param::get("~/ego_vehicle/target_speed", this->_ego_vehicle_param._target_speed);
    ros::param::get("~/ego_vehicle/speed_Kp", this->_ego_vehicle_param._speed_Kp);
    ros::param::get("~/ego_vehicle/speed_Ki", this->_ego_vehicle_param._speed_Ki);
    ros::param::get("~/ego_vehicle/speed_Kd", this->_ego_vehicle_param._speed_Kd);
    ros::param::get("~/ego_vehicle/accel_Kp", this->_ego_vehicle_param._accel_Kp);
    ros::param::get("~/ego_vehicle/accel_Ki", this->_ego_vehicle_param._accel_Ki);
    ros::param::get("~/ego_vehicle/accel_Kd", this->_ego_vehicle_param._accel_Kd);
    ros::param::get("~/ego_vehicle/control_time_step", this->_ego_vehicle_param._control_time_step);

    // carla
    ros::param::get("~/carla/town_name", this->_client_param._town_name);
    ros::param::get("~/carla/host", this->_client_param._host);
    ros::param::get("~/carla/port", this->_client_param._port);
    ros::param::get("~/carla/timeout", this->_client_param._timeout);
    ros::param::get("~/carla/synchronous_mode", this->_client_param._synchronous_mode);

    // traffic
    ros::param::get("~/traffic/creat_traffic_flow", this->_traffic_param._creat_traffic_flow);
    ros::param::get("~/traffic/object_count", this->_traffic_param._object_count);
    ros::param::get("~/traffic/auto_pilot", this->_traffic_param._auto_pilot);


    // map
    ros::param::get("~/map/route_start_point_index", this->_map_param._route_start_point_index);
    ros::param::get("~/map/route_dis", this->_map_param._route_dis);
    ros::param::get("~/map/route_size", this->_map_param._route_size);
    ros::param::get("~/map/tailoring", this->_map_param._tailoring);
    ros::param::get("~/map/get_route_plot", this->_map_param._get_route_plot);
    ros::param::get("~/map/tailoring_point", this->_map_param._tailoring_point);
}

ParamParser* ParamParser::GetInstance()
{
    if (_carla_param_parser == nullptr)
    {
        _carla_param_parser = new ParamParser();
    }
    return _carla_param_parser;
}

}  // namespace simulator
}  // namespace laiseern