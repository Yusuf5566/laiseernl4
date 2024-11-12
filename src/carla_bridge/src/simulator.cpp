/** @file      simulator.cpp
 *  @brief      carla_bridge
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2023-11-28
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 */

#include "simulator.h"
using namespace std;

namespace laiseern
{
namespace simulator
{

Simulator::Simulator()
    : _nh("~")
{
}

bool Simulator::CreatCarlaSimulator()
{
    try
    {
        client::Client client = client::Client(ParamParser::GetInstance()->GetParamCarla()._host,
            ParamParser::GetInstance()->GetParamCarla()._port);
        client.SetTimeout(50s);
        std::cout << "Client API version : " << client.GetClientVersion() << std::endl;
        std::cout << "Server API version : " << client.GetServerVersion() << std::endl;
        client::World world = client.LoadWorld(ParamParser::GetInstance()->GetParamCarla()._town_name);
        SharedPtr<client::Map> map = world.GetMap();
        _carla = CarlaSimulator::GetCarlaInstance(world, map);
    }
    catch (const client::TimeoutException& e)
    {
        std::cout << '\n' << e.what() << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cout << "\nException: " << e.what() << std::endl;
    }
    return true;
}

bool Simulator::Init()
{
    _vehicle_state_pub = _nh.advertise<carla_msgs::carla_vehicle_state>("/State", 10);
    _carla_route_pub = _nh.advertise<carla_msgs::carla_route>("/carla_route", 10);
    _obstacle_pub = _nh.advertise<derived_object_msgs::ObjectArray>("/carla_obstacle", 10);
    _control_sub = _nh.subscribe("/control_topic", 10, &Simulator::ControllerCallback, this);
    //    _planning_sub = _nh.subscribe("/planning_topic", 10, &Simulator::PlanningCallback, this);
    //    _m_timer = _nh.createTimer(ros::Duration(0.01), &Simulator::Debug, this);
    return true;
}

void Simulator::Debug(const ros::TimerEvent& e)
{
    if (_path_debug.empty())
    {
        std::cout << "planning path is empty!" << std::endl;
    }
    _carla->CarlaDebug(_path_debug);
    //        ros::spinOnce();
}

void Simulator::ControllerCallback(const control_msgs::ackerman_control& control_msg)
{
    _acker_controller_command._steer = control_msg.steering_angle;
    _acker_controller_command._speed = control_msg.speed;
    _acker_controller_command._acceleration = control_msg.accel;
    _watch_dog = 0;
}

void Simulator::PlanningCallback(const planning_msgs::way_point& waypoint_msg)
{
    _planning_path.point = waypoint_msg.point;
    for (auto& i : waypoint_msg.point)
    {
        carla::geom::Transform transform;
        transform.location.x = i.x;
        transform.location.y = i.y;
        transform.rotation.yaw = i.yaw;
        _path_debug.push_back(transform);
    }
    //    _carla->CarlaDebug(_path_debug);
}

bool Simulator::SetCarlaRoute()
{
    _carla_route.clear();
    auto route = _carla->GetMapRoute();
    //    _carla->CarlaDebug();

    if (route.empty())
    {
        std::cout << "get map route failed!" << std::endl;
        return false;
    }

    for (const auto& it : route)
    {
        carla_msgs::carla_waypoint route_temp;
        route_temp.x = it->GetTransform().location.x;
        route_temp.y = it->GetTransform().location.y;
        route_temp.yaw = (it->GetTransform().rotation.yaw / 180) * M_PI;
        int lane_change = (int)it->GetLaneChange();
        route_temp.lane_change = lane_change;

        _carla_route.push_back(route_temp);
    }
    //    std::reverse(_carla_route.begin(), _carla_route.end());
    return true;
}

void Simulator::SetCarlaState()
{
    _ego_vehicle_state.header.stamp = ros::Time::now();
    _ego_vehicle_state.header.frame_id = "laiseernl4";
    _ego_vehicle_state.x = _carla->GetEgoVehicle()->GetTransform().location.x;
    _ego_vehicle_state.y = _carla->GetEgoVehicle()->GetTransform().location.y;
    _ego_vehicle_state.z = _carla->GetEgoVehicle()->GetTransform().location.z;
    _ego_vehicle_state.yaw = (_carla->GetEgoVehicle()->GetTransform().rotation.yaw / 180) * M_PI;
    _ego_vehicle_state.steer = (float)(70 * _carla->GetEgoVehicle()->GetControl().steer * M_PI) / 180;
    _ego_vehicle_state.vx = _carla->GetEgoVehicle()->GetVelocity().x;
    _ego_vehicle_state.vy = _carla->GetEgoVehicle()->GetVelocity().y;
    _ego_vehicle_state.accx = _carla->GetEgoVehicle()->GetAcceleration().x;
    _ego_vehicle_state.accy = _carla->GetEgoVehicle()->GetAcceleration().y;
    _ego_vehicle_state.speed_limit = _carla->GetEgoVehicle()->GetSpeedLimit();

    //    _vehicle_state.traffic_light._state =
    //        (topic::CarlaTrafficLight::State)(_carla->GetEgoVehicle()->GetTrafficLightState());
}

bool Simulator::WatchDog()
{
    _watch_dog++;
    if (_watch_dog > 5)
    {
        _acker_controller_command._steer = 0;
        _acker_controller_command._speed = 0;
        _acker_controller_command._acceleration = 0;
    }
    return true;
}

bool Simulator::CarlaRoutePub()
{
    carla_msgs::carla_route temp;
    temp.header.stamp = ros::Time::now();
    temp.header.frame_id = "laiseernl4";

    // 发布carla_route
    if (!ParamParser::GetInstance()->GetParamMap()._tailoring)
    {
        temp.waypoints = _carla_route;
        _carla_route_pub.publish(temp);
    }
    else
    {
        SetCarlaPath();

        temp.waypoints = _carla_route;
        _carla_route_pub.publish(temp);
    }
    return true;
}

bool Simulator::ObstaclePub()
{
    if (_traffic_vehicles.empty())
    {
        return false;
    }
    _obstacle_array.objects.clear();
    for (const auto& it : _traffic_vehicles)
    {
        derived_object_msgs::Object obstacle;
        obstacle.pose.position.x = it->GetTransform().location.x;
        obstacle.pose.position.y = it->GetTransform().location.y;
        obstacle.pose.position.z = it->GetTransform().location.z;

        // 绕z轴旋转yaw
        obstacle.pose.orientation.z = it->GetTransform().rotation.yaw;
        obstacle.pose.orientation.y = it->GetTransform().rotation.pitch;
        obstacle.pose.orientation.x = it->GetTransform().rotation.roll;

        obstacle.twist.linear.x = it->GetVelocity().x;
        obstacle.twist.linear.y = it->GetVelocity().y;
        obstacle.shape.dimensions.resize(2);
        obstacle.shape.dimensions[0] = it->GetBoundingBox().extent.x;
        obstacle.shape.dimensions[1] = it->GetBoundingBox().extent.y;

        _obstacle_array.objects.push_back(obstacle);
        //        std::cout << "x=" << obstacle.state.position.x << " y=" << obstacle.state.position.y
        //                  << " z=" << obstacle.state.position.z << " vx=" << obstacle.twist.linear.x
        //                  << " vy=" << obstacle.twist.linear.y << " box_x=" <<
        //                  _traffic_vehicles[i]->GetBoundingBox().extent.x
        //                  << " box_y=" << _traffic_vehicles[i]->GetBoundingBox().extent.y << std::endl;
    }
    _obstacle_pub.publish(_obstacle_array);
    return true;
}

bool Simulator::Process()
{
    std::vector<geom::Transform> recommend_point = _carla->GetRecommendPoint();
    size_t vehicle_num = recommend_point.size() / ParamParser::GetInstance()->GetParamTraffic()._object_count - 1;

    if (ParamParser::GetInstance()->GetParamTraffic()._creat_traffic_flow)
    {
        for (size_t i = 0; i < recommend_point.size() - 1; i += vehicle_num)
        {
            if (i == ParamParser::GetInstance()->GetParamEgoVehicle()._start_point_index)
            {
                continue;
            }
            geom::Transform object_transform = recommend_point[i];

            SharedPtr<client::Vehicle> one_vehicle = _carla->SetRandomVehicle(object_transform);
            _traffic_vehicles.push_back(one_vehicle);
        }
    }
    std::cout << "traffic_vehicles size=" << _traffic_vehicles.size() << std::endl;
    RunOnce();

    return true;
}

void Simulator::RunOnce()
{
    // Publish data
    geom::Transform view_transform = _carla->GetEgoVehicle()->GetTransform();
    SetCarlaRoute();

    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        WatchDog();
        _carla->SetEgoVehicleAckermanControl(_acker_controller_command);
        //        std::cout << " speed=" << _acker_controller_command._speed << " steer=" <<
        //        _acker_controller_command._steer
        //                  << std::endl;
        SetCarlaState();

        _vehicle_state_pub.publish(_ego_vehicle_state);
        ObstaclePub();
        CarlaRoutePub();
        view_transform = _carla->GetEgoVehicle()->GetTransform();
        _carla->View(view_transform);
        ros::spinOnce();
        //        loop_rate.sleep();
        sleep(0.1);
    }
}

// 裁剪path
bool Simulator::SetCarlaPath()
{
    if (_carla_route.empty())
    {
        std::cout << "get map route failed!" << std::endl;
        return false;
    }
    _carla_path.clear();
    double min_dis = std::numeric_limits<double>::max();
    double distance = 0.0;

    for (size_t i = 0; i < _carla_route.size(); ++i)
    {
        distance = CalDistance(_carla->GetEgoVehicle()->GetTransform().location.x,
            _carla->GetEgoVehicle()->GetTransform().location.y,
            _carla_route[i].x,
            _carla_route[i].y);
        if (distance < min_dis)
        {
            min_dis = distance;
            _index = i;
        }
    }

    carla_msgs::carla_waypoint temp;
    //    std::cout << " index=" << index_ << std::endl;
    for (size_t i = _index; i < _index + ParamParser::GetInstance()->GetParamMap()._tailoring_point; ++i)
    {
        temp.x = _carla_route[i].x;
        temp.y = _carla_route[i].y;
        temp.yaw = _carla_route[i].yaw;
        _carla_path.push_back(temp);
    }

    if (_carla_path.empty())
    {
        std::cout << "carla path is empty!" << std::endl;
        return false;
    }

    return true;
}

}  // namespace simulator
}  // namespace laiseern
