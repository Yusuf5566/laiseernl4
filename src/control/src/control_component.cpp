/***
 *  @brief      control
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2023-12-01
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#include "control_component.h"

namespace laiseern
{
namespace control
{
ControlComponent::ControlComponent()
    : _nh("~")
{
}

bool ControlComponent::Init()
{
    _control_pub = _nh.advertise<control_msgs::ackerman_control>("/control_topic", 10);
    _control_debug = _nh.advertise<control_msgs::control_debug>("/control_debug", 10);
    _vehicle_state_sub = _nh.subscribe("/State", 10, &ControlComponent::VehicleStateCallback, this);
    //    _planning_sub = _nh.subscribe("/planning_topic", 10, &ControlComponent::PlanningCallback, this);
    _control_client = _nh.serviceClient<planning_msgs::Trajectory>("/pnc_trajectory");
    _tid = std::thread(&ControlComponent::TimerCallbackRun, this);
    return true;
}

void ControlComponent::TimerCallbackRun()
{
    while (ros::ok())
    {
        RequestTrajectory();
        sleep(0.1);
    }
}

void ControlComponent::PlanningCallback(const planning_msgs::way_point& waypoint_msg)
{
    if (waypoint_msg.point.empty())
    {
        std::cout << "empty waypoint" << std::endl;
        return;
    }

    _planning_trajectory.clear();
    for (auto& i : waypoint_msg.point)
    {
        Point point;
        point.x = i.x;
        point.y = i.y;
        point.yaw = i.yaw;
        _planning_trajectory.push_back(point);
    }
}

void ControlComponent::RequestTrajectory()
{
    planning_msgs::Trajectory trajectory;

    trajectory.request.command = 1;

    bool success = _control_client.call(trajectory);

    if (success)
    {
        if (trajectory.response.trajectory.empty())
        {
            std::cout << "empty waypoint" << std::endl;
            return;
        }

        _planning_trajectory.clear();
        for (auto& i : trajectory.response.trajectory)
        {
            Point point;
            point.x = i.x;
            point.y = i.y;
            point.yaw = i.yaw;
            _planning_trajectory.push_back(point);
        }
    }
}

void ControlComponent::ControlDebug()
{
    _control_debug_msg.act_x = _vehicle_state.point.x;
    _control_debug_msg.act_y = _vehicle_state.point.y;

    _control_debug.publish(_control_debug_msg);
}

void ControlComponent::VehicleStateCallback(const carla_msgs::carla_vehicle_state& vehicle_state_msg)
{
    _vehicle_state.point.x = vehicle_state_msg.x;
    _vehicle_state.point.y = vehicle_state_msg.y;
    _vehicle_state.point.yaw = vehicle_state_msg.yaw;
    _vehicle_state.vx = vehicle_state_msg.vx;
    _vehicle_state.vy = vehicle_state_msg.vy;
    _vehicle_state.v = std::sqrt(_vehicle_state.vx * _vehicle_state.vx + _vehicle_state.vy * _vehicle_state.vy);
    _vehicle_state.ax = vehicle_state_msg.accx;
    _vehicle_state.ay = vehicle_state_msg.accy;
    _vehicle_state.acc = std::sqrt(_vehicle_state.ax * _vehicle_state.ax + _vehicle_state.ay * _vehicle_state.ay);
    _vehicle_state.jerk = vehicle_state_msg.jerk;
}

void ControlComponent::SetCommand(ControlCommand command)
{
    _control_msg.header.frame_id = "laiseernl4";
    _control_msg.header.stamp = ros::Time::now();
    _control_msg.accel = command.acc;
    _control_msg.steering_angle = command.steer;
    _control_msg.speed = command.target_v;
}

bool ControlComponent::Process()
{
    //    PurePursuitController pp_controller;
    std::string controller_type = "PurePursuitController";
    ParamParser::ParamParse("controller/type", controller_type);

    std::cout << "controller_type:" << controller_type << std::endl;
    //    exit(1);
    _controller = CreateObject(Controller, controller_type);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if (!_controller->UpdatePath(_planning_trajectory))
        {
            std::cout << "path error" << std::endl;
        }
        float v = 10;
        _controller->LookAheadDistance(0.6, v, 3);
        _command = _controller->CalculateControl(_vehicle_state);
        _command.target_v = v;
        SetCommand(_command);
        //        ControlDebug();

        std::cout << " speed=" << _control_msg.speed << " steer=" << _control_msg.steering_angle << std::endl;

        _control_pub.publish(_control_msg);

        ros::spinOnce();
        sleep(0.1);
        //        loop_rate.sleep();
    }
    return true;
}

}  // namespace control
}  // namespace laiseern