/***
 *  @brief      control
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2023-12-01
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_CONTROL_COMPONENT_H
#define LAISEERNL4_CONTROL_COMPONENT_H
#include "controller/pure_controller.h"
#include <thread>
#include "ros/ros.h"
#include "control_msgs/ackerman_control.h"
#include "control_msgs/control_debug.h"
#include "carla_msgs/carla_vehicle_state.h"
#include "carla_msgs/carla_route.h"
#include "planning_msgs/way_point.h"
#include "planning_msgs/Trajectory.h"

namespace laiseern
{
namespace control
{
class ControlComponent
{
public:
    ControlComponent();
    bool Init();
    void SetCommand(ControlCommand command);
    bool Process();

private:
    void PlanningCallback(const planning_msgs::way_point& waypoint_msg);
    void VehicleStateCallback(const carla_msgs::carla_vehicle_state& vehicle_state_msg);
    void RequestTrajectory();

private:
    ros::NodeHandle _nh;
    ros::Publisher _control_pub;
    ros::Publisher _control_debug;
    ros::Subscriber _planning_sub;
    ros::Subscriber _route_sub;
    ros::Subscriber _vehicle_state_sub;
    ros::ServiceClient _control_client;
    std::thread _tid;
    std::vector<Point> _planning_trajectory;
    State _vehicle_state{};

    VehicleParam _vehicle;
    control_msgs::ackerman_control _control_msg;
    control_msgs::control_debug _control_debug_msg;
    ControlCommand _command{};
    Controller* _controller;
};

}  // namespace control
}  // namespace laiseern

#endif  // CONTROL_CONTROL_COMPONENT_H
