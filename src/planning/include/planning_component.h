//
// Created by yuong on 23-10-8.
//

#ifndef AUTOPNC_PLANNING_COMPONENT_H
#define AUTOPNC_PLANNING_COMPONENT_H

#include <iostream>
#include "ros/ros.h"
#include "common/utils.h"
#include "common/param_parser.h"
#include "carla_msgs/carla_route.h"
#include "planning_msgs/PlanningPoint.h"
#include "planning_msgs/way_point.h"
#include "carla_msgs/carla_vehicle_state.h"
#include "/home/yuong/Desktop/my_project/carla/laiseernl4/devel/include/carla_msgs/carla_route.h"
#include "/home/yuong/Desktop/my_project/carla/laiseernl4/devel/include/carla_msgs/carla_vehicle_state.h"

namespace laiseern
{
namespace planning
{

struct Point
{
    double x;
    double y;
    double yaw;
};

class PlanningComponent
{
public:
    PlanningComponent();
    void Init();
    void RoutingCallback(const carla_msgs::carla_route msg);
    void VehicleStateCallback(const carla_msgs::carla_vehicle_state vehicle_state_msg);
    void MainLoop();
    bool PlotFigure(std::vector<Point> path);
    bool GetPath(std::vector<Point> reference);

private:
    ros::NodeHandle _n;
    ros::Subscriber _routing_sub;
    ros::Publisher _path_pub;
    ros::Subscriber _vehicle_state_sub;

    std::vector<Point> _routing_path;
    std::vector<Point> _final_path;

    carla_msgs::carla_vehicle_state _vehicle_state;
};

}  // namespace planning
}  // namespace laiseern

#endif  // AUTOPNC_PLANNING_COMPONENT_H
