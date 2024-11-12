//
// Created by yuong on 23-10-8.
//

#include "planning_component.h"
#include "matplotlibcpp.h"

namespace laiseern
{
namespace planning
{
PlanningComponent::PlanningComponent()
    : _n("~")
{
}

void PlanningComponent::Init()
{
    _routing_sub = _n.subscribe("/carla_route", 10, &PlanningComponent::RoutingCallback, this);
    _path_pub = _n.advertise<planning_msgs::way_point>("/planning_topic", 10);
    _vehicle_state_sub = _n.subscribe<carla_msgs::carla_vehicle_state>("/vehicle_state",
        10,
        &PlanningComponent::VehicleStateCallback,
        this);
}

void PlanningComponent::VehicleStateCallback(const carla_msgs::carla_vehicle_state vehicle_state_msg)
{
    _vehicle_state = vehicle_state_msg;
}

void PlanningComponent::RoutingCallback(const carla_msgs::carla_route msg)
{
    if (msg.waypoints.empty())
    {
        std::cout << "routing_point is empty" << std::endl;
        return;
    }
    for (size_t i = 0; i < msg.waypoints.size(); ++i)
    {
        Point temp;
        temp.x = msg.waypoints[i].x;
        temp.y = msg.waypoints[i].y;
        temp.yaw = msg.waypoints[i].yaw;
        _routing_path.push_back(temp);
        // ROS_INFO("position%zu\n x:%f y:%f", i, temp.x, temp.y);
    }
}

void PlanningComponent::MainLoop()
{
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        // std::cout << "routing_path size=" << _routing_path.size() << std::endl;
        GetPath(_routing_path);
        //  PlotFigure(_routing_path);
        planning_msgs::way_point final_path;
        for (auto& i : _final_path)
        {
            if (_final_path.empty())
            {
                break;
            }
            planning_msgs::PlanningPoint temp;
            temp.x = i.x;
            temp.y = i.y;
            temp.yaw = i.yaw;
            final_path.point.push_back(temp);
        }
        _path_pub.publish(final_path);
        std::cout << "publish a path=" << _final_path.size() << std::endl;
        if (!final_path.point.empty())
        {
            ROS_INFO("published a path!");
        }
        else
        {
            ROS_INFO("final path is empty!");
        }

        loop_rate.sleep();
    }
}

bool PlanningComponent::GetPath(std::vector<Point> reference)
{
    if (reference.empty())
    {
        std::cout << "reference is empty" << std::endl;
        return false;
    }
    double closest_distance = std::numeric_limits<double>::max();
    int _closest_index = 0;
    for (int i = 0; i < _routing_path.size(); ++i)
    {
        double distance = CalDistance(_vehicle_state.x, _vehicle_state.y, _routing_path[i].x, _routing_path[i].y);
        if (closest_distance > distance)
        {
            closest_distance = distance;
            _closest_index = i;
        }
    }

    for (size_t i = _closest_index; i < 200; ++i)
    {
        Point temp;
        temp.x = reference[i].x;
        temp.y = reference[i].y;
        temp.yaw = reference[i].yaw;
        _final_path.push_back(temp);
        //        ROS_INFO("reference%zu\n x:%f y:%f", i, temp.x, temp.y);
    }
    if (_final_path.size() < 50)
    {
        std::cout << "final path is too short" << std::endl;
        return false;
    }
    return true;
}

bool PlanningComponent::PlotFigure(std::vector<Point> path)
{
    if (path.empty())
    {
        return false;
    }
    // plot map
    int n = 100;

    std::vector<double> x(n), y(n);
    for (int i = 0; i < n; ++i)
    {
        x.at(i) = path[i].x;
        y.at(i) = path[i].y;
    }

    // Set the size of output image to 1200x780 pixels
    matplotlibcpp::figure_size(1200, 780);
    // Plot line from given x and y data. Color is selected automatically.
    matplotlibcpp::plot(x, y, "r--");
    // Set x-axis to interval [0,1000000]
    matplotlibcpp::xlim(0, n / 10);
    matplotlibcpp::ylim(-5, 5);
    // Add graph title
    matplotlibcpp::title("Planning figure");
    // Save the image (file format is determined by the extension)
    matplotlibcpp::save("./image/planning.svg");

    return true;
}

}  // namespace planning
}  // namespace laiseern