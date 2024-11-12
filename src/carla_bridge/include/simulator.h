/***
 *  @brief      laiseernle
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2023-11-28
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_SIMULATOR_HPP
#define LAISEERNL4_SIMULATOR_HPP

#include <thread>
#include "carla_simulator.h"
#include "ros/ros.h"
#include "carla_msgs/carla_route.h"
#include "carla_msgs/carla_vehicle_state.h"
#include "carla_msgs/carla_waypoint.h"
#include "control_msgs/ackerman_control.h"
#include "planning_msgs/way_point.h"
#include "derived_object_msgs/ObjectArray.h"

namespace laiseern
{
namespace simulator
{
class Simulator
{
public:
    Simulator();
    bool Init();
    bool Process();
    bool CreatCarlaSimulator();

private:
    void RunOnce();
    bool SetCarlaRoute();
    void SetCarlaState();
    bool SetCarlaPath();

    void ControllerCallback(const control_msgs::ackerman_control& control_msg);
    void PlanningCallback(const planning_msgs::way_point& waypoint_msg);
    bool WatchDog();
    bool CarlaRoutePub();
    bool ObstaclePub();
    void Debug(const ros::TimerEvent& e);

private:
    std::shared_ptr<CarlaSimulator> _carla;
    std::string _town_name;
    bool _reach_terminal = false;
    std::vector<carla::geom::Transform> _path_debug;
    pthread_t _debug_thread{};
    ControllerCommand _controller_command;
    AckerControllerCommand _acker_controller_command;
    std::vector<SharedPtr<client::Vehicle>> _traffic_vehicles;
    size_t _index = 0;

    size_t _watch_dog = 0;

private:
    ros::NodeHandle _nh;
    ros::Publisher _vehicle_state_pub;
    ros::Publisher _carla_route_pub;
    ros::Publisher _obstacle_pub;
    ros::Subscriber _control_sub;
    ros::Subscriber _planning_sub;

    carla_msgs::carla_vehicle_state _ego_vehicle_state;
    derived_object_msgs::ObjectArray _obstacle_array;

    planning_msgs::way_point _planning_path;
    std::vector<carla_msgs::carla_waypoint> _carla_route;  // 从carla获取的path
    std::vector<carla_msgs::carla_waypoint> _carla_path;   // 裁剪后的path,也就是需要往外发布的path
    ros::Timer _m_timer;
};

}  // namespace simulator
}  // namespace laiseern

#endif  // LAISEERNL4_SIMULATOR_HPP
