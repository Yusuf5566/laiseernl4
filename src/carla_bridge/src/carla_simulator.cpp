/***
 *  @brief      carla_bridge
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2023-11-28
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#include "carla_simulator.h"
#include <utility>
using namespace std;

namespace laiseern
{
namespace simulator
{

std::shared_ptr<CarlaSimulator> CarlaSimulator::_carla_simulator;

std::shared_ptr<CarlaSimulator> CarlaSimulator::GetCarlaInstance(client::World& world, SharedPtr<client::Map> map)
{
    if (_carla_simulator == nullptr)
    {
        _carla_simulator = std::make_shared<CarlaSimulator>(CarlaSimulator(std::move(world), std::move(map)));
    }
    return _carla_simulator;
}

CarlaSimulator::CarlaSimulator(client::World world, SharedPtr<client::Map> map)
    : _world(std::move(world))
{
    _map = std::move(map);
    _blueprint_library = _world.GetBlueprintLibrary();
    _recommend_point = _map->GetRecommendedSpawnPoints();
    //    SetWorldParam();

    SetEgoVehicle();
    //    SetSynchronousMode();
}

void CarlaSimulator::SetWorldParam()
{
    rpc::WeatherParameters weather = _world.GetWeather();
    weather.fog_density = 20;
    //    weather.sun_altitude_angle = 60;
    //    weather.dust_storm = 90;
    _world.SetWeather(weather);
}

void CarlaSimulator::SetSynchronousMode()
{
    rpc::EpisodeSettings settings = _world.GetSettings();
    settings.synchronous_mode = ParamParser::GetInstance()->GetParamCarla()._synchronous_mode;
    _world.ApplySettings(settings, 10s);
}

bool CarlaSimulator::SetEgoVehicle()
{
    SetSynchronousMode();
    geom::Transform transform = _recommend_point[ParamParser::GetInstance()->GetParamEgoVehicle()._start_point_index];
    //    std::cout << "location:" << transform.location.x << "," << transform.location.y << "," << transform.location.z
    //              << " rotation=" << transform.rotation.yaw << " ," << transform.rotation.pitch << " ,"
    //              << transform.rotation.roll << std::endl;
    //    transform.location = {-6.13516, -146.16, 0.281942};
    //    transform.rotation = {89.7752, 0, 0};

    if (_blueprint_library == nullptr)
    {
        std::cout << "blueprint library is nullptr!" << std::endl;
        return false;
    }

    // 防止生成随机车辆时发生碰撞，故将自车起始点从_recommend_point中删除（不必这么设计）
    for (auto it = _recommend_point.begin(); it != _recommend_point.end(); ++it)
    {
        if (CalDistance(it->location.x, it->location.y, transform.location.x, transform.location.y) < 3)
        {
            _recommend_point.erase(it);
        }
    }

    // 指定汽车的类型和颜色
    client::ActorBlueprint vehicle_bp = *_blueprint_library->Find("vehicle.tesla.model3");
    vehicle_bp.SetAttribute("color", ParamParser::GetInstance()->GetParamEgoVehicle()._color);
    // 在这个位置生成汽车
    SharedPtr<client::Actor> actor = _world.SpawnActor(vehicle_bp, transform);
    _ego_vehicle = boost::static_pointer_cast<client::Vehicle>(actor);

    // PID参数
    rpc::AckermannControllerSettings ackermann_settings;
    ackermann_settings.speed_kp = ParamParser::GetInstance()->GetParamEgoVehicle()._speed_Kp;
    ackermann_settings.speed_ki = ParamParser::GetInstance()->GetParamEgoVehicle()._speed_Ki;
    ackermann_settings.speed_kd = ParamParser::GetInstance()->GetParamEgoVehicle()._speed_Kd;
    ackermann_settings.accel_kp = ParamParser::GetInstance()->GetParamEgoVehicle()._accel_Kp;
    ackermann_settings.accel_ki = ParamParser::GetInstance()->GetParamEgoVehicle()._accel_Ki;
    ackermann_settings.accel_kd = ParamParser::GetInstance()->GetParamEgoVehicle()._accel_Kd;
    _ego_vehicle->ApplyAckermannControllerSettings(ackermann_settings);
    View(transform);
    return true;
}

SharedPtr<client::Vehicle> CarlaSimulator::SetRandomVehicle(geom::Transform transform)
{
    std::mt19937_64 rng((std::random_device())());

    // Get a random vehicle blueprint.
    SharedPtr<client::BlueprintLibrary> blueprint_library = _world.GetBlueprintLibrary();
    auto vehicles = blueprint_library->Filter("vehicle");
    client::ActorBlueprint vehicle_bp = RandomChoice(*vehicles, rng);
    // Randomize the _color blueprint.
    if (vehicle_bp.ContainsAttribute("_color"))
    {
        auto& attribute = vehicle_bp.GetAttribute("_color");
        vehicle_bp.SetAttribute("_color", RandomChoice(attribute.GetRecommendedValues(), rng));
    }

    SharedPtr<client::Actor> actor = _world.SpawnActor(vehicle_bp, transform);
    SharedPtr<client::Vehicle> vehicle = boost::static_pointer_cast<client::Vehicle>(actor);
    vehicle->SetAutopilot(ParamParser::GetInstance()->GetParamTraffic()._auto_pilot);
    return vehicle;
}

SharedPtr<client::Walker> CarlaSimulator::SetRandomWalker(geom::Transform transform)
{
    std::mt19937_64 rng((std::random_device())());

    // Get a random vehicle blueprint.
    SharedPtr<client::BlueprintLibrary> blueprint_library = _world.GetBlueprintLibrary();
    // 这里存储的类型是unordered_map，Find相当于在里面进行了一次查找
    SharedPtr<client::BlueprintLibrary> walkers_bp = blueprint_library->Filter("walker");  //*pedestrian*

    client::ActorBlueprint walker_bp = RandomChoice(*walkers_bp, rng);
    SharedPtr<client::Actor> walker_actor = _world.SpawnActor(walker_bp, transform);
    SharedPtr<client::Walker> walker = boost::static_pointer_cast<client::Walker>(walker_actor);
    return walker;
}

std::vector<geom::Transform> CarlaSimulator::GetRecommendPoint()
{
    if (_recommend_point.empty())
    {
        _recommend_point = _map->GetRecommendedSpawnPoints();
    }
    return _recommend_point;
}

std::vector<SharedPtr<client::Waypoint>> CarlaSimulator::GetMapRoute()
{
    SharedPtr<client::Waypoint> start_waypoint = _map->GetWaypoint(
        _recommend_point[ParamParser::GetInstance()->GetParamMap()._route_start_point_index].location);

    std::vector<SharedPtr<client::Waypoint>> route;
    for (size_t i = 0; i < ParamParser::GetInstance()->GetParamMap()._route_size; ++i)
    {
        route.push_back(start_waypoint);
        std::vector<SharedPtr<client::Waypoint>> next_point =
            start_waypoint->GetNext(ParamParser::GetInstance()->GetParamMap()._route_dis);
        start_waypoint = next_point.front();
    }
    _map_route = route;

    if (ParamParser::GetInstance()->GetParamMap()._get_route_plot)
    {
        std::vector<double> x1;
        std::vector<double> y1;
        std::vector<double> x2;
        std::vector<double> y2;
        for (const auto& it : _map_route)
        {
            x1.push_back(it->GetTransform().location.x);
            y1.push_back(it->GetTransform().location.y);
        }
        x2.push_back(start_waypoint->GetTransform().location.x);
        y2.push_back(start_waypoint->GetTransform().location.y);

        PlotFigure(x1, y1, x2, y2);
    }
    //    CarlaDebug();

    return _map_route;
}

bool CarlaSimulator::GetEnviromentObject()
{
    std::vector<rpc::EnvironmentObject> environment_object = _world.GetEnvironmentObjects(1);
    for (auto& i : environment_object)
    {
        std::cout << "environment_id=" << i.id << " environment_name=" << i.name << std::endl;
    }
    return true;
}

void CarlaSimulator::SetEgoVehicleControl(ControllerCommand& controller_command)
{
    NormalizeAngle(controller_command._steer);
    _control.throttle = controller_command._throttle;
    _control.steer = controller_command._steer;
    _control.brake = controller_command._brake;
    _control.hand_brake = controller_command._hand_brake;
    _control.manual_gear_shift = controller_command._manual_gear_shift;
    _control.gear = controller_command._gear;
    _ego_vehicle->ApplyControl(_control);
}

void CarlaSimulator::SetEgoVehicleAckermanControl(AckerControllerCommand& acker_controller_command)
{
    NormalizeAngle(acker_controller_command._steer);
    _ackerman_control.speed = acker_controller_command._speed;
    _ackerman_control.steer = acker_controller_command._steer;
    _ackerman_control.acceleration = acker_controller_command._acceleration;
    _ackerman_control.steer_speed = acker_controller_command._steer_speed;
    _ackerman_control.jerk = acker_controller_command._jerk;
    _ego_vehicle->ApplyAckermannControl(_ackerman_control);
}

void CarlaSimulator::View(geom::Transform& view_transform)
{
    SharedPtr<client::Actor> spectator = _world.GetSpectator();
    view_transform.location -= 5.0f * view_transform.GetForwardVector();
    view_transform.location.z += 3.0f;
    view_transform.rotation.yaw += 0.0f;
    view_transform.rotation.pitch = -15.0f;
    spectator->SetTransform(view_transform);
}

bool CarlaSimulator::CarlaDebug(std::vector<carla::geom::Transform> map_route)
{
    if (map_route.empty())
    {
        std::cout << " map is empty!" << std::endl;
        return false;
    }

    auto carla_debug = _world.MakeDebugHelper();

    //    for (size_t i = 0; i < _map_route.size(); ++i)
    //    {
    //        carla_debug.DrawPoint(_map_route[i]->GetTransform().location);
    //    }

    for (size_t i = 0; i < map_route.size() - 1; ++i)
    {
        carla_debug.DrawLine(map_route[i].location, map_route[i + 1].location, 0.5, {0u, 0u, 5u, 20});
    }

    //    for (size_t i = 0; i < _map_route.size() - 1; ++i)
    //    {
    //        carla_debug.DrawArrow(_map_route[i]->GetTransform().location, _map_route[i + 1]->GetTransform().location,
    //        0.2);
    //    }

    return true;
}

bool CarlaSimulator::CarlaDebug()
{
    if (_map_route.empty())
    {
        std::cout << " map is empty!" << std::endl;
        return false;
    }

    auto carla_debug = _world.MakeDebugHelper();

    //    for (size_t i = 0; i < _map_route.size(); ++i)
    //    {
    //        carla_debug.DrawPoint(_map_route[i]->GetTransform().location);
    //    }

    for (size_t i = 0; i < _map_route.size() - 1; ++i)
    {
        carla_debug.DrawLine(_map_route[i]->GetTransform().location,
            _map_route[i + 1]->GetTransform().location,
            0.5,
            {0u, 0u, 5u, 20});
    }
    return true;
}

}  // namespace simulator
}  // namespace laiseern