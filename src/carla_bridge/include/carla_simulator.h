/***
 *  @brief      laiseernle
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2023-11-28
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#ifndef LAISEERNL4_CARLA_SiMULATOR_HPP
#define LAISEERNL4_CARLA_SiMULATOR_HPP
#include "common/carla_param.h"
#include "common/controller_command.h"
#include "common/utils.h"
#include <boost/smart_ptr/shared_ptr.hpp>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Waypoint.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

using namespace carla;

namespace laiseern
{
namespace simulator
{

class CarlaSimulator
{
public:
    static std::shared_ptr<CarlaSimulator> GetCarlaInstance(client::World& world, SharedPtr<client::Map> map);
    SharedPtr<client::Vehicle> GetEgoVehicle() { return _ego_vehicle; }
    SharedPtr<client::Vehicle> SetRandomVehicle(geom::Transform transform);
    SharedPtr<client::Walker> SetRandomWalker(geom::Transform transform);
    std::vector<geom::Transform> GetRecommendPoint();
    std::vector<SharedPtr<client::Waypoint>> GetMapRoute();
    void SetEgoVehicleControl(ControllerCommand& controller_command);
    void SetEgoVehicleAckermanControl(AckerControllerCommand& acker_controller_command);
    void View(geom::Transform& view_transform);
    bool CarlaDebug(std::vector<carla::geom::Transform> map_route);
    bool CarlaDebug();
    bool GetEnviromentObject();
    void SetWorldParam();

private:
    CarlaSimulator(client::World world, SharedPtr<client::Map> map);
    bool SetEgoVehicle();
    void SetSynchronousMode();

private:
    static std::shared_ptr<CarlaSimulator> _carla_simulator;
    client::World _world;
    SharedPtr<client::Map> _map = nullptr;
    SharedPtr<client::BlueprintLibrary> _blueprint_library;
    SharedPtr<client::Vehicle> _ego_vehicle;
    std::vector<geom::Transform> _recommend_point;
    std::vector<SharedPtr<client::Waypoint>> _map_route;
    client::Vehicle::Control _control;
    client::Vehicle::AckermannControl _ackerman_control;
};

}  // namespace simulator
}  // namespace laiseern

#endif  // LAISEERNL4_CARLA_SiMULATOR_HPP
