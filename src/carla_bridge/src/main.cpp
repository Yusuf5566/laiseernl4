/***
 *  @brief      carla_bridge
 *  @author     Laiseern
 *  @version    1.0
 *  @date       2023-11-28
 *  @copyright  Laiseern Co., Ltd. All rights reserved.
 ***/

#include "simulator.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "carla_bridge");

    laiseern::simulator::Simulator simulator;
    simulator.CreatCarlaSimulator();
    simulator.Init();
    simulator.Process();

    return 0;
}