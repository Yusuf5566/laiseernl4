//
// Created by yuong on 23-9-26.
//
#include "planning_component.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Planning");

    laiseern::planning::PlanningComponent planning_component;
    planning_component.Init();
    planning_component.MainLoop();
    return 0;
}
