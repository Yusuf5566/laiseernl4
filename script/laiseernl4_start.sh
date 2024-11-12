# shellcheck disable=SC1113
#/!bin/bash

logDirName="./log"
ControlLogDirName="./log/control_log"
PlanningLogDirName="./log/planning_log"

if [ ! -d $logDirName ];then
  mkdir $logDirName
  mkdir $ControlLogDirName
  mkdir $PlanningLogDirName
else
echo $logDirName
  if [ ! -d $ControlLogDirName ];then
    mkdir $ControlLogDirName
  else
    echo $ControlLogDirName
  fi

  if [ ! -d $PlanningLogDirName ];then
    mkdir $PlanningLogDirName
  else
    echo $PlanningLogDirName
  fi

fi

echo "STEP: source setup.bash"
source devel/setup.bash

echo "start carla"
roslaunch carla_bridge carla_bridge.launch >./log/carla.log 2>&1 &
sleep 8

echo "start planning"
roslaunch planning planning.launch >./log/plan.log 2>&1 &
sleep 2

echo "start rviz"
rviz -d ../rviz/laiseernl4.rviz &
sleep 2

echo "start control"
roslaunch control control.launch >./log/control.log 




