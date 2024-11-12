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
