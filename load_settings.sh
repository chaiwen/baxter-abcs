#!/bin/sh

MYPATH=$(pwd)
echo $MYPATH

export GAZEBO_MODEL_PATH=$MYPATH/src/system_launch/world/models
export GAZEBO_RESOURCE_PATH=$MYPATH/src/system_launch/world/models

source ./devel/setup.bash
./baxter.sh sim
