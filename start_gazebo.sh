#!/bin/bash
# Updated for Gazebo Harmonic

gazebo_assets=../aeroloop_gazebo
export SDF_PATH=${gazebo_assets}/models:${SDF_PATH}
export GZ_SIM_RESOURCE_PATH=${gazebo_assets}/worlds:${GZ_SIM_RESOURCE_PATH}
export GZ_SIM_SYSTEM_PLUGIN_PATH=${gazebo_assets}/plugins/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}
gz sim -r -v 4 # ${gazebo_assets}/worlds/betaloop_iris_arducopter_demo.world

