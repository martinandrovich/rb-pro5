#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
export GAZEBO_PLUGIN_PATH=$DIR/collision_map_creator_plugin/build:$GAZEBO_PLUGIN_PATH 

gazebo --verbose map.world
