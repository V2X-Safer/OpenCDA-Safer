#!/bin/bash

export CARLA_HOME=$(pwd)/carla_0_9_12/
export CARLA_ROOT=$(pwd)/carla_0_9_12/
export CARLA_VERSION=0.9.12
export SUMO_HOME=/usr/share/sumo
export MESA_GL_VERSION_OVERRIDE=3.3
export SCENARIO_RUNNER_ROOT=$(pwd)/V2Xverse/simulation/scenario_runner/
export PYTHONPATH=$SCENARIO_RUNNER_ROOT
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla
