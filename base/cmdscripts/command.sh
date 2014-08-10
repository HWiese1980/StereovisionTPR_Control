#!/bin/bash

source /opt/ros/hydro/setup.bash
cd ~/tpr
source devel/setup.bash

echo "Sending Command '$*' to headunit"
rostopic pub /me/commands std_msgs/String -1 -- "$*"