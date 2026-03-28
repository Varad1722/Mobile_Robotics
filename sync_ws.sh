#!/bin/bash
# Mobile Manipulation Workspace Sync Script
# Usage: ./sync_ws.sh [teammate_branch]

REPO=~/Mobile_Robotics
WS=~/mobile_manipulation_ws/src

echo "=== Syncing workspace from repo ==="

# If a branch argument is provided, pull from that branch first
if [ ! -z "$1" ]; then
    echo "Pulling from branch: $1"
    cd $REPO
    git fetch origin $1
    git merge origin/$1 --no-edit
fi

# Sync ros2_ws contents to workspace
echo "Copying packages to workspace..."
cp -r $REPO/ros2_ws/. $WS/

# Ignore ROS1 catkin packages
echo "Marking ROS1 packages to ignore..."
find $WS -name "CMakeLists.txt" -exec grep -l "catkin" {} \; | while read f; do
    dir=$(dirname $f)
    touch $dir/COLCON_IGNORE
    echo "  Ignoring: $dir"
done

# Rebuild workspace
echo "Building workspace..."
cd ~/mobile_manipulation_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

echo "=== Done! Workspace synced and built ==="
