#!/usr/bin/env bash
# Временный помощник: останавливает все процессы симуляции.
pkill -9 -f 'ros2 launch uav_bringup' 2>/dev/null
pkill -9 -f 'sim_full.launch.py' 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f 'parameter_bridge' 2>/dev/null
pkill -9 -f 'bin/px4' 2>/dev/null
pkill -9 -f 'make px4_sitl' 2>/dev/null
pkill -9 -f 'cmake --build' 2>/dev/null
pkill -9 -f 'PX4_SIM_MODEL' 2>/dev/null
pkill -9 -f 'MicroXRCEAgent' 2>/dev/null
pkill -9 -f 'trajectory_trail' 2>/dev/null
pkill -9 -f 'nav2_commander_node' 2>/dev/null
pkill -9 -f 'offboard_node' 2>/dev/null
pkill -9 -f 'controller_server' 2>/dev/null
pkill -9 -f 'planner_server' 2>/dev/null
pkill -9 -f 'bt_navigator' 2>/dev/null
pkill -9 -f 'waypoint_follower' 2>/dev/null
pkill -9 -f 'behavior_server' 2>/dev/null
pkill -9 -f 'lifecycle_manager' 2>/dev/null
pkill -9 -f 'depth_to_scan' 2>/dev/null
pkill -9 -f 'odom_tf_broadcaster' 2>/dev/null
pkill -9 -f 'marker_detector' 2>/dev/null
pkill -9 -f 'human_detector' 2>/dev/null
pkill -9 -f 'target_localizer' 2>/dev/null
pkill -9 -f 'static_transform_publisher' 2>/dev/null
exit 0
