#!/bin/bash

TEST_TIMEOUT=300

# 1. Kill
echo "Cleaning environment..."
pkill -9 -f ros
pkill -9 -f gz
pkill -9 -f rviz
sleep 2


echo "Sourcing workspace..."
source install/setup.bash


for cycle in {1..10}; do
  echo "=========================================="
  echo "Starting Test Cycle $cycle / 10"
  echo "=========================================="


  echo "Launching Robot Spawn..."
  ros2 launch nav_map spawn_robot.launch.py &
  PID_SPAWN=$!

  echo "Waiting 3 seconds for Gazebo to initialize..."
  sleep 3

  echo "Launching Navigation Stack..."
  ros2 launch turtlebot3_navigation2 navigation2.launch.py \
      use_sim_time:=True \
      params_file:=/home/tahmid/thesis_nav_ws/src/nav_map/src/params/hybrid.yaml \
      map:=/home/tahmid/thesis_nav_ws/src/nav_map/src/maps/simple_map.yaml &
  PID_NAV=$!

  echo "Waiting 8 seconds for Nav2 to stabilize..."
  sleep 10

  echo "Starting Python Navigation Test..."
  echo "Cycle $cycle:" >> /home/tahmid/thesis_nav_ws/src/nav_map/src/navigation_goal_metrics.txt
  timeout "$TEST_TIMEOUT" python3 '/home/tahmid/thesis_nav_ws/src/nav_map/src/run_navigation_test.py'
  TEST_RC=$?
  if [[ $TEST_RC -eq 124 ]]; then
    echo "Cycle $cycle timed out after ${TEST_TIMEOUT}s"
    echo "Cycle $cycle: TIMEOUT after ${TEST_TIMEOUT}s" >> /home/tahmid/thesis_nav_ws/src/nav_map/src/navigation_goal_metrics.txt
  elif [[ $TEST_RC -ne 0 ]]; then
    echo "Cycle $cycle: Test exited with code $TEST_RC" >> /home/tahmid/thesis_nav_ws/src/nav_map/src/navigation_goal_metrics.txt
  fi

  echo "Python script finished. Shutting down all processes..."
  if [[ -n "${PID_SPAWN:-}" ]]; then
    kill "$PID_SPAWN" 2>/dev/null || true
  fi
  if [[ -n "${PID_NAV:-}" ]]; then
    kill "$PID_NAV" 2>/dev/null || true
  fi

  pkill -9 -f nav2_
  pkill -9 -f ros
  pkill -9 -f gz
  pkill -9 -f rviz

  echo "Test Cycle $cycle Complete."
  sleep 2
done

echo "All 10 Test Cycles Complete."