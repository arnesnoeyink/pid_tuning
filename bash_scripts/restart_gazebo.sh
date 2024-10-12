#!/bin/bash

# Beende die Gazebo-Simulation in der tmux-Sitzung
tmux send-keys -t gazebo_session C-c  # Sende STRG+C zum Stoppen

sleep 3  # Warte, bis Gazebo vollständig beendet wurde

# Starte Gazebo erneut in der tmux-Sitzung
tmux send-keys -t gazebo_session "ros2 launch scara_robot_description main.launch.py" Enter