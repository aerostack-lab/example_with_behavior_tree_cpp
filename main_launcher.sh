#!/bin/bash

NUMID_DRONE=111
DRONE_SWARM_ID=1
MAV_NAME=hummingbird_adr
export APPLICATION_PATH="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

#---------------------------------------------------------------------------------------------
# Groot
#---------------------------------------------------------------------------------------------
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# Groot		                                                                       ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Groot"  --command "bash -c \"
./Groot/build/Groot;
exec bash\"" \

#---------------------------------------------------------------------------------------------
# INTERNAL PROCESSES
#---------------------------------------------------------------------------------------------
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# Motor speed controller                                                                      ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Motor speed controller"  --command "bash -c \"
roslaunch motor_speed_controller motor_speed_controller.launch --wait \
  namespace:=drone$NUMID_DRONE \
  mav_name:=hummingbird;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Belief Memory Viewer                                                                        ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Belief memory Viewer" --command "bash -c \"
roslaunch belief_memory_viewer belief_memory_viewer.launch --wait \
  robot_namespace:=drone$NUMID_DRONE \
  drone_id:=$NUMID_DRONE ;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Gazebo Interface                                                                            ` \
`#---------------------------------------------------------------------------------------------` \
  --tab --title "Gazebo Interface" --command "bash -c \"
roslaunch gazebo_interface gazebo_interface.launch --wait \
    robot_namespace:=drone$NUMID_DRONE \
    drone_id:=$DRONE_SWARM_ID \
    mav_name:=$MAV_NAME;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Belief Manager                                                                              ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Belief Manager" --command "bash -c \"
roslaunch belief_manager_process belief_manager_process.launch --wait \
  drone_id_namespace:=drone$NUMID_DRONE \
  drone_id:=$NUMID_DRONE \
  config_path:=${APPLICATION_PATH}/configs/mission;
exec bash\""  \
`#---------------------------------------------------------------------------------------------` \
`# Common Belief Updater                                                                              ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Common Belief Updater" --command "bash -c \"
roslaunch common_belief_updater_process common_belief_updater_process.launch --wait \
  drone_id_namespace:=drone$NUMID_DRONE \
  drone_id:=$NUMID_DRONE;
exec bash\""  &

gnome-terminal \
`#---------------------------------------------------------------------------------------------` \
`# Basic Quadrotor Behaviors                                                                   ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Basic Quadrotor Behaviors" --command "bash -c \"
roslaunch basic_quadrotor_behaviors basic_quadrotor_behaviors.launch --wait \
  namespace:=drone$NUMID_DRONE;
exec bash\"" \
`#---------------------------------------------------------------------------------------------` \
`# Quadrotor Motion With PID Control                                                           ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Quadrotor Motion With PID Control" --command "bash -c \"
roslaunch quadrotor_motion_with_pid_control quadrotor_motion_with_pid_control.launch --wait \
    namespace:=drone$NUMID_DRONE \
    robot_config_path:=${APPLICATION_PATH}/configs/drone$NUMID_DRONE \
    uav_mass:=0.75;
exec bash\""  &

sleep 3
gnome-terminal \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Coordinator                                                                       ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "Behavior coordinator" --command "bash -c \" sleep 2;
roslaunch behavior_coordinator behavior_coordinator.launch --wait \
  robot_namespace:=drone$NUMID_DRONE \
  catalog_path:=${APPLICATION_PATH}/configs/mission/behavior_catalog.yaml;
exec bash\""  &

sleep 15

gnome-terminal \
`#---------------------------------------------------------------------------------------------` \
`# alphanumeric_viewer                                                                         ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "alphanumeric_viewer"  --command "bash -c \"
roslaunch alphanumeric_viewer alphanumeric_viewer.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    my_stack_directory:=${APPLICATION_PATH};
exec bash\"" \
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# alphanumeric_viewer                                                                         ` \
`#---------------------------------------------------------------------------------------------` \
--tab --title "alphanumeric_behavior_viewer"  --command "bash -c \"
roslaunch alphanumeric_behavior_viewer alphanumeric_behavior_viewer.launch --wait \
    drone_id_namespace:=drone$NUMID_DRONE \
    catalog_path:=${APPLICATION_PATH}/configs/mission/behavior_catalog.yaml;
exec bash\"" &
gnome-terminal  \
`#---------------------------------------------------------------------------------------------` \
`# Behavior Tree CPP                                                                   ` \
`#---------------------------------------------------------------------------------------------` \
  --tab --title "Behavior Tree CPP" --command "bash -c \"
roslaunch example_with_behavior_tree_cpp behavior_tree_cpp.launch --wait \
  robot_namespace:=drone$NUMID_DRONE \
  drone_id:=$NUMID_DRONE \
  mission_configuration_folder:=${APPLICATION_PATH}/configs/mission \
  catalog_path:=${APPLICATION_PATH}/configs/mission/behavior_catalog.yaml;
exec bash\""  &

rqt_image_view /hummingbird_adr1/camera_front/image_raw/compressed


