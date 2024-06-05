search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=ofa_robot.srdf
robot_name_in_srdf=ofa_robot
moveit_config_pkg=ofa_robot_moveit_config
robot_name=ofa_robot
planning_group_name=arm
ikfast_plugin_pkg=ofa_ikfast_plugin
base_link_name=base_link
eef_link_name=link3_end
ikfast_output_path=/home/janmarco/ws/src/ofa-weed-control-ros/ofa_ikfast_plugin/src/ofa_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
