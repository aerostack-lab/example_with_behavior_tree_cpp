#include "variables.h"
#include "global.h"
#include <stdio.h>
#include <ros/ros.h>
#include "behavior_task.h"
#include "add_belief.h"
#include "remove_belief.h"
#include "query_belief.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
Variables * variables;
bool new_variable;
int visualState;
bool stopMission; 
bool aborted;
bool paused;
bool completed_mission;
bool mission_failed;
int lastVisualState;
//bool stop_loop;
bool processing_belief_query;//we use this attribute tu assure that the query has done their job

int main(int argc, char *argv[])
{
  sleep(2);
  ros::init(argc,argv,"behavior_tree_control_keyboard"); //ros node started.

  //obtaining mission file route 
  std::string mission_configuration_folder,file_route;
  ros::NodeHandle nh("~");
  nh.param<std::string>("mission_configuration_folder", mission_configuration_folder, "$(env AEROSTACK_STACK)/configs/drone1");
  std::cout << "Mission configuration folder: " << mission_configuration_folder << std::endl;
  file_route = mission_configuration_folder + "/mission.xml";
  
  sleep(4);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<BehaviorTask>("BehaviorTask");  
  factory.registerNodeType<AddBelief>("AddBelief");
  factory.registerNodeType<RemoveBelief>("RemoveBelief");
  factory.registerNodeType<QueryBelief>("QueryBelief");

  auto tree = factory.createTreeFromFile(file_route);
  
  std::cout<< "Connect Groot";
  BT::PublisherZMQ  publisher_zmq(tree);
  sleep(5);
  
  while(tree.tickRoot() == BT::NodeStatus::RUNNING)
  {
    sleep(2);
  }

  return 0;
}
