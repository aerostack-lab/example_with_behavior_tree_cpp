#ifndef REMOVE_BELIEF_H
#define REMOVE_BELIEF_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <aerostack_msgs/RemoveBelief.h>
#include <ros/ros.h>

class RemoveBelief : public BT::SyncActionNode
{
  public:
    RemoveBelief(const std::string& name,  const BT::NodeConfiguration& config);
    /*This method calls the service RemoveBelief and returns its success*/
    bool removeBelief();
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
      return{ BT::InputPort<std::string>("belief_expression") };
    }

  private:
    std::string belief_expression;

    ros::ServiceClient remove_belief_srv;
    ros::NodeHandle n;
    
    aerostack_msgs::RemoveBelief::Response res_activate;
    aerostack_msgs::RemoveBelief::Request req_activate;

    std::string remove_belief;
    std::string drone_id_namespace;
};
#endif