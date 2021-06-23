#ifndef QUERY_BELIEF_H
#define QUERY_BELIEF_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <aerostack_msgs/QueryBelief.h>
#include <ros/ros.h>
#include <variables.h>
#include <global.h>
class QueryBelief : public BT::SyncActionNode
{
  public:
    QueryBelief(const std::string& name,  const BT::NodeConfiguration& config);
    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    /*This method calls the service QueryBelief and returns its success*/
    bool askQueryBelief();
    /*this method converts a string with the format (predicate(?X,?Y)) or 
    * ({X:0,Y:3}) to a vector of pairs like ({<X,-1>,<Y,-1>} if it is the 
    * first format) or ({<X,0>,<Y,3>} if it is the second format)
    */
    std::vector<std::pair<char,double>> stringToPair(std::string);
    std::vector<std::pair<char,double>> parserResponse(std::string);
    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
      return{ BT::InputPort<std::string>("query") };
    }

  private:
    std::string query_belief;
    std::string drone_id_namespace;

    aerostack_msgs::QueryBelief::Response res_activate;
    aerostack_msgs::QueryBelief::Request req_activate;
    ros::ServiceClient query_belief_srv;
      
    std::string query;
    std::string belief_expression;

    ros::ServiceClient remove_belief_srv;
    ros::NodeHandle n;

};
#endif