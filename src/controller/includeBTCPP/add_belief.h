#ifndef ADD_BELIEF_H
#define ADD_BELIEF_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include <aerostack_msgs/AddBelief.h>
#include <ros/ros.h>
struct BeliefParameters
{
    std::string belief_expression;
    bool belief_multivalued;
};
namespace BT{
template <> inline
BeliefParameters convertFromString(StringView key)
{
    // three parameters separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        BeliefParameters output;
        output.belief_expression = convertFromString<std::string>(parts[0]);
        output.belief_multivalued = convertFromString<bool>(parts[1]);
        return output;
    }
}
}
class AddBelief : public BT::SyncActionNode
{
  public:
    int num;
    AddBelief(const std::string& name,  const BT::NodeConfiguration& config);

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;
    //this function calls the service add belief and returns its success
    bool addBelief();
    
    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<std::string>("belief_expression") };
    }
    private:
      std::string belief_expression;
      //whether to keep clashing beliefs in the memory or not
      bool belief_multivalued;
      
      ros::ServiceClient add_belief_srv;
      ros::NodeHandle n;
      aerostack_msgs::AddBelief::Response res_activate;
      aerostack_msgs::AddBelief::Request req_activate;
      std::string add_belief;
      std::string drone_id_namespace;
};
#endif