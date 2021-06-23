#include "add_belief.h"

AddBelief::AddBelief(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, {})
{
  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1"); 
  n.param<std::string>("add_belief", add_belief, "add_belief");
  add_belief_srv = n.serviceClient<aerostack_msgs::AddBelief>('/' + drone_id_namespace + '/' + add_belief);
}


BT::NodeStatus AddBelief::tick()
{
  BeliefParameters belief;
  if ( !getInput<BeliefParameters>("belief_description", belief))
  {
    throw BT::RuntimeError("missing required input [goal]");
  }
  belief_expression=belief.belief_expression;
  belief_multivalued=belief.belief_multivalued;

  bool activation_result=addBelief();
  /*If activation_result=true the service was succesfully completed otherwise it was a failure completion*/
  if(activation_result)
  {
    //setColor(COLOR_GREEN);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    //setColor(COLOR_RED);
    return BT::NodeStatus::FAILURE;
  }
}


bool AddBelief::addBelief()
{
  /* We call the service add_belief and receive the result of the call to the service*/
  req_activate.multivalued=belief_multivalued;
  req_activate.belief_expression=belief_expression;
  add_belief_srv.call(req_activate,res_activate);
  
  bool service_success=res_activate.success;
  return service_success;
   
}