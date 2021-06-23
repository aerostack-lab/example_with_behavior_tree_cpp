#include "remove_belief.h"

RemoveBelief::RemoveBelief(const std::string& name, const BT::NodeConfiguration& config) : BT::SyncActionNode(name, {})
{
  this->belief_expression=belief_expression;
  
  n.param<std::string>("robot_namespace", drone_id_namespace, "drone1"); 
  n.param<std::string>("remove_belief", remove_belief, "remove_belief");   
  remove_belief_srv = n.serviceClient<aerostack_msgs::RemoveBelief>('/' + drone_id_namespace + '/' + remove_belief);
}


BT::NodeStatus RemoveBelief::tick()
{
  std::string belief;
  if ( !getInput<std::string>("belief_description", belief))
  {
    throw BT::RuntimeError("missing required input [goal]");
  }
  belief_expression=belief;
  
  bool activation_result=removeBelief();
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


bool RemoveBelief::removeBelief()
{
  //we call the service remove belief
  req_activate.belief_expression=belief_expression;
  remove_belief_srv.call(req_activate,res_activate);
  
  bool service_success=res_activate.success;
  
  return service_success;
}
