#ifndef MOVEBASE_BT_NODES_H
#define MOVEBASE_BT_NODES_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include<ros/ros.h>
#include <behavior_coordinator_msgs/StartTask.h>
#include <behavior_coordinator_msgs/TaskCommand.h>
#include <behavior_coordinator_msgs/TaskStopped.h>
#include <behavior_coordinator_msgs/StopTask.h>
#include <behavior_coordinator_msgs/ListOfRunningTasks.h>
#include <QMessageBox> 
#include "variables.h"
#include "global.h"
struct Parameters
{
    std::string task_name, task_parameters;
    int task_priority;
};
namespace BT{
template <> inline
Parameters convertFromString(StringView key)
{
    // three parameters separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Parameters output;
        output.task_name     = convertFromString<std::string>(parts[0]);
        output.task_parameters     = convertFromString<std::string>(parts[1]);
        output.task_priority = convertFromString<int>(parts[2]);
        return output;
    }
}
}

// It requires the input port "Parameters".
class BehaviorTask : public BT::AsyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    BehaviorTask(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<Parameters>("task_description") };
    }
    BT::NodeStatus tick() override;

    virtual void halt() override;
    //this function starts a task and returns if the task was succesfully started and an error msg
    behavior_coordinator_msgs::StartTask::Response activateBehaviorTask();
      
    std::string getTaskName();
    bool checkTaskVariables(std::string parameters);
    std::string substitutionVariables(std::string parameters);
  
  private:
    std::atomic_bool _halt_requested;

    std::string task_name;
    std::string task_parameters;
    int task_priority;
    
    ros::Subscriber tasks_ended;
    ros::NodeHandle n;
    ros::ServiceClient activate_task_srv;
    behavior_coordinator_msgs::StartTask::Response res_activate;
    behavior_coordinator_msgs::StartTask::Request req_activate;
    
    std::string task;
    std::string drone_id_namespace;
    std::string task_stopped;

    bool stop_loop;
    bool finished_correctly;
    void CallbackBehaviorActivationFinished(const behavior_coordinator_msgs::TaskStopped &msg);
    void stopExecution();
};

#endif   // MOVEBASE_BT_NODES_H