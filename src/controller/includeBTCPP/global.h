#include"variables.h"
#ifndef GLOBAL
#define GLOBAL
extern int visualState;
extern bool stopMission;  //variable that stops the execution of the behavior tree
extern bool aborted;
extern bool paused;
extern Variables * variables;
extern bool new_variable;
extern bool completed_mission;
extern bool mission_failed;
//extern bool stop_loop;
extern int lastVisualState;
extern bool processing_belief_query;//we use this attribute tu assure that the query has done their job

#endif
