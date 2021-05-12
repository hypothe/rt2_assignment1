#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
//#include "rt2_assignment1/Position.h" /*   Now it's an action   */
#include "rt2_assignment1/RandomPosition.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "rt2_assignment1/PoseAction.h"

bool start = false; /*!< Has the user commanded the robot to start moving */

int state = 0;  /*!< Current robot state:
                    {
                    0 -> empty_state;
                    1 -> start;
                    2 -> end_reached;
                    -1 -> interrupted_goal;
                    }
                */


/****************************************//**
* Service callback setting the start/stop
* robot state
*
* \param req (rt2_assignment1::Command::Request &):
*   Service request, containing the command (string).
* \param res (rt2_assignment1::Command::Response &):
*   Service response, the value of the 'start' state (bool).
*
********************************************/ 
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
  if (req.command == "start"){
    state = 1;
  }
  else {
    state = -1;
  }
  return true;
}

/****************************************//**
* Callback launched at the end of the action
*
* This function is launched once the goal of
* of the action is reached.
*
* \param goal_state (const actionlib::SimpleClientGoalState&):
*   The goal state reached.
* \param result (const rt2_assignment1::PoseResultConstPtr&):
*   The result of the action.
*
********************************************/ 
void doneCllbck(const actionlib::SimpleClientGoalState& goal_state,
                const rt2_assignment1::PoseResultConstPtr& result){    
  state = 2; /* Goal reached state */
}

/****************************************//**
* Function called once the goal is issued.
*
* In this case it's empty.
*
********************************************/ 
void activeCllbck(){return;}

/****************************************//**
* Called each time the feedback is updated.
*
* In this case it's empty.
*
* \param feedback (const rt2_assignment1::PoseFeedbackConstPtr&):
*   The action feedback.
*
********************************************/ 
void feedbackCllbck(const rt2_assignment1::PoseFeedbackConstPtr& feedback){
  //ROS_INFO("FEEDBACK: %s", feedback->status.c_str());
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_machine");
  ros::NodeHandle n;
   
  ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
  ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
  /* Action client of the Pose */
  actionlib::SimpleActionClient<rt2_assignment1::PoseAction> ac("go_to_point", true);
   
  rt2_assignment1::RandomPosition rp;
  /*  Limits of the random position request  */
  rp.request.x_max = 5.0;
  rp.request.x_min = -5.0;
  rp.request.y_max = 5.0;
  rp.request.y_min = -5.0;
  rt2_assignment1::PoseGoal goal;
  //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){ // timeout can be set also for the waiforserrver
     ROS_INFO("Waiting for the go_to_point action server to come up");
   }
   
  while(ros::ok()){
    ros::spinOnce();
    switch (state){
      case 1:   /*  Go toward the goal, user called for 'start'  */
        client_rp.call(rp); /*  retrieve random pose  */
        goal.x = rp.response.x;
        goal.y = rp.response.y;
        goal.theta = rp.response.theta;
        ROS_DEBUG("\nGoing to the position: x= %f y= %f theta = %f\n", goal.x, goal.y, goal.theta);
        /*  Send the Random Pose received as the Goal for the Action  */
        ac.sendGoal(goal, &doneCllbck, &activeCllbck, &feedbackCllbck);
        state = 0;
        break;
      case -1:  /*  User called for the 'stop'  */
        ac.cancelGoal();  /*  Stop the action cancelling the Goal  */
        state = 0;
        break;    
      case 2:   /*  Action ended */
        actionlib::SimpleClientGoalState goal_state = ac.getState();
        if(goal_state == actionlib::SimpleClientGoalState::SUCCEEDED){
          /*  Action ended because the goal was reached  */
          ROS_DEBUG("Pose goal reached");
          state = 1;
        }
        else if(goal_state == actionlib::SimpleClientGoalState::PREEMPTED){
          /*  Action ended because the action was preeempted  */
          /*  The robot is already standing still, go to "empty" state  */
          ROS_DEBUG("Goal canceled");
          state = 0;
        }
        else{
          /*  Action ended because the robot faileed to reach the goal  */
          ROS_DEBUG("Failed to reach the goal pose");
          state = 0;
        }
    }
  }
  return 0;
}
