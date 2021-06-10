#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
import math
import numpy as np
import actionlib
import actionlib.msg
import rt2_assignment1.msg #this package
from functools import partial

from rt2_assignment1.msg import SetVel

# robot state variables
position_ = Point()
yaw_ = 0
position_ = 0
state_ = 0
# publisher
pub_ = None

# action_server
act_s = None

# velocity server
vel_s = None

# parameters for control
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_a = -3.0 
kp_d = 0.2
ub_a = 0.6
ub_d = 0.6

# Chosen a message instead of a service so it 
# can be received by multiple nodes (this + the
# one for direct control). Also, changes in the
# slider (notebook) appear in a burst but in
# a continuum, woulnd't have sense to have a 
# service request for each small temporary change
def clbk_set_vel(msg):
    global ub_a, ub_d
    
    ub_d = msg.linear
    ub_a = msg.angular
    rospy.loginfo("RECEIVED lin: {} ang: {}".format(msg.linear, msg.angular))
    

def clbk_odom(msg):
    """
    Odometry callback

    Retrieve (x,y,theta) from the Odom message.

    Args:
      msg (Odometry): odometry message.
    """
  
    global position_
    global yaw_

    # position
    position_ = msg.pose.pose.position

    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]
#-#-#-#-#

def change_state(state):
    """
    Update the current global state

    Args:
      state (int):  new state
    """

    global state_
    state_ = state
    print ('State changed to [%s]' % state_)
#-#-#-#-#

def normalize_angle(angle):
    """
    Renormalize an angle berween [-pi, pi]

    Args:
      angle (float):  input angle
      
    Returns:
      angle (float):  normalized angle
    """

    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle
#-#-#-#-#

def fix_yaw(des_yaw, next_state):
    """
    Orient the robot in a desired way

    The function is used either to orient
    the robot toward the goal (x,y) position
    or, once it's reached, to achieve the
    goal orientation. It also changes to
    a new state, depending on the current
    one (either initial heading or final
    orientation).

    Args:
      des_yaw (float):  desired yaw
      next_state (int): next state to set
    """

    err_yaw = normalize_angle(des_yaw - yaw_)
    rospy.loginfo(err_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_2_:
        twist_msg.angular.z = kp_a*err_yaw
        if np.abs(twist_msg.angular.z) > ub_a:
            twist_msg.angular.z = np.sign(twist_msg.angular.z)*ub_a
    
    pub_.publish(twist_msg)
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_2_:
        change_state(next_state)
#-#-#-#-#

def go_straight_ahead(des_pos):
    """
    Drive toward the goal

    Set the linear and angular speed
    depending on the distance to the 
    goal pose.

    Args:
      des_pos (Point):  desired (x, y) position
    """

    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))
    err_yaw = normalize_angle(desired_yaw - yaw_)
    rospy.loginfo(err_yaw)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3+0.5*err_pos
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d

        twist_msg.angular.z = kp_a*err_yaw
        pub_.publish(twist_msg)
    else: # goal pose reached
        change_state(2)

    # re-orient if needed
    if math.fabs(err_yaw) > yaw_precision_:
        change_state(0)
#-#-#-#-#

def done():
    """
    Stop the robot

    Set the robot linear and angular 
    velocity to 0.
    """

    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub_.publish(twist_msg)
#-#-#-#-#

def go_to_point(goal):
    """
    State machine implementation

    Set an appropriate behaviour depending
    on the current robot state, in orderd
    to reach the goal.
    The state machine keeps running until
    the goal is reached or the action is
    preempted (the goal gets cancelled).

    Args:
      goal (PoseActionGoal): (x,y,theta) goal pose
    """

    global act_s
    desired_position = Point()
    desired_position.x = goal.x # get the desired position from the goal received
    desired_position.y = goal.y
    des_yaw = goal.theta

    rate = rospy.Rate(20)
    success = True
    change_state(0)

    feedback = rt2_assignment1.msg.PoseFeedback()
    result = rt2_assignment1.msg.PoseResult() # actually empty for this custom action
    
    done_g = False  # whether the action has ended, either by reaching or removing the goal
    
    while not rospy.is_shutdown() and not done_g:
        if act_s.is_preempt_requested():
            feedback.status = 'Goal was preempted'
            act_s.set_preempted() # if we received the cancel we interrupt
            success = False
            func = partial(done)
            change_state(-1)    # not really useful, just to keep track of the state
            done_g = True
        else:
            if state_ == 0:
                feedback.status = "Aligning with goal"
                desired_yaw = math.atan2(desired_position.y - position_.y, desired_position.x - position_.x)
                func = partial(fix_yaw, desired_yaw, 1)
                
            elif state_ == 1:
                feedback.status = "Aligned with goal"
                func = partial(go_straight_ahead, desired_position)
                
            elif state_ == 2:
                feedback.status = "Goal [x, y] position reached"
                func = partial(fix_yaw, des_yaw, 3)
                
            elif state_ == 3:
                feedback.status = "Goal pose reached!"
                func = partial(done)
                done_g = True
                
            else:
                rospy.logerr('Unknown state!')
                
        act_s.publish_feedback(feedback) # feedback published at every step
        func()  # an appropriate function is called, depending
                # on the state

        rate.sleep()
    if success:
        rospy.loginfo('TYPE RESULT'+str(type(result)))
        result.reached = success
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)
#-#-#-#-#

def main():
    global pub_, act_s
    rospy.init_node('go_to_point')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer(
        '/go_to_point', rt2_assignment1.msg.PoseAction, go_to_point, auto_start=False) #creation of the action server
        # generally auto_start needs to go to false, or it could sometime start too early; we have to manually start
        # planning is the action binded to the callback of the action
    
    vel_s = rospy.Subscriber('/set_vel', SetVel, clbk_set_vel)
    
    act_s.start()
    
    rospy.spin()
#-#-#-#-#

if __name__ == '__main__':
    main()
