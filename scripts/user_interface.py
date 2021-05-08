import rospy
import time
from rt2_assignment1.srv import Command

def main():
    rospy.init_node('user_interface_node')
    rospy.wait_for_service('/user_interface')
    try:
        ui_client = rospy.ServiceProxy('/user_interface', Command)
    except rospy.ServiceException as e:
        print("Service call to /user_interface failed with {}".format(e))
        
    #time.sleep(10)
    rate = rospy.Rate(20)
    x = int(input("\nPress 1 to start the robot "))
    while not rospy.is_shutdown():
        res = 'empty'
        if (x == 1):
            try:
                res = ui_client("start")
            except rospy.ServiceException as e:
                print("Service call to /user_interface failed with {}".format(e))
            x = int(input("\nPress 0 to stop the robot "))
        else:
            print("Please wait, the robot is going to stop when the position will be reached")
            try:
                res = ui_client("stop")
            except rospy.ServiceException as e:
                print("Service call to /user_interface failed with {}".format(e))
            x = int(input("\nPress 1 to start the robot "))
            
        print("Got {}".format(res))
            
if __name__ == '__main__':
    main()
