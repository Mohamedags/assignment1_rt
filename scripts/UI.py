#!/usr/bin/env python3
import rospy
from turtlesim.srv import Spawn

def UI():

    rospy.init_node('turtle_control_ui', anonymous=True)
    rospy.wait_for_service('/spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        
        spawn_turtle(2.0, 1.0, 0.0, 'turtle2')
        rospy.loginfo("Turtle 2 was spawned successfully!")
    except rospy.ServiceException as e:
        rospy.logerr("Service was failed: %s", e)
        

if __name__ == '__main__':
    try:
        UI()
    except rospy.ROSInterruptException:
        pass

