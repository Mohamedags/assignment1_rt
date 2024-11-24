#!/usr/bin/env python3
import rospy
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

def UI():

    rospy.init_node('turtle_control_ui', anonymous=True)
    rospy.wait_for_service('/spawn')
    try:
        spawn_turtle = rospy.ServiceProxy('/spawn', Spawn)
        
        spawn_turtle(2.0, 1.0, 0.0, 'turtle2')
        rospy.loginfo("Turtle 2 was spawned successfully!")
        
        robot = input("Enter the robot you want to control (turtle1/turtle2): ")
        if robot == "turtle1":
            global pub
            pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        elif robot == "turtle2":
            pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        else:
            rospy.logwarn("Invalid robot selection! Using turtle1 by default.")
            pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        linear_velocity = float(input(f"Enter the linear velocity for {robot} (number): "))
        angular_velocity = float(input(f"Enter the angular velocity for {robot} (number): "))
        
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = angular_velocity
        
        rospy.loginfo(f"Setting {robot} velocity: Linear Velocity = {linear_velocity} m/s, Angular Velocity = {angular_velocity} rad/s")
        
        rate = rospy.Rate(10)  
        for _ in range(50):  
            pub.publish(cmd_vel)
            rate.sleep()
        
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0
        pub.publish(cmd_vel)
        rospy.loginfo(f"{robot} has stopped.")
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        

if __name__ == '__main__':
    try:
        UI()
    except rospy.ROSInterruptException:
        pass

