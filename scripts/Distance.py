#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math


turtle1_pose = Pose()
turtle2_pose = Pose()
cmd_vel_pub_1 = None
cmd_vel_pub_2 = None


DISTANCE_THRESHOLD = 2.0

turtle1_moving = True
turtle2_moving = True


def turtle1_pose_callback(msg):
    """Callback function to update turtle1's pose"""
    global turtle1_pose
    turtle1_pose = msg
    
def turtle2_pose_callback(msg):
    """Callback function to update turtle2's pose"""
    global turtle2_pose
    turtle2_pose = msg
    
def Publish_distanceAndstop_turtles():
    """Stop both turtles if they are too close to each other"""
    global cmd_vel_pub_1, cmd_vel_pub_2, turtle1_moving, turtle2_moving
    
    dx = turtle1_pose.x - turtle2_pose.x
    dy = turtle1_pose.y - turtle2_pose.y
    distance = math.sqrt(dx * dx + dy * dy)
    
    distance_msg = Float32()
    distance_msg.data = distance
    distance_pub.publish(distance_msg)  
    rospy.loginfo(f"The distance between turtle1 and turtle2 is : {distance:.2f}")
    
    
    if distance < DISTANCE_THRESHOLD:
        rospy.logwarn("Turtles are too close! Stopping the moving turtle.")
        
        if turtle1_moving:
            stop_msg = Twist()  
            cmd_vel_pub_1.publish(stop_msg)
            turtle1_moving = False  
            
        if turtle2_moving:
            stop_msg = Twist()  
            cmd_vel_pub_2.publish(stop_msg)
            turtle2_moving = False  
            
    if (turtle1_pose.x < 1 or turtle1_pose.x > 10.0 or 
        turtle1_pose.y < 1 or turtle1_pose.y > 10.0):
        rospy.logwarn("Turtle1 is too close to the boundary! Stopping turtle1.")
        if turtle1_moving:
            stop_msg = Twist()  
            cmd_vel_pub_1.publish(stop_msg)
            turtle1_moving = False  

    if (turtle2_pose.x < 1 or turtle2_pose.x > 10.0 or 
        turtle2_pose.y < 1 or turtle2_pose.y > 10.0):
        rospy.logwarn("Turtle2 is too close to the boundary! Stopping turtle2.")
        if turtle2_moving:
            stop_msg = Twist()  
            cmd_vel_pub_2.publish(stop_msg)
            turtle2_moving = False  

    if distance >= DISTANCE_THRESHOLD and not (turtle1_pose.x < 1 or turtle1_pose.x > 10.0 or 
                                                turtle1_pose.y < 1 or turtle1_pose.y > 10.0):
        if not turtle1_moving:
            move_msg = Twist()
            move_msg.linear.x = 1.0  
            cmd_vel_pub_1.publish(move_msg)
            turtle1_moving = True  

    if distance >= DISTANCE_THRESHOLD and not (turtle2_pose.x < 1 or turtle2_pose.x > 10.0 or 
                                                turtle2_pose.y < 1 or turtle2_pose.y > 10.0):
        if not turtle2_moving:
            move_msg = Twist()
            move_msg.linear.x = 1.0  
            cmd_vel_pub_2.publish(move_msg)
            turtle2_moving = True   
            
                
def Distance():
    rospy.init_node('Distance')

    global cmd_vel_pub_1, cmd_vel_pub_2, distance_pub
    cmd_vel_pub_1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    cmd_vel_pub_2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    
    distance_pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10)

    rospy.Subscriber('/turtle1/pose', Pose, turtle1_pose_callback)
    rospy.Subscriber('/turtle2/pose', Pose, turtle2_pose_callback)

    loop_rate = rospy.Rate(50)  
    while not rospy.is_shutdown():
        Publish_distanceAndstop_turtles()    
        loop_rate.sleep()    
    
        
if __name__ == '__main__':
    try:
        Distance()
    except rospy.ROSInterruptException:
        pass
