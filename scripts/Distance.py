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
REPEL_VELOCITY = 1.0


def turtle1_pose_callback(msg):
    """Callback function to update turtle1's pose"""
    global turtle1_pose
    turtle1_pose = msg
    
def turtle2_pose_callback(msg):
    """Callback function to update turtle2's pose"""
    global turtle2_pose
    turtle2_pose = msg
    
def Publish_distanceAndstop_turtles():
    """Repel turtles if they are too close to each other"""
    global cmd_vel_pub_1, cmd_vel_pub_2
    
    dx = turtle1_pose.x - turtle2_pose.x
    dy = turtle1_pose.y - turtle2_pose.y
    distance = math.sqrt(dx * dx + dy * dy)
    
    distance_msg = Float32()
    distance_msg.data = distance
    distance_pub.publish(distance_msg)  
    rospy.loginfo(f"The distance between turtle1 and turtle2 is : {distance:.2f}")
    
    
    if distance < DISTANCE_THRESHOLD:
        rospy.logwarn("Turtles are too close! Repel them !!")
        
        if distance > 0: 
            unit_vector_x = dx / distance
            unit_vector_y = dy / distance
        else:
            unit_vector_x = 0
            unit_vector_y = 0
        
        repel_msg_1 = Twist()
        repel_msg_1.linear.x = REPEL_VELOCITY * unit_vector_x
        repel_msg_1.linear.y = REPEL_VELOCITY * unit_vector_y
        cmd_vel_pub_1.publish(repel_msg_1)

        repel_msg_2 = Twist()
        repel_msg_2.linear.x = -REPEL_VELOCITY * unit_vector_x
        repel_msg_2.linear.y = -REPEL_VELOCITY * unit_vector_y
        cmd_vel_pub_2.publish(repel_msg_2)

    
    if turtle1_pose.x < 1.0:  
        rospy.logwarn("Turtle1 is too close to the left boundary! Repelling.")
        repel_msg_1 = Twist()
        repel_msg_1.linear.x = REPEL_VELOCITY  
        cmd_vel_pub_1.publish(repel_msg_1)
    
    if turtle1_pose.x > 10.0:  
        rospy.logwarn("Turtle1 is too close to the right boundary! Repelling.")
        repel_msg_1 = Twist()
        repel_msg_1.linear.x = -REPEL_VELOCITY  
        cmd_vel_pub_1.publish(repel_msg_1)
    
    if turtle1_pose.y < 1.0:  
        rospy.logwarn("Turtle1 is too close to the bottom boundary! Repelling.")
        repel_msg_1 = Twist()
        repel_msg_1.linear.y = REPEL_VELOCITY  
        cmd_vel_pub_1.publish(repel_msg_1)

    if turtle1_pose.y > 10.0:  
        rospy.logwarn("Turtle1 is too close to the top boundary! Repelling.")
        repel_msg_1 = Twist()
        repel_msg_1.linear.y = -REPEL_VELOCITY  
        cmd_vel_pub_1.publish(repel_msg_1)
    
   
    if turtle2_pose.x < 1.0:  
        rospy.logwarn("Turtle2 is too close to the left boundary! Repelling.")
        repel_msg_2 = Twist()
        repel_msg_2.linear.x = REPEL_VELOCITY  
        cmd_vel_pub_2.publish(repel_msg_2)
    
    if turtle2_pose.x > 10.0:  
        rospy.logwarn("Turtle2 is too close to the right boundary! Repelling.")
        repel_msg_2 = Twist()
        repel_msg_2.linear.x = -REPEL_VELOCITY  
        cmd_vel_pub_2.publish(repel_msg_2)
    
    if turtle2_pose.y < 1.0:  
        rospy.logwarn("Turtle2 is too close to the bottom boundary! Repelling.")
        repel_msg_2 = Twist()
        repel_msg_2.linear.y = REPEL_VELOCITY  
        cmd_vel_pub_2.publish(repel_msg_2)

    if turtle2_pose.y > 10.0:  
        rospy.logwarn("Turtle2 is too close to the top boundary! Repelling.")
        repel_msg_2 = Twist()
        repel_msg_2.linear.y = -REPEL_VELOCITY  
        cmd_vel_pub_2.publish(repel_msg_2)
   
            
                
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
