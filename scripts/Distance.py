#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
import math

turtle1_pose = Pose()
turtle2_pose = Pose()

def turtle1_pose_callback(msg):
    """Callback function to update turtle1's pose"""
    global turtle1_pose
    turtle1_pose = msg
    
def turtle2_pose_callback(msg):
    """Callback function to update turtle2's pose"""
    global turtle2_pose
    turtle2_pose = msg
    
def publish_distance():
    """Publish the distance between turtle1 and turtle2"""
    global turtle1_pose, turtle2_pose
    
    dx = turtle1_pose.x - turtle2_pose.x
    dy = turtle1_pose.y - turtle2_pose.y
    distance = math.sqrt(dx * dx + dy * dy)
    
    distance_msg = Float32()
    distance_msg.data = distance
    distance_pub.publish(distance_msg)  
    rospy.loginfo(f"The distance between turtle1 and turtle2 is : {distance:.2f}")
    
def Distance():
    rospy.init_node('Distance')

    global distance_pub
    distance_pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10)

    rospy.Subscriber('/turtle1/pose', Pose, turtle1_pose_callback)
    rospy.Subscriber('/turtle2/pose', Pose, turtle2_pose_callback)

    loop_rate = rospy.Rate(50)  
    while not rospy.is_shutdown():
        publish_distance()    
        loop_rate.sleep()    
    
        
if __name__ == '__main__':
    try:
        Distance()
    except rospy.ROSInterruptException:
        pass
