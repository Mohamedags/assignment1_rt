#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
import random

turtle1_pose = Pose()
turtle2_pose = Pose()
cmd_vel_pub_1 = None
cmd_vel_pub_2 = None

DISTANCE_THRESHOLD = 2.0

def turtle1_pose_callback(msg):
    global turtle1_pose
    turtle1_pose = msg

def turtle2_pose_callback(msg):
    global turtle2_pose
    turtle2_pose = msg

def is_safe_position(x, y):
    if (x < 1.0 or x > 10.0 or y < 1.0 or y > 10.0):
        return False
    if (math.sqrt((x - turtle1_pose.x)**2 + (y - turtle1_pose.y)**2) < DISTANCE_THRESHOLD):
        return False
    if (math.sqrt((x - turtle2_pose.x)**2 + (y - turtle2_pose.y)**2) < DISTANCE_THRESHOLD):
        return False
    return True

def find_safe_position():
    while True:
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        if is_safe_position(x, y):
            return x, y

def teleport_turtle(turtle_name, x, y):
    rospy.wait_for_service('/' + turtle_name + '/teleport_absolute')
    try:
        teleport_service = rospy.ServiceProxy('/' + turtle_name + '/teleport_absolute', TeleportAbsolute)
        teleport_service(x, y, 0.0)
        rospy.loginfo(f"{turtle_name} teleported to safe position ({x}, {y})")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def Publish_distanceAndstop_turtles():
    global cmd_vel_pub_1, cmd_vel_pub_2

    dx = turtle1_pose.x - turtle2_pose.x
    dy = turtle1_pose.y - turtle2_pose.y
    distance = math.sqrt(dx * dx + dy * dy)

    distance_msg = Float32()
    distance_msg.data = distance
    distance_pub.publish(distance_msg)
    rospy.loginfo(f"The distance between turtle1 and turtle2 is : {distance:.2f}")

    if distance < DISTANCE_THRESHOLD:
        rospy.logwarn("Turtles are too close! Teleporting them to safe positions!")
        x1, y1 = find_safe_position()
        teleport_turtle('turtle1', x1, y1)
        x2, y2 = find_safe_position()
        teleport_turtle('turtle2', x2, y2)

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
