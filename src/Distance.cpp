#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32.h"
#include "cmath"

ros::Publisher distance_pub;
ros::Publisher cmd_vel_pub_1;  // To control turtle1's velocity
ros::Publisher cmd_vel_pub_2;  // To control turtle2's velocity

geometry_msgs::Pose turtle1_pose;
geometry_msgs::Pose turtle2_pose;

const double DISTANCE_THRESHOLD = 2.0;

void turtle1PoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    turtle1_pose = *msg;  //turtle1 pose
}

void turtle2PoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    turtle2_pose = *msg;  //turtle2 pose
}

void stopTurtle(int turtle_number)
{
    geometry_msgs::Twist stop_msg;  // Zero velocity message
    if (turtle_number == 1)
    {
        ROS_WARN("Turtle 1 is too close to Turtle 2! Stopping Turtle 1.");
        cmd_vel_pub_1.publish(stop_msg);  // Stop turtle1
    }
    else if (turtle_number == 2)
    {
        ROS_WARN("Turtle 2 is too close to Turtle 1! Stopping Turtle 2.");
        cmd_vel_pub_2.publish(stop_msg);  // Stop turtle2
    }
}

void calculateDistanceAndPublish()
{
    // Calculate the distance between turtle1 and turtle2
    double dx = turtle1_pose.position.x - turtle2_pose.position.x;
    double dy = turtle1_pose.position.y - turtle2_pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    // Create and publish the distance message
    std_msgs::Float32 distance_msg;
    distance_msg.data = distance;
    distance_pub.publish(distance_msg);

    ROS_INFO("Distance between turtle1 and turtle2: %f", distance);
    
    
    if (distance < DISTANCE_THRESHOLD)
    {
    ROS_WARN("Turtles are too close to each other!! Stopping turtle1");
    void stopTurtle(int turtle_number);
    
    }
}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);
    
    cmd_vel_pub_1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    cmd_vel_pub_2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    
    // Subscribe to the turtles' positions
    ros::Subscriber sub1 = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber sub2 = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);

    // Set the loop rate
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();  // Process incoming messages
        calculateDistanceAndPublish();
        loop_rate.sleep();  // Maintain the loop rate
    }

    return 0;
}


