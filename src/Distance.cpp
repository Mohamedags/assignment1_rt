#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32.h"
#include "cmath"

ros::Publisher distance_pub;

geometry_msgs::Pose turtle1_pose;
geometry_msgs::Pose turtle2_pose;

void turtle1PoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    turtle1_pose = *msg;  //turtle1 pose
}

void turtle2PoseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    turtle2_pose = *msg;  //turtle2 pose
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
}


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;

    distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);

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


