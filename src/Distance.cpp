#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"


ros::Publisher distance_pub; 

turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;

void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    turtle1_pose = *msg;  
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    turtle2_pose = *msg;  
}

void calculateDistanceAndPublish()
{
    
    double dx = turtle1_pose.x - turtle2_pose.x;
    double dy = turtle1_pose.y - turtle2_pose.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    
    std_msgs::Float32 distance_msg;
    distance_msg.data = distance;
    distance_pub.publish(distance_msg);
    ROS_INFO("The distance between turtle1 and turtle2 is : %f", distance);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;
    distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);
    
    
    ros::Subscriber sub1 = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber sub2 = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();  
        calculateDistanceAndPublish();
        loop_rate.sleep(); 
    }
    
    return 0;
}
