#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"


ros::Publisher distance_pub; 

turtlesim::Pose turtle1_pose;
turtlesim::Pose turtle2_pose;

ros::Publisher cmd_vel_pub_1;  
ros::Publisher cmd_vel_pub_2;  

const double DISTANCE_THRESHOLD = 2.0;

void turtle1PoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    turtle1_pose = *msg;  
}

void turtle2PoseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    turtle2_pose = *msg;  
}

void stopTurtle()
{
    geometry_msgs::Twist stop_msg;  
    stop_msg.linear.x = 0;
    stop_msg.angular.z = 0;
}

void calculateDistance_PublishAndCheck(std::string robot)
{
    
    double dx = turtle1_pose.x - turtle2_pose.x;
    double dy = turtle1_pose.y - turtle2_pose.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    
    std_msgs::Float32 distance_msg;
    distance_msg.data = distance;
    distance_pub.publish(distance_msg);
    ROS_INFO("The distance between turtle1 and turtle2 is : %f", distance);


    bool stop_turtle1 = false, stop_turtle2 = false;
    if (distance < DISTANCE_THRESHOLD) {
        
        if (robot == "turtle1") {
            stop_turtle1 = true; 
            ROS_WARN("Turtles are too close to each other! Stopping turtle 1");
            
        } else if (robot == "turtle2") {
            stop_turtle2 = true; 
            ROS_WARN("Turtles are too close to each other! Stopping turtle 2");
        }
    }
    
    if (turtle1_pose.x > 10.0 || turtle1_pose.x < 1.0 || turtle1_pose.y > 10.0 || turtle1_pose.y < 1.0) {
        ROS_WARN("Turtle1 is too close to the boundary!");
        stop_turtle1 = true;
    }

    if (turtle2_pose.x > 10.0 || turtle2_pose.x < 1.0 || turtle2_pose.y > 10.0 || turtle2_pose.y < 1.0) {
        ROS_WARN("Turtle2 is too close to the boundary!");
        stop_turtle2 = true;
    }
    
    geometry_msgs::Twist stop_msg;  
    if (stop_turtle1) {
        cmd_vel_pub_1.publish(stop_msg);
    }
        
    if (stop_turtle2) {
        cmd_vel_pub_2.publish(stop_msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "distance_node");
    ros::NodeHandle nh;
    
    distance_pub = nh.advertise<std_msgs::Float32>("/turtle_distance", 10);
    
    cmd_vel_pub_1 = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    cmd_vel_pub_2 = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    
    ros::Subscriber sub1 = nh.subscribe("/turtle1/pose", 10, turtle1PoseCallback);
    ros::Subscriber sub2 = nh.subscribe("/turtle2/pose", 10, turtle2PoseCallback);
    
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();  
        calculateDistance_PublishAndCheck("turtle1");
        calculateDistance_PublishAndCheck("turtle2");
        ros::spinOnce();
        loop_rate.sleep(); 
    }
    
    return 0;
}
