#include "ros/ros.h"
#include "turtlesim/Spawn.h"
#include <iostream>
#include "geometry_msgs/Twist.h"

ros::Publisher pub;

void getUserInput(std::string &robot, double &linear_velocity, double &angular_velocity) {
    std::cout << "Available robots: turtle1, turtle2" << std::endl;
    std::cout << "Enter the robot you want to control (turtle1/turtle2): ";
    std::cin >> robot;
    std::cout << "Enter the linear velocity for " << robot << " (number): ";
    std::cin >> linear_velocity;
    std::cout << "Enter the angular velocity for " << robot << " (number): ";
    std::cin >> angular_velocity;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot_control");
    ros::NodeHandle nh;
    
    ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");
    
    turtlesim::Spawn srv1;
    srv1.request.x = 2.0;
    srv1.request.y = 2.0;
    srv1.request.theta = 0.0;
    srv1.request.name = "turtle2";
    client1.call(srv1);

    std::string robot;
    double linear_velocity, angular_velocity;
    geometry_msgs::Twist cmd_vel;
    
    ros::Rate loop_rate(10);
    
    pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    while (ros::ok()) {
        getUserInput(robot, linear_velocity, angular_velocity);
        
        
        if (robot == "turtle1" && pub.getTopic() != "/turtle1/cmd_vel") {
            pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        } else if (robot == "turtle2" && pub.getTopic() != "/turtle2/cmd_vel") {
            pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
        }
        
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;

        ros::Time start_time = ros::Time::now();
       
        while (ros::ok() && (ros::Time::now() - start_time).toSec() <= 1) {
            pub.publish(cmd_vel);
            loop_rate.sleep();
        }

        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        pub.publish(cmd_vel);
        loop_rate.sleep();
    }

    return 0;
}
