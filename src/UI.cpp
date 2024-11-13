#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"
#include <iostream>
#include <limits>

// Declare the ROS Publisher for controlling turtle's movement
ros::Publisher pub;

void getUserInput(std::string &robot, double &linear_velocity, double &angular_velocity){

        // Display available robots
        std::cout << "Available robots: turtle1, turtle2" << std::endl;
        
        // Ask user to select a robot
        std::cout << "Enter the robot you want to control (turtle1/turtle2): ";
        std::cin>>robot;
      
        
        std::cout << "Enter the linear velocity for " << robot << " (positive number): ";
        std::cin >> linear_velocity;

        std::cout << "Enter the angular velocity for " << robot << " (number): ";
        std::cin >> angular_velocity;

}

void positionCallback(const turtlesim::Pose::ConstPtr& msg) {


}

int main(int argc, char** argv) {

    ros::Time time;
    //Initialize the ROS node
    ros::init(argc,argv, "turtlebot_control");
    ros::NodeHandle nh;
    

    //Create the service clients
    ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");
    
    
    // Spawn a new turtle (turtle2)
    turtlesim::Spawn srv1;
    srv1.request.x = 2.0;
    srv1.request.y = 1.0;
    srv1.request.theta = 0.0;
    srv1.request.name = "turtle2";
    client1.call(srv1);
     
    
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000, positionCallback);
    
    
    std::string robot;
    double linear_velocity, angular_velocity;


    geometry_msgs::Twist cmd_vel;

    
    ros::Rate loop_rate(1);
  
    while (ros::ok()){
    getUserInput(robot, linear_velocity, angular_velocity);
    
    // Change the publisher based on selected robot
    if (robot == "turtle1") {
        pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    } else if (robot == "turtle2") {
        pub = nh.advertise<geometry_msgs::Twist>("/turtle2/cmd_vel", 10);
    }
    
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angular_velocity;
    time=ros::Time::now();
    
    while(ros::ok()){
    
    if((ros::Time::now()  - time).toSec() <= 1){
    
    pub.publish(cmd_vel);
    
    }
    
    else {
    break;
    } 
    }
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub.publish(cmd_vel);
    loop_rate.sleep();
    
    }
    
    return 0;
}
