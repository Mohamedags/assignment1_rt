#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Spawn.h"



// Declare the ROS Publisher for controlling turtle's movement
ros::Publisher pub;

void positionCallback(const turtlesim::Pose::ConstPtr& msg) {


}

int main(int argc, char** argv) {

    //Initialize the ROS node
    ros::init(argc,argv, "turtlebot_subscriber");
    ros::NodeHandle nh;
    
    //Subscriber to the turtle's pose
    ros::Subscriber sub = nh.subscribe("turtle1/pose", 1000,positionCallback);

    //Create the service clients
    ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");
    
    
        // Spawn a new turtle (turtle2)
    turtlesim::Spawn srv1;
    srv1.request.x = 2.0;
    srv1.request.y = 1.0;
    srv1.request.theta = 0.0;
    srv1.request.name = "turtle2";
    client1.call(srv1);
     
    
    
    //ros::spinOnce();
    
    return 0;
}
