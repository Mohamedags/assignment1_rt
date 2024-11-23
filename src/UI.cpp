#include "ros/ros.h"
#include "turtlesim/Spawn.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot_control");
    ros::NodeHandle nh;
    
    ros::ServiceClient client1 = nh.serviceClient<turtlesim::Spawn>("/spawn");
    
    turtlesim::Spawn srv1;
    srv1.request.x = 2.0;
    srv1.request.y = 1.0;
    srv1.request.theta = 0.0;
    srv1.request.name = "turtle2";
    client1.call(srv1);
        
    ros::spin();

    return 0;
}
