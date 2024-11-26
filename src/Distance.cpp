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
const double REPEL_VELOCITY = 1.0;  

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
    
        double unit_vector_x = dx / distance;
        double unit_vector_y = dy / distance;
        
        // repelling turtles when close to each other
        geometry_msgs::Twist repel_msg_1;
        repel_msg_1.linear.x = REPEL_VELOCITY * unit_vector_x;  // Repel turtle1 
        repel_msg_1.linear.y = REPEL_VELOCITY * unit_vector_y;

        geometry_msgs::Twist repel_msg_2;
        repel_msg_2.linear.x = -REPEL_VELOCITY * unit_vector_x; // Repel turtle2 
        repel_msg_2.linear.y = -REPEL_VELOCITY * unit_vector_y;

        
        cmd_vel_pub_1.publish(repel_msg_1);
        cmd_vel_pub_2.publish(repel_msg_2);

        ROS_WARN("Turtles are too close! Applying repelling velocities.");
        }

        // Check for Turtle 1 border crossing
    if (turtle1_pose.x < 1.0) {  
        ROS_WARN("Turtle1 is too close to the left boundary!");
        geometry_msgs::Twist repel_msg_1;
        repel_msg_1.linear.x = REPEL_VELOCITY;  
        cmd_vel_pub_1.publish(repel_msg_1);
    }

    if (turtle1_pose.x > 10.0) {  
        ROS_WARN("Turtle1 is too close to the right boundary!");
        geometry_msgs::Twist repel_msg_1;
        repel_msg_1.linear.x = -REPEL_VELOCITY;  
        cmd_vel_pub_1.publish(repel_msg_1);
    }

    if (turtle1_pose.y < 1.0) { 
        ROS_WARN("Turtle1 is too close to the bottom boundary!");
        geometry_msgs::Twist repel_msg_1;
        repel_msg_1.linear.y = REPEL_VELOCITY;  
        cmd_vel_pub_1.publish(repel_msg_1);
    }

    if (turtle1_pose.y > 10.0) {  
        ROS_WARN("Turtle1 is too close to the top boundary!");
        geometry_msgs::Twist repel_msg_1;
        repel_msg_1.linear.y = -REPEL_VELOCITY;  
        cmd_vel_pub_1.publish(repel_msg_1);
    }

        // Check for Turtle 2 border crossing
    if (turtle2_pose.x < 1.0) {  
        ROS_WARN("Turtle2 is too close to the left boundary!");
        geometry_msgs::Twist repel_msg_2;
        repel_msg_2.linear.x = REPEL_VELOCITY;  
        cmd_vel_pub_2.publish(repel_msg_2);
    }

    if (turtle2_pose.x > 10.0) {  
        ROS_WARN("Turtle2 is too close to the right boundary!");
        geometry_msgs::Twist repel_msg_2;
        repel_msg_2.linear.x = -REPEL_VELOCITY;  
        cmd_vel_pub_2.publish(repel_msg_2);
    }

    if (turtle2_pose.y < 1.0) {  
        ROS_WARN("Turtle2 is too close to the bottom boundary!");
        geometry_msgs::Twist repel_msg_2;
        repel_msg_2.linear.y = REPEL_VELOCITY;  
        cmd_vel_pub_2.publish(repel_msg_2);
    }

    if (turtle2_pose.y > 10.0) {  
        ROS_WARN("Turtle2 is too close to the top boundary!");
        geometry_msgs::Twist repel_msg_2;
        repel_msg_2.linear.y = -REPEL_VELOCITY;  
        cmd_vel_pub_2.publish(repel_msg_2);
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
