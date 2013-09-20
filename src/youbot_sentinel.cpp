#include <iostream>
#include <ros/ros.h>
#include <dfv/dfv.h>
#include <geometry_msgs/Twist.h>

void SubscriberCallback(const geometry_msgs::Twist::ConstPtr& msg);

int counter = 0;
const int counter_limit = 10;
dfv::Vector3 linear_vel;
dfv::Vector3 angular_vel;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "youbot_sentinel");
    ros::NodeHandle node_handle;
    
    ros::Subscriber subscriber = node_handle.subscribe("cmd_vel", 1, SubscriberCallback);
    ros::Publisher publisher = node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
     
    while (node_handle.ok())
    {
        std::cout << "Linear Vel: " << linear_vel << std::endl;
        std::cout << "Angular Vel: " << angular_vel << std::endl;
        counter++;
        
        if (counter > counter_limit)
        {
            geometry_msgs::Twist msg;
            msg.linear.x = 0.f;
            msg.linear.y = 0.f;
            msg.linear.z = 0.f;
            msg.angular.x = 0.f;
            msg.angular.y = 0.f;
            msg.angular.z = 0.f;
            publisher.publish(msg);
            counter = 0;
        }
        
        ros::spinOnce();
        ros::Duration(0.05f).sleep();
    }
    
    return 0;
}

void SubscriberCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    linear_vel = dfv::Vector3(msg->linear.x, msg->linear.y, msg->linear.z);
    angular_vel = dfv::Vector3(msg->angular.x, msg->angular.y, msg->angular.z);
    counter = 0;
}
