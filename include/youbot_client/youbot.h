/*
 * Clase Youbot. Encapsula el mecanismo
 * de publicación de los mensajes necesarios
 * para mover las articulaciones del
 * robot Youbot.
 *
 * Autor: Daniel Fernández Villanueva
 * Mayo de 2013
 *
 */

#ifndef YOUBOT_H
#define YOUBOT_H

#include <string>
#include <cstdlib>
#include <ros/ros.h>
#include <brics_actuator/JointPositions.h>
#include <dfv/dfv.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include "youbot_client/control.h"

namespace dfv
{

class Youbot : public Control
{
    public:
        Youbot(ros::NodeHandle& node_handle_, 
               std::string arm_topic_name_ = "arm_1/arm_controller/position_command", 
               std::string gripper_topic_name_ = "arm_1/gripper_controller/position_command",
               std::string joint_states_topic_name = "joint_states",
               std::string cmd_vel_topic_name = "cmd_vel");
        ~Youbot();
        
        double joint_positions[5];
        double gripper_positions[2];
        
        dfv::Vector3 linear_vel;
        dfv::Vector3 angular_vel;
        
        void PublishMessage(bool publish_gripper = false);
        void PublishPlatformVel();
        
        dfv::Quaternion GetJointLocalQuaternion(int index) const;
        dfv::Quaternion GetJointLocalQuatFromAngle(int index, float angle) const;
        
        dfv::Vector3 GetJointPosition(int index) const;
        dfv::Vector3 GetJointPosFromAngles(unsigned int index, 
                                           const std::vector<float>& joint_angles) const;
        
        void OpenGripper();
        void CloseGripper();
        void ResetArmPosition();
        
        std::vector<float> FindAnglesForPos(dfv::Vector3& target_pos);
        
        static const float joint_min_pos[];
        static const float joint_max_pos[];
        static const float joint_ini_pos[];
        
        static const dfv::Vector3 r[];
        
        enum GripperState
        {
            closed = 0,
            open
        };
        
        std::vector<float> joint_states;
        
        // override methods
        virtual void Draw(sf::RenderWindow& window) const;
        virtual void HandleEvent(std::list<std::string>& responses, const sf::Event& event);
        
        void SetFont(const sf::Font& font);
        
    protected:
        ros::NodeHandle& node_handle;
        ros::Publisher arm_publisher;
        ros::Publisher gripper_publisher;
        ros::Publisher cmd_vel_publisher;
        
        std::vector<brics_actuator::JointValue> v_joint_values;
        std::vector<brics_actuator::JointValue> v_gripper_values;
        
        GripperState gripper_state;
        
        ros::Subscriber joint_states_subscriber;
        void JointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg);
        
        sf::String str_radius;
        sf::String str_height;
};

}

#endif
