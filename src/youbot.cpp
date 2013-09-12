#include <youbot_client/youbot.h>

const float Youbot::joint_min_pos[] = {0.0100692f, 0.0100692f, -5.02655f, 0.0221239f, 0.110619f};
const float Youbot::joint_max_pos[] = {5.84014f, 2.61799f, -0.015708f, 3.4292f, 5.64159f};
const float Youbot::joint_ini_pos[] = {2.959675f, 1.144533f, -2.57124f, 1.811086f, 2.91237f};

Youbot::Youbot(ros::NodeHandle& node_handle_, 
               std::string arm_topic_name_, 
               std::string gripper_topic_name):
    node_handle(node_handle_)
{
    this->arm_publisher = 
        this->node_handle.advertise<brics_actuator::JointPositions>(arm_topic_name_, 1);
    this->gripper_publisher = 
        this->node_handle.advertise<brics_actuator::JointPositions>(gripper_topic_name, 1);
        
        this->v_joint_values.resize(5);
        for(int i = 0; i < 5; ++i)
        {
            std::stringstream ss;
            ss << "arm_joint_" << (i+1);
            v_joint_values[i].joint_uri = ss.str();
            v_joint_values[i].unit = std::string("rad");
            v_joint_values[i].value = 0.0;
        }
        
        this->v_gripper_values.resize(2);
        v_gripper_values[0].joint_uri = "gripper_finger_joint_l";
        v_gripper_values[0].unit = std::string("m");
        v_gripper_values[0].value = 0.001;
        
        v_gripper_values[1].joint_uri = "gripper_finger_joint_r";
        v_gripper_values[1].unit = std::string("m");
        v_gripper_values[1].value = 0.001;
        
        this->gripper_positions[0] = 0.01;    
        this->gripper_positions[1] = 0.01;
        
}

Youbot::~Youbot()
{
    
}

void Youbot::PublishMessage(bool publish_gripper)
{
    brics_actuator::JointPositions msg;    
    for(int i = 0; i < 5; ++i)
    {
        v_joint_values[i].value = this->joint_positions[i];
    }    
    msg.positions = v_joint_values;
    this->arm_publisher.publish(msg);    
    
    // Publicar las posiciones del gripper resulta en una
    // bajada importante en el rendimiento del robot.
    // Se recomienda no pubicar las posiciones del gripper
    // al mismo ritmo que las del brazo
    if(publish_gripper)
    {
        brics_actuator::JointPositions gripper_msg;
        v_gripper_values[0].value = this->gripper_positions[0];
        v_gripper_values[1].value = this->gripper_positions[1];    
        gripper_msg.positions = v_gripper_values;
        
        this->gripper_publisher.publish(gripper_msg);         
    }
}
