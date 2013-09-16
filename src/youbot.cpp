#include <youbot_client/youbot.h>

const float Youbot::joint_min_pos[] = {0.0100692f, 0.0100692f, -5.02655f, 0.0221239f, 0.110619f};
const float Youbot::joint_max_pos[] = {5.84014f, 2.61799f, -0.015708f, 3.4292f, 5.64159f};
const float Youbot::joint_ini_pos[] = {2.959675f, 1.144533f, -2.57124f, 1.811086f, 2.91237f};
const dfv::Vector3 Youbot::r[] = {dfv::Vector3(-0.034f, 0.f, 0.075f), 
                                  dfv::Vector3(0.f, 0.f, 0.155f),
                                  dfv::Vector3(0.f, 0.f, 0.135f),
                                  dfv::Vector3(0.f, 0.f, 0.113f),
                                  dfv::Vector3(0.f, 0.f, 0.105f)};

Youbot::Youbot(ros::NodeHandle& node_handle_, 
               std::string arm_topic_name_, 
               std::string gripper_topic_name):
    node_handle(node_handle_),
    gripper_state(closed)
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

dfv::Quaternion Youbot::GetJointLocalQuaternion(int index) const
{
    if (index == 0)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::k,
            this->joint_positions[0] - this->joint_ini_pos[0]);
    }
    else if (index == 1)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            this->joint_positions[1] - this->joint_ini_pos[1]);
    }
    else if (index == 2)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            this->joint_positions[2] - this->joint_ini_pos[2]);
    }
    else if (index == 3)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            this->joint_positions[3] - this->joint_ini_pos[3]);
    }
    else if (index == 4)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::k,
            this->joint_positions[4] - this->joint_ini_pos[4]);
    }
    else
    {
        return dfv::Quaternion(0, 0, 0, 0);
    }
}

dfv::Quaternion Youbot::GetJointLocalQuatFromAngle(int index, float angle) const
{
    if (index == 0)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::k,
            angle - this->joint_ini_pos[0]);
    }
    else if (index == 1)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            angle - this->joint_ini_pos[1]);
    }
    else if (index == 2)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            angle - this->joint_ini_pos[2]);
    }
    else if (index == 3)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::j,
            angle - this->joint_ini_pos[3]);
    }
    else if (index == 4)
    {
        return dfv::Quaternion::GetRotationQuaternion(-dfv::Vector3::k,
            angle - this->joint_ini_pos[4]);
    }
    else
    {
        return dfv::Quaternion(0, 0, 0, 0);
    }
}

dfv::Vector3 Youbot::GetJointPosition(int index) const
{
    if (index == 0)
    {
        return dfv::Vector3(0, 0, 0);
    }
    else
    {
        dfv::Vector3 rn = this->r[index - 1];
        for (int i = index - 1; i >= 0; i--)
        {
            rn.Rotate(this->GetJointLocalQuaternion(i));
        }        
        return rn + this->GetJointPosition(index - 1);
    }
}

void Youbot::ResetArmPosition()
{
    for (unsigned int i = 0; i < 5; i++)
    {
        this->joint_positions[i] = Youbot::joint_ini_pos[i];
    }
    this->PublishMessage();
}

dfv::Vector3 Youbot::GetJointPosFromAngles(unsigned int index, const std::vector<float>& joint_angles) const
{
    if (joint_angles.size() == 5)
    {
        if (index == 0)
        {
            return dfv::Vector3(0, 0, 0);
        }
        else
        {
            dfv::Vector3 rn = this->r[index - 1];
            for (int i = index - 1; i >= 0; i--)
            {
                rn.Rotate(this->GetJointLocalQuatFromAngle(i, joint_angles[i]));
            }        
            return rn + this->GetJointPosFromAngles(index - 1, joint_angles);
        }
    }
    else
    {
        return dfv::Vector3(0, 0, 0);
    }
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

void Youbot::OpenGripper()
{
    if (this->gripper_state == closed)
    {
        brics_actuator::JointPositions gripper_msg;
        v_gripper_values[0].value = 0.01f;
        v_gripper_values[1].value = 0.01f;    
        gripper_msg.positions = v_gripper_values;
        
        this->gripper_publisher.publish(gripper_msg); 
        this->gripper_state = open;
    }
}

void Youbot::CloseGripper()
{
    if (this->gripper_state == open)
    {
        brics_actuator::JointPositions gripper_msg;
        v_gripper_values[0].value = 0.001f;
        v_gripper_values[1].value = 0.001f;    
        gripper_msg.positions = v_gripper_values;
        
        this->gripper_publisher.publish(gripper_msg);
        this->gripper_state = closed;
    }
}

std::vector<float> Youbot::FindAnglesForPos(dfv::Vector3& target_pos)
{
    // cylindrical coordinates
    float theta_0 = atan2(target_pos.y, target_pos.x);
    float r = sqrt(target_pos.x*target_pos.x + target_pos.y*target_pos.y) + Youbot::r[0].x;
    float h = target_pos.z - Youbot::r[0].z;
    
    float l = sqrt(r*r + h*h);
    float x = (l*l + Youbot::r[2].z*Youbot::r[2].z - Youbot::r[1].z*Youbot::r[1].z) / (2*l);
    float hh = sqrt(Youbot::r[2].z*Youbot::r[2].z - x*x);
    float a0 = asin(hh / Youbot::r[1].z);
    float a1 = asin(hh / Youbot::r[2].z);
    float g0 = atan2(h, r);
    float g1 = atan2(r, h);
    float theta_1 = 3.1415926535f/2.f - a0 - g0;
    float theta_2 = a1 + g1 - theta_1;
    
    std::vector<float> res(3);
    res[0] = theta_0;
    res[1] = theta_1;
    res[2] = theta_2;
    
    return res; 
}



