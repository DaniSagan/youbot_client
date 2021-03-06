#ifndef APP_H
#define APP_H

#include <ros/ros.h>
#include <SFML/Graphics.hpp>
#include <youbot_client/control.h>
#include <youbot_client/text.h>
#include <youbot_client/button.h>
#include <youbot_client/slider.h>
#include <youbot_client/youbot.h>
#include <std_msgs/Float32.h>

class App
{
public:
    App(ros::NodeHandle& node_handle_);
    ~App();
    
    void Initialize();
    void Run();
    void HandleInput();
    void Update();
    void Draw();
    
    bool LoadAssets(std::string res_path);
    bool AddControls();
    
    enum State
    {
        slider_control,
        sensor_control   
    }; 
    
    void SubscribeToTopics();
    
protected:
private:
    ros::NodeHandle& node_handle;    
    sf::RenderWindow window;
    dfv::Youbot youbot;
    State state;
    
    sf::Font font_big;
    sf::Font font_med;
    sf::Font font_small;
    
    // Velocity cmd subscribers
    ros::Subscriber linear_vel_subs;
    ros::Subscriber angular_vel_subs;
    
    // subscriber callbacks
    void LinearVelCallback(const std_msgs::Float32::ConstPtr& msg);
    void AngularVelCallback(const std_msgs::Float32::ConstPtr& msg);
    
    // Window Components
    std::vector<dfv::Slider> sliders;    
    std::vector<dfv::Text> text_joint_pos;
    std::vector<dfv::Text> text_joint_pos_2;
    
    dfv::Button button_open;
    dfv::Button button_close;
    dfv::Button button_reset;
    dfv::Button button_deactivate;
    dfv::Button button_activate;
    
    dfv::Button button_e1;
    dfv::Button button_e2;
    dfv::Button button_e3;
    dfv::Button button_e4;
    dfv::Button button_e5;
    
    int topic_wd;
    
};

#endif
