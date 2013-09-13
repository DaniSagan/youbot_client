#include <iostream>
#include <string>
#include <sstream>
#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include "youbot_client/slider.h"
#include "youbot_client/youbot.h"

int main(int argc, char** argv)
{
    const std::string NAME = "YouBot_Client";
    
    // ROS initialization
    ros::init(argc, argv, NAME);
    ros::NodeHandle node_handle;
    
    // YouBot object
    Youbot youbot(node_handle);
    
    // Window
    const unsigned int HEIGHT = 480;
    const unsigned int WIDTH = HEIGHT * 16 / 9;
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), NAME);
    window.SetFramerateLimit(30);
    
    std::vector<dfv::Slider> sliders(5);
    
    for (unsigned int i = 0; i < 5; i++)
    {
        sliders[i].SetSize(sf::Vector2i(200, 25));
        sliders[i].SetPosition(sf::Vector2i(50, 50 + 50 * i));
        sliders[i].SetMinValue(Youbot::joint_min_pos[i]);
        sliders[i].SetMaxValue(Youbot::joint_max_pos[i]);
        sliders[i].SetCurrentValue(Youbot::joint_ini_pos[i]);
    }
    
    std::vector<sf::String> str_pos(5);
    std::vector<sf::String> str_joint_pos(5);
    
    for (unsigned int i = 0; i < 5; i++)
    {
        str_pos[i].SetPosition(sf::Vector2f(270, 50 + 50 * i));
        str_pos[i].SetSize(20.f);
        str_pos[i].SetColor(sf::Color::Black);
        
        str_joint_pos[i].SetPosition(sf::Vector2f(50, 300 + 25 * i));
        str_joint_pos[i].SetSize(16.f);
        str_joint_pos[i].SetColor(sf::Color::Black);
    }
    
    while(!node_handle.ok());
    
    // Main loop    
    while (window.IsOpened() && node_handle.ok())
    {
        // Event handling
        sf::Event event;
        while (window.GetEvent(event))
        {
            if (event.Type == sf::Event::Closed)
            {
                window.Close();
            }
            
            std::list<std::string> responses;
            for (unsigned int i = 0; i < 5; i++)
            {
                sliders[i].HandleEvent(responses, event);
            }            
        }
        
        // Window display
        std::stringstream ss;
        for (unsigned int i = 0; i < 5; i++)
        {
            ss.str(std::string(""));
            ss << sliders[i].GetCurrentValue();
            str_pos[i].SetText(ss.str());
            
            youbot.joint_positions[i] = sliders[i].GetCurrentValue();
            
            ss.str(std::string(""));
            ss << "A" << i + 1 << " POS: " << youbot.GetJointPosition(i + 1);
            str_joint_pos[i].SetText(ss.str());
        }
        
        window.Clear(sf::Color(200, 200, 200));
        for (unsigned int i = 0; i < 5; i++)
        {
            sliders[i].Draw(window);
            window.Draw(str_pos[i]);
            window.Draw(str_joint_pos[i]);
        }
        
        window.Display();
        
        youbot.PublishMessage();
    }
    
    return 0;
}
