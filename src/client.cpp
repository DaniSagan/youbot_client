#include <iostream>
#include <string>
#include <sstream>
#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include "youbot_client/slider.h"
#include "youbot_client/button.h"
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
    
    // fonts
    std::string font_path = argv[0];
    font_path.erase(font_path.end() - 17, font_path.end());
    font_path += "res/font/Ubuntu-L.ttf";
    std::cout << font_path << std::endl;
    
    std::cout << argv[0] << std::endl;
    sf::Font font_big;
    font_big.LoadFromFile(font_path, 20);
    sf::Font font_med;
    font_med.LoadFromFile(font_path, 16);
    
    // buttons
    dfv::Button button_open;
    button_open.SetFont(&font_med);
    button_open.SetPosition(sf::Vector2i(500, 50));
    button_open.SetSize(sf::Vector2i(300, 50));
    button_open.SetText("Open Gripper");
    
    dfv::Button button_close;
    button_close.SetFont(&font_med);
    button_close.SetPosition(sf::Vector2i(500, 110));
    button_close.SetSize(sf::Vector2i(300, 50));
    button_close.SetText("Close Gripper");
    
    dfv::Button button_reset;
    button_reset.SetFont(&font_med);
    button_reset.SetPosition(sf::Vector2i(500, 170));
    button_reset.SetSize(sf::Vector2i(300, 50));
    button_reset.SetText("Reset Arm");
    
    // text displays    
    std::vector<sf::String> str_pos(5);
    std::vector<sf::String> str_joint_pos(5);
    
    for (unsigned int i = 0; i < 5; i++)
    {
        str_pos[i].SetPosition(sf::Vector2f(270, 50 + 50 * i));
        str_pos[i].SetSize(20.f);
        str_pos[i].SetColor(sf::Color::Black);
        str_pos[i].SetFont(font_big);
        
        str_joint_pos[i].SetPosition(sf::Vector2f(50, 310 + 25 * i));
        str_joint_pos[i].SetSize(16.f);
        str_joint_pos[i].SetColor(sf::Color::Black);
        str_joint_pos[i].SetFont(font_med);
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
            button_open.HandleEvent(responses, event);
            button_close.HandleEvent(responses, event);
            button_reset.HandleEvent(responses, event);
            
            if (button_reset.GetState() == dfv::Button::clicked)
            {
                for (unsigned int i = 0; i < 5; i++)
                {
                    sliders[i].SetCurrentValue(Youbot::joint_ini_pos[i]);
                }   
            }
            if (button_open.GetState() == dfv::Button::clicked)
            {
                youbot.OpenGripper();
            }
            if (button_close.GetState() == dfv::Button::clicked)
            {
                youbot.CloseGripper();
            }
        }
        
        // Window display
        std::stringstream ss;
        for (unsigned int i = 0; i < 5; i++)
        {
            ss.str(std::string(""));
            ss << sliders[i].GetCurrentValue() << " [rad]";
            str_pos[i].SetText(ss.str());
            
            youbot.joint_positions[i] = sliders[i].GetCurrentValue();
            
            ss.str(std::string(""));
            ss << "A" << i + 1 << " POS: " << youbot.GetJointPosition(i + 1);
            str_joint_pos[i].SetText(ss.str());
        }
        
        window.Clear(sf::Color(255, 255, 255));
        for (unsigned int i = 0; i < 5; i++)
        {
            sliders[i].Draw(window);
            window.Draw(str_pos[i]);
            window.Draw(str_joint_pos[i]);
        }
        
        button_open.Draw(window);
        button_close.Draw(window);
        button_reset.Draw(window);
        
        window.Display();
        
        youbot.PublishMessage();
    }
    
    return 0;
}
