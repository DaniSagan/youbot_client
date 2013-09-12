#include <iostream>
#include <string>
#include <sstream>
#include <SFML/Graphics.hpp>
#include <ros/ros.h>
#include "youbot_client/slider.h"
#include "youbot_client/youbot.h"

int main(int argc, char** argv)
{
    const unsigned int HEIGHT = 480;
    const unsigned int WIDTH = HEIGHT * 16 / 9;
    const std::string NAME = "YouBot_Client";
    
    float joint_min_pos[] = {0.0100692, 0.0100692, -5.02655, 0.0221239, 0.110619};
    float joint_max_pos[] = {5.84014, 2.61799, -0.015708, 3.4292, 5.64159};
    
    ros::init(argc, argv, NAME);
    ros::NodeHandle node_handle;
    
    Youbot youbot(node_handle);
    
    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), NAME);
    window.SetFramerateLimit(30);
    
    std::vector<dfv::Slider> sliders(5);
    
    for (unsigned int i = 0; i < 5; i++)
    {
        sliders[i].SetSize(sf::Vector2i(200, 25));
        sliders[i].SetPosition(sf::Vector2i(100, 100 + 50 * i));
        sliders[i].SetMinValue(Youbot::joint_min_pos[i]);
        sliders[i].SetMaxValue(Youbot::joint_max_pos[i]);
        sliders[i].SetCurrentValue(Youbot::joint_ini_pos[i]);
    }
    
    std::vector<sf::String> str_pos(5);
    
    for (unsigned int i = 0; i < 5; i++)
    {
        str_pos[i].SetPosition(sf::Vector2f(320, 100 + 50 * i));
        str_pos[i].SetSize(20.f);
        str_pos[i].SetColor(sf::Color::Black);
    }
    
    while (window.IsOpened() && node_handle.ok())
    {
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
        
        std::stringstream ss;
        for (unsigned int i = 0; i < 5; i++)
        {
            ss.str(std::string(""));
            ss << sliders[i].GetCurrentValue();
            str_pos[i].SetText(ss.str());
            
            youbot.joint_positions[i] = sliders[i].GetCurrentValue();
        }
        
        window.Clear(sf::Color(200, 200, 200));
        for (unsigned int i = 0; i < 5; i++)
        {
            sliders[i].Draw(window);
            window.Draw(str_pos[i]);
        }
        
        window.Display();
        
        youbot.PublishMessage();
    }
    
    return 0;
}
