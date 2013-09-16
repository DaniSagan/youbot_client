#include "youbot_client/app.h"

App::App(ros::NodeHandle& node_handle_):
    node_handle(node_handle_),
    youbot(node_handle_),
    state(App::sensor_control)
{
}

App::~App()
{
}

void App::Initialize()
{
    this->window.Create(sf::VideoMode(480 * 16 / 9, 480), "Youbot Client", sf::Style::Close);
    this->window.SetFramerateLimit(60);
    this->AddControls();
}

void App::Run()
{
    while (this->window.IsOpened())
    {        
        this->HandleInput();
        this->Update();
        this->Draw();
    }
    
}

void App::HandleInput()
{
    sf::Event event;
    while (this->window.GetEvent(event))
    {
        if (event.Type == sf::Event::Closed)
        {
            this->window.Close();
        }
        
        if (event.Type == sf::Event::KeyPressed)
        {
            if (event.Key.Code == sf::Key::O)
            {
                youbot.OpenGripper();
            }
            if (event.Key.Code == sf::Key::C)
            {
                youbot.CloseGripper();
            }
        }
        
        std::list<std::string> responses;
        for (unsigned int i = 0; i < this->sliders.size(); i++)
        {
            this->sliders[i].HandleEvent(responses, event);
        }
        this->button_open.HandleEvent(responses, event);
        this->button_close.HandleEvent(responses, event);
        this->button_reset.HandleEvent(responses, event);
        this->button_deactivate.HandleEvent(responses, event);
        this->button_activate.HandleEvent(responses, event);
    }
}

void App::Update()
{
    for (unsigned int i = 0; i < this->text_joint_pos.size(); i++)
    {
        std::stringstream ss;
        ss << sliders[i].GetCurrentValue() << " [rad]";
        this->text_joint_pos[i].SetText(ss.str());
        
        ss.str(std::string(""));
        ss << this->sliders[i].GetCurrentValue() - Youbot::joint_ini_pos[i] << " [rad]";
        this->text_joint_pos_2[i].SetText(ss.str());
    }
    
    if (this->button_open.GetState() == dfv::Button::clicked) youbot.OpenGripper();
    if (this->button_close.GetState() == dfv::Button::clicked) youbot.CloseGripper();
    if (this->button_reset.GetState() == dfv::Button::clicked)
    {
        for (unsigned int i = 0; i < this->sliders.size(); i++)
        {
            this->sliders[i].SetCurrentValue(Youbot::joint_ini_pos[i]);
        }
        //youbot.ResetArmPosition();
    }
    if (this->button_deactivate.GetState() == dfv::Button::clicked)
    {
        for (unsigned int i = 0; i < this->sliders.size(); i++)
        {
            this->sliders[i].Disable();
        }
        this->state = App::sensor_control;
    }
    if (this->button_activate.GetState() == dfv::Button::clicked)
    {
        for (unsigned int i = 0; i < this->sliders.size(); i++)
        {
            this->sliders[i].Enable();
        }
        this->state = App::slider_control;
    }
    
    for (unsigned int i = 0; i < this->sliders.size(); i++)
    {
        youbot.joint_positions[i] = sliders[i].GetCurrentValue();
    }
    
    if (this->state == App::slider_control) youbot.PublishMessage();
}

void App::Draw()
{
    this->window.Clear(sf::Color(250, 240, 255));
    for (unsigned int i = 0; i < this->sliders.size(); i++)
    {
        this->sliders[i].Draw(this->window);
        this->text_joint_pos[i].Draw(this->window);
        this->text_joint_pos_2[i].Draw(this->window);
    }
    this->button_open.Draw(this->window);
    this->button_close.Draw(this->window);
    this->button_reset.Draw(this->window);
    this->button_deactivate.Draw(this->window);
    this->button_activate.Draw(this->window);
    this->window.Display();
}

bool App::LoadAssets(std::string res_path)
{
    std::string font_path = res_path + "res/font/Ubuntu-L.ttf";
    if (!this->font_big.LoadFromFile(font_path, 20)) return false;
    if (!this->font_med.LoadFromFile(font_path, 16)) return false;
    return true;
}

bool App::AddControls()
{
    // sliders
    this->sliders.resize(5);
    for (unsigned int i = 0; i < this->sliders.size(); i++)
    {
        this->sliders[i].SetSize(sf::Vector2i(200, 25));
        this->sliders[i].SetPosition(sf::Vector2i(50, 50 + 30 * i));
        this->sliders[i].SetMinValue(Youbot::joint_min_pos[i]);
        this->sliders[i].SetMaxValue(Youbot::joint_max_pos[i]);
        this->sliders[i].SetCurrentValue(Youbot::joint_ini_pos[i]);
        this->sliders[i].Disable();
    }
    
    // text displays
    this->text_joint_pos.resize(5);
    for (unsigned int i = 0; i < this->sliders.size(); i++)
    {
        this->text_joint_pos[i].SetPosition(sf::Vector2i(270, 50 + 30 * i));
        this->text_joint_pos[i].SetFont(this->font_med);
        this->text_joint_pos[i].SetSize(16.f);
        this->text_joint_pos[i].SetColor(sf::Color::Black);
        std::stringstream ss;
        ss << this->sliders[i].GetCurrentValue() << " [rad]";
        this->text_joint_pos[i].SetText(ss.str());
    }
    
    this->text_joint_pos_2.resize(5);
    for (unsigned int i = 0; i < this->sliders.size(); i++)
    {
        this->text_joint_pos_2[i].SetPosition(sf::Vector2i(400, 50 + 30 * i));
        this->text_joint_pos_2[i].SetFont(this->font_med);
        this->text_joint_pos_2[i].SetSize(16.f);
        this->text_joint_pos_2[i].SetColor(sf::Color::Black);
        std::stringstream ss;
        ss << sliders[i].GetCurrentValue() - Youbot::joint_ini_pos[i] << " [rad]";
        this->text_joint_pos_2[i].SetText(ss.str());
    }
    
    // buttons
    this->button_open.SetFont(&this->font_med);
    this->button_open.SetPosition(sf::Vector2i(50, 250));
    this->button_open.SetSize(sf::Vector2i(200, 25));
    this->button_open.SetText("Open Gripper");
    
    this->button_close.SetFont(&this->font_med);
    this->button_close.SetPosition(sf::Vector2i(50, 280));
    this->button_close.SetSize(sf::Vector2i(200, 25));
    this->button_close.SetText("Close Gripper");
    
    this->button_reset.SetFont(&this->font_med);
    this->button_reset.SetPosition(sf::Vector2i(50, 310));
    this->button_reset.SetSize(sf::Vector2i(200, 25));
    this->button_reset.SetText("Reset Arm");
    
    this->button_deactivate.SetFont(&this->font_med);
    this->button_deactivate.SetPosition(sf::Vector2i(50, 340));
    this->button_deactivate.SetSize(sf::Vector2i(200, 25));
    this->button_deactivate.SetText("Deactivate sliders");
    
    this->button_activate.SetFont(&this->font_med);
    this->button_activate.SetPosition(sf::Vector2i(50, 370));
    this->button_activate.SetSize(sf::Vector2i(200, 25));
    this->button_activate.SetText("Activate sliders");
    
    return true;
}
    
    
    
