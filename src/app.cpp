#include "youbot_client/app.h"

App::App(ros::NodeHandle& node_handle_):
    node_handle(node_handle_),
    youbot(node_handle_),
    state(App::sensor_control),
    topic_wd(0)
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
    this->SubscribeToTopics();
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
        
        // Key events
        if (event.Type == sf::Event::KeyPressed)
        {
            // open gripper
            if (event.Key.Code == sf::Key::Num1)
            {
                youbot.OpenGripper();
            }
            
            // close gripper
            if (event.Key.Code == sf::Key::Num2)
            {
                youbot.CloseGripper();
            }
            
            // disable sliders
            if (event.Key.Code == sf::Key::Num3)
            {
                for (unsigned int i = 0; i < this->sliders.size(); i++)
                {
                    this->sliders[i].Disable();
                }
                this->state = App::sensor_control;
            }
            
            // enable sliders
            if (event.Key.Code == sf::Key::Num4)
            {
                for (unsigned int i = 0; i < this->sliders.size(); i++)
                {
                    this->sliders[i].Enable();
                }
                this->state = App::slider_control;
            }
            
            /*
            // turn left
            if (event.Key.Code == sf::Key::Q)
            {
                youbot.angular_vel = dfv::Vector3(0.f, 0.f, 0.8f);
            }
            
            // turn right
            if (event.Key.Code == sf::Key::E)
            {
                youbot.angular_vel = dfv::Vector3(0.f, 0.f, -0.8f);
            }
            
            // move forward
            if (event.Key.Code == sf::Key::W)
            {
                youbot.linear_vel = dfv::Vector3(0.2f, 0.f, 0.f);
            }
            
            // move backwards
            if (event.Key.Code == sf::Key::S)
            {
                youbot.linear_vel = dfv::Vector3(-0.2f, 0.f, 0.f);
            }
            
            // move to the left
            if (event.Key.Code == sf::Key::A)
            {
                youbot.linear_vel = dfv::Vector3(0.f, 0.2f, 0.f);
            }
            
            // move to the right
            if (event.Key.Code == sf::Key::D)
            {
                youbot.linear_vel = dfv::Vector3(0.0f, -0.2f, 0.f);
            }*/
        }
        
        /*if (event.Type == sf::Event::KeyReleased)
        {
            if (event.Key.Code == sf::Key::Q)
            {
                youbot.angular_vel = dfv::Vector3(0.f, 0.f, 0.f);
            }
            if (event.Key.Code == sf::Key::E)
            {
                youbot.angular_vel = dfv::Vector3(0.f, 0.f, 0.f);
            }
            if (event.Key.Code == sf::Key::W)
            {
                youbot.linear_vel = dfv::Vector3(0.f, 0.f, 0.f);
            }
            if (event.Key.Code == sf::Key::S)
            {
                youbot.linear_vel = dfv::Vector3(0.f, 0.f, 0.f);
            }
            if (event.Key.Code == sf::Key::A)
            {
                youbot.linear_vel = dfv::Vector3(0.f, 0.f, 0.f);
            }
            if (event.Key.Code == sf::Key::D)
            {
                youbot.linear_vel = dfv::Vector3(0.0f, 0.f, 0.f);
            }
        }*/
        
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
        
        this->button_e1.HandleEvent(responses, event);
        this->button_e2.HandleEvent(responses, event);
        this->button_e3.HandleEvent(responses, event);
        this->button_e4.HandleEvent(responses, event);
        this->button_e5.HandleEvent(responses, event);
    }
}

void App::Update()
{   
    ros::spinOnce();
    
    if (this->button_open.GetState() == dfv::Button::clicked) youbot.OpenGripper();
    if (this->button_close.GetState() == dfv::Button::clicked) youbot.CloseGripper();
    if (this->button_reset.GetState() == dfv::Button::clicked)
    {
        for (unsigned int i = 0; i < this->sliders.size(); i++)
        {
            this->sliders[i].SetCurrentValue(dfv::Youbot::joint_ini_pos[i]);
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
    if (this->state == App::sensor_control)
    {
        for (unsigned int i = 0; i < this->youbot.joint_states.size(); i++)
        {
            this->sliders[i].SetCurrentValue(this->youbot.joint_states[i]);
        }
    }
    
    for (unsigned int i = 0; i < this->text_joint_pos.size(); i++)
    {
        std::stringstream ss;
        ss.precision(4);
        ss << std::fixed << sliders[i].GetCurrentValue() << " [rad]";
        this->text_joint_pos[i].SetText(ss.str());
        
        ss.str(std::string(""));
        ss << std::fixed << this->sliders[i].GetCurrentValue() - dfv::Youbot::joint_ini_pos[i] << " [rad]";
        this->text_joint_pos_2[i].SetText(ss.str());
    }
    
    
    this->topic_wd++;
    if (this->topic_wd > 30)
    {
        this->youbot.linear_vel = dfv::Vector3(0.f, 0.f, 0.f);
        this->youbot.angular_vel = dfv::Vector3(0.f, 0.f, 0.f);
        ROS_WARN("Topic watchdog has actuated");
    }
    youbot.PublishPlatformVel();
}

void App::Draw()
{
    //this->window.Clear(sf::Color(250, 240, 255));
    this->window.Clear(sf::Color(255, 255, 255));
    
    // draw margins
    sf::Shape margin = sf::Shape::Rectangle(0, 0, this->window.GetWidth(), 20, sf::Color(255, 128, 0));
    this->window.Draw(margin);
    margin = sf::Shape::Rectangle(0, 0, 20, this->window.GetHeight(), sf::Color(255, 128, 0));
    this->window.Draw(margin);
    margin = sf::Shape::Rectangle(0, this->window.GetHeight() - 20, 
                                  this->window.GetWidth(), this->window.GetHeight(), 
                                  sf::Color(255, 128, 0));
    this->window.Draw(margin);
    margin = sf::Shape::Rectangle(this->window.GetWidth() - 20, 0, 
                                  this->window.GetWidth(), this->window.GetHeight(), 
                                  sf::Color(255, 128, 0));
    this->window.Draw(margin);
    
    // draw controls
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

    this->button_e1.Draw(this->window);
    this->button_e2.Draw(this->window);
    this->button_e3.Draw(this->window);
    this->button_e4.Draw(this->window);
    this->button_e5.Draw(this->window);
    
    this->youbot.Draw(this->window);
    this->window.Display();
}

bool App::LoadAssets(std::string res_path)
{
    std::string font_path = res_path + "res/font/Ubuntu-L.ttf";
    if (!this->font_big.LoadFromFile(font_path, 20)) return false;
    if (!this->font_med.LoadFromFile(font_path, 16)) return false;
    if (!this->font_small.LoadFromFile(font_path, 12)) return false;
    return true;
}

bool App::AddControls()
{
    // sliders
    this->sliders.resize(5);
    for (unsigned int i = 0; i < this->sliders.size(); i++)
    {
        this->sliders[i].SetSize(sf::Vector2i(200, 25));
        this->sliders[i].SetPosition(sf::Vector2i(25, 25 + 30 * i));
        this->sliders[i].SetMinValue(dfv::Youbot::joint_min_pos[i]);
        this->sliders[i].SetMaxValue(dfv::Youbot::joint_max_pos[i]);
        this->sliders[i].SetCurrentValue(dfv::Youbot::joint_ini_pos[i]);
        this->sliders[i].Disable();
    }
    
    // text displays
    this->text_joint_pos.resize(5);
    for (unsigned int i = 0; i < this->sliders.size(); i++)
    {
        this->text_joint_pos[i].SetPosition(sf::Vector2i(235, 25 + 30 * i));
        this->text_joint_pos[i].SetFont(this->font_med);
        this->text_joint_pos[i].SetSize(16.f);
        this->text_joint_pos[i].SetColor(sf::Color::Black);
        std::stringstream ss;
        ss.precision(4);
        ss << std::fixed << this->sliders[i].GetCurrentValue() << " [rad]";
        this->text_joint_pos[i].SetText(ss.str());
    }
    
    this->text_joint_pos_2.resize(5);
    for (unsigned int i = 0; i < this->sliders.size(); i++)
    {
        this->text_joint_pos_2[i].SetPosition(sf::Vector2i(340, 25 + 30 * i));
        this->text_joint_pos_2[i].SetFont(this->font_med);
        this->text_joint_pos_2[i].SetSize(16.f);
        this->text_joint_pos_2[i].SetColor(sf::Color::Black);
        std::stringstream ss;
        ss.precision(4);
        ss << std::fixed << sliders[i].GetCurrentValue() - dfv::Youbot::joint_ini_pos[i] << " [rad]";
        this->text_joint_pos_2[i].SetText(ss.str());
    }
    
    // buttons
    this->button_open.SetFont(&this->font_med);
    this->button_open.SetPosition(sf::Vector2i(523, 25));
    this->button_open.SetSize(sf::Vector2i(150, 25));
    this->button_open.SetText("[Q] Open Gripper");
    
    this->button_close.SetFont(&this->font_med);
    this->button_close.SetPosition(sf::Vector2i(523, 25 + 30));
    this->button_close.SetSize(sf::Vector2i(150, 25));
    this->button_close.SetText("[A] Close Gripper");
    
    this->button_reset.SetFont(&this->font_med);
    this->button_reset.SetPosition(sf::Vector2i(523, 25 + 60));
    this->button_reset.SetSize(sf::Vector2i(150, 25));
    this->button_reset.SetText("Reset Arm");
    
    this->button_deactivate.SetFont(&this->font_med);
    this->button_deactivate.SetPosition(sf::Vector2i(523, 25 + 90));
    this->button_deactivate.SetSize(sf::Vector2i(150, 25));
    this->button_deactivate.SetText("[W] Sensor control");
    
    this->button_activate.SetFont(&this->font_med);
    this->button_activate.SetPosition(sf::Vector2i(523, 25 + 120));
    this->button_activate.SetSize(sf::Vector2i(150, 25));
    this->button_activate.SetText("[S] Slider control");
    
    this->button_e1.SetFont(&this->font_med);
    this->button_e1.SetPosition(sf::Vector2i(678, 25));
    this->button_e1.SetSize(sf::Vector2i(150, 25));
    this->button_e1.SetText("***");
    
    this->button_e2.SetFont(&this->font_med);
    this->button_e2.SetPosition(sf::Vector2i(678, 25 + 30));
    this->button_e2.SetSize(sf::Vector2i(150, 25));
    this->button_e2.SetText("***");
    
    this->button_e3.SetFont(&this->font_med);
    this->button_e3.SetPosition(sf::Vector2i(678, 25 + 60));
    this->button_e3.SetSize(sf::Vector2i(150, 25));
    this->button_e3.SetText("***");
    
    this->button_e4.SetFont(&this->font_med);
    this->button_e4.SetPosition(sf::Vector2i(678, 25 + 90));
    this->button_e4.SetSize(sf::Vector2i(150, 25));
    this->button_e4.SetText("***");
    
    this->button_e5.SetFont(&this->font_med);
    this->button_e5.SetPosition(sf::Vector2i(678, 25 + 120));
    this->button_e5.SetSize(sf::Vector2i(150, 25));
    this->button_e5.SetText("***");
    
    // youbot
    this->youbot.SetPosition(sf::Vector2i(25, 175));
    this->youbot.SetSize(sf::Vector2i(803, 280));
    this->youbot.SetFont(this->font_small);
    
    return true;
}

void App::SubscribeToTopics()
{
    this->linear_vel_subs = 
        this->node_handle.subscribe("youbot_client/platform_vel_cmd/linear", 
                                    1,
                                    &App::LinearVelCallback,
                                    this);
    this->angular_vel_subs = 
        this->node_handle.subscribe("youbot_client/platform_vel_cmd/angular", 
                                    1,
                                    &App::AngularVelCallback,
                                    this);
}

void App::LinearVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->youbot.linear_vel = dfv::Vector3(msg->data, 0.f, 0.f);
    this->topic_wd = 0;
}

void App::AngularVelCallback(const std_msgs::Float32::ConstPtr& msg)
{
    this->youbot.angular_vel = dfv::Vector3(0.f, 0.f, -msg->data);
    this->topic_wd = 0;
}
    
    
    
