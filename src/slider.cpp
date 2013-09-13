#include "youbot_client/slider.h"

namespace dfv
{

Slider::Slider() :
    Control(),
    min_value(0.f),
    max_value(1.f),
    slider_position(0.f),
    state(normal)
{
    this->SetSize(sf::Vector2i(100, 25));
}

Slider::~Slider()
{
    
}

void Slider::Draw(sf::RenderWindow& window) const
{
    sf::Shape sh = sf::Shape::Rectangle(this->position.x, 
                                        this->position.y, 
                                        this->position.x + this->size.x, 
                                        this->position.y + this->size.y,
                                        sf::Color(220, 220, 220), 
                                        1.f, sf::Color(0, 0, 0));
    window.Draw(sh);
    
    float slider_pos_x = (float)this->position.x + 
                         (float)this->size.x * this->slider_position;
                         
    sh = sf::Shape::Line(slider_pos_x, 
                         this->position.y, 
                         slider_pos_x, 
                         this->position.y + this->size.y,
                         3.f,
                         sf::Color(255, 0, 0));
                         
    window.Draw(sh);
    
}

void Slider::HandleEvent(std::list<std::string>& responses, 
                         const sf::Event& event)
{
    if (event.Type == sf::Event::MouseButtonPressed)
    {
        sf::Vector2i mouse_pos(event.MouseButton.X, event.MouseButton.Y);
        if (event.MouseButton.Button == sf::Mouse::Left)
        {
            if (this->Contains(mouse_pos))
            {
                 this->state = clicked;
                 this->SetSliderPosition(mouse_pos);
            }
        }
    }
    else if (event.Type == sf::Event::MouseButtonReleased)
    {
        sf::Vector2i mouse_pos(event.MouseButton.X, event.MouseButton.Y);
        if (event.MouseButton.Button == sf::Mouse::Left)
        {
            if (this->Contains(mouse_pos)) this->state = over;
            else this->state = normal;
        }   
    }
    else if (event.Type == sf::Event::MouseMoved)
    {
        sf::Vector2i mouse_pos(event.MouseMove.X, event.MouseMove.Y);
        if (this->state == normal || this->state == over)
        {
            if (this->Contains(mouse_pos)) this->state = over;
            else this->state = normal;
        }
        else if (this->state == clicked)
        {
            this->SetSliderPosition(mouse_pos);
        }
    }
    else if (event.Type == sf::Event::MouseWheelMoved)
    {
        if (this->state == over)
        {
             this->slider_position += 0.001 * (float)event.MouseWheel.Delta;
             if (this->slider_position > 1.f) this->slider_position = 1.f;
             if (this->slider_position < 0.f) this->slider_position = 0.f;
        }
    }
}

void Slider::SetMinValue(float value)
{
    this->min_value = value;
}

void Slider::SetMaxValue(float value)
{
    this->max_value = value;
}

float Slider::GetCurrentValue() const
{
    return this->min_value + 
           (this->max_value - this->min_value) * this->slider_position;
}

void Slider::SetCurrentValue(float value)
{
    float calc_slider_pos = (value - this->min_value) / 
                            (this->max_value - this->min_value);
    this->slider_position = calc_slider_pos > 1.f ? 1.f :
                            calc_slider_pos < 0.f ? 0.f :
                            calc_slider_pos;
}

void Slider::SetSliderPosition(sf::Vector2i mouse_pos)
{
    //if (mouse_pos.x >= this->position.x && 
    //    mouse.pos.x <= this->position.x + this->size.x)
    if (this->Contains(mouse_pos))
    {
        this->slider_position = (float)(mouse_pos.x - this->position.x) / 
                                (float)(this->size.x);
    }
}

}
