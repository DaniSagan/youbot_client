#include "youbot_client/button.h"

namespace dfv
{

Button::Button():
    Control(),
    lp_font(NULL),
    text(std::string("Button")),
    state(normal),
    normal_color(sf::Color(200, 200, 220)),
    over_color(sf::Color(160, 160, 200)),
    clicked_color(sf::Color(100, 100, 200))
{
    this->SetSize(sf::Vector2i(150, 40));
}

Button::~Button()
{
}

void Button::Draw(sf::RenderWindow& window) const
{
    sf::Color button_color;
    if (this->state == normal) button_color = this->normal_color;
    else if (this->state == over) button_color = this->over_color;
    else if (this->state == clicked) button_color = this->clicked_color;
    
    sf::Shape sh = sf::Shape::Rectangle(this->position.x, 
                                        this->position.y, 
                                        this->position.x + this->size.x, 
                                        this->position.y + this->size.y,
                                        button_color);//, 
                                        //1.f, sf::Color(0, 0, 0));
    window.Draw(sh);
    
    sf::String caption;
    if(this->lp_font != NULL)
    {
        caption.SetFont(*(this->lp_font));
    }
    caption.SetText(this->text);
    caption.SetColor(sf::Color::Black);
    //caption.SetPosition(sf::Vector2f(this->position.x, this->position.y));
    caption.SetSize(16.f);
    
    sf::FloatRect rect = caption.GetRect();
    caption.SetPosition(sf::Vector2f(floor(this->position.x + (this->size.x - rect.GetWidth()) / 2), 
                                     floor(this->position.y + (this->size.y - rect.GetHeight()) / 2)));
                             
    window.Draw(caption);
}

void Button::HandleEvent(std::list<std::string>& responses, const sf::Event& event)
{
    if (event.Type == sf::Event::MouseButtonPressed)
    {
        sf::Vector2i mouse_pos(event.MouseButton.X, event.MouseButton.Y);
        if (event.MouseButton.Button == sf::Mouse::Left)
        {
            if (this->Contains(mouse_pos))
            {
                 this->state = clicked;
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
            //this->SetSliderPosition(mouse_pos);
        }
    }
    else if (event.Type == sf::Event::MouseWheelMoved)
    {
        if (this->state == over)
        {
             //this->slider_position += 0.001 * (float)event.MouseWheel.Delta;
             //if (this->slider_position > 1.f) this->slider_position = 1.f;
             //if (this->slider_position < 0.f) this->slider_position = 0.f;
        }
    }
}

void Button::SetFont(sf::Font* lp_font)
{
    this->lp_font = lp_font;
}

void Button::SetText(std::string text)
{
    this->text = text;
}

Button::State Button::GetState() const
{
    return this->state;
}

}
