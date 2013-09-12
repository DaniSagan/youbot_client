#include <youbot_client/control.h>

namespace dfv
{

Control::Control():
    size(sf::Vector2i(0, 0)),
    position(sf::Vector2i(0, 0))
{
}

Control::~Control()
{
}


void Control::SetSize(const sf::Vector2i& size)
{
    this->size = size;
}

void Control::SetPosition(const sf::Vector2i& position)
{
    this->position = position;
}

void Control::Draw(sf::RenderWindow& window) const
{
    
}

void Control::HandleEvent(std::list<std::string>& responses, const sf::Event& event)
{
    
}

bool Control::Contains(const sf::Vector2i& point)
{
    return point.x >= this->position.x && point.x <= this->position.x + this->size.x &&
           point.y >= this->position.y && point.y <= this->position.y + this->size.y;    
}

}
