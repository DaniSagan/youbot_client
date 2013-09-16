#include "youbot_client/text.h"

namespace dfv
{

Text::Text():
    Control(),
    str(sf::String("Text", sf::Font::GetDefaultFont(), 20.f))
{
    //this->str.SetStyle(sf::String::Bold);
}

Text::~Text()
{
}

void Text::SetPosition(const sf::Vector2i& position)
{
    this->str.SetPosition(sf::Vector2f(position.x, position.y));
}

void Text::Draw(sf::RenderWindow& window) const
{
    window.Draw(this->str);
}

void Text::HandleEvent(std::list<std::string>& responses, const sf::Event& event)
{
}

void Text::SetText(const std::string& text)
{
    this->str.SetText(text);
}

void Text::SetFont(const sf::Font& font)
{
    this->str.SetFont(font);
}

void Text::SetSize(float size)
{
    this->str.SetSize(size);
}

void Text::SetColor(const sf::Color& color)
{
    this->str.SetColor(color);
}

}
