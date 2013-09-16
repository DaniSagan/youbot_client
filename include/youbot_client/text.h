#ifndef TEXT_H
#define TEXT_H

#include <SFML/Graphics.hpp>
#include "youbot_client/control.h"

namespace dfv
{

class Text : public Control
{
public:
    Text();
    ~Text();
    
    virtual void SetPosition(const sf::Vector2i& position);
    virtual void Draw(sf::RenderWindow& window) const;
    virtual void HandleEvent(std::list<std::string>& responses, const sf::Event& event);
    
    void SetText(const std::string& text);
    void SetFont(const sf::Font& font);
    void SetSize(float size);
    void SetColor(const sf::Color& color);
    
protected:

    sf::String str;

private:
    
};

}

#endif
