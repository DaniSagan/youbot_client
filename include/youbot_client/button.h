#ifndef BUTTON_H
#define BUTTON_H

#include <SFML/Graphics.hpp>
#include "youbot_client/control.h"

namespace dfv
{

class Button : public Control
{
public:
    Button();
    ~Button();
    
    virtual void Draw(sf::RenderWindow& window) const;
    virtual void HandleEvent(std::list<std::string>& responses, const sf::Event& event);
    
    enum State
    {
        normal = 0,
        over,
        clicked
    };
    
    void SetFont(sf::Font* lp_font);
    void SetText(std::string text);
    State GetState() const;
    
protected:
    sf::Font* lp_font;
    std::string text;
    State state;
    sf::Color normal_color;
    sf::Color over_color;
    sf::Color clicked_color;

private:
};

}

#endif
