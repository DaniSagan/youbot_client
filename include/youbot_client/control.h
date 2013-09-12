#ifndef CONTROL_H
#define CONTROL_H

#include <SFML/Graphics.hpp>
#include <list>
#include <string>

namespace dfv
{

class Control
{
public:
    Control();
    ~Control();
    
    void SetSize(const sf::Vector2i& size);
    void SetPosition(const sf::Vector2i& position);
    virtual void Draw(sf::RenderWindow& window) const;
    virtual void HandleEvent(std::list<std::string>& responses, const sf::Event& event);
    
protected:
    sf::Vector2i size;
    sf::Vector2i position;
    
    bool Contains(const sf::Vector2i& point);
    
private:

};

}

#endif
