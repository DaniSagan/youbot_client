#ifndef SLIDER_H
#define SLIDER_H

#include "youbot_client/control.h"

namespace dfv
{

class Slider : public Control
{
public:
    Slider();
    ~Slider();
    
    // override methods
    virtual void Draw(sf::RenderWindow& window) const;
    virtual void HandleEvent(std::list<std::string>& responses, const sf::Event& event);
    
    // own methods
    void SetMinValue(float value);
    void SetMaxValue(float value);
    
    enum State
    {
        normal = 0,
        over,
        clicked,
        disabled
    };
    
    float GetCurrentValue() const;
    void SetCurrentValue(float value);
    void Disable();
    void Enable();
    
protected:
    float min_value;
    float max_value;
    float slider_position; // value between 0 and 1
    State state;    
    
    void SetSliderPosition(sf::Vector2i mouse_pos);
    
private:
};

}

#endif
