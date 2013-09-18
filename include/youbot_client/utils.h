#ifndef UTILS_H
#define UTILS_H

#include <dfv/dfv.h>
#include <SFML/Graphics.hpp>

inline sf::Vector2i SfVectorFloor(const sf::Vector2f& v)
{
    return sf::Vector2i(floor(v.x), floor(v.y));
}

inline sf::Vector3f DfvToSfVector(const dfv::Vector3& v)
{
    return sf::Vector3f(v.x, v.y, v.z);
}

#endif
