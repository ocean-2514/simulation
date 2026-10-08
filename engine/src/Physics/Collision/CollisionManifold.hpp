#pragma once

#include <glm/glm.hpp>

struct CollisionManifold
{
    bool colliding = false;
    glm::vec2 normal{0.0f};
    glm::vec2 contactPoint{0.0f};
    float penetration = 0.0f;
};
