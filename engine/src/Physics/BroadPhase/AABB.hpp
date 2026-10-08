#pragma once

#include <glm/glm.hpp>

struct AABB
{
    glm::vec2 min{0.0f};
    glm::vec2 max{0.0f};

    glm::vec2 Center() const {
        return (min + max) / 2.0f;
    }

    float Area() const {
        return (max.x - min.x) * (max.y - min.y);
    }

    bool Overlaps(const AABB& other) const
    {
        return min.x <= other.max.x && max.x >= other.min.x &&
               min.y <= other.max.y && max.y >= other.min.y;
    }
};