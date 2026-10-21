#pragma once

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>

struct AABB2
{
    glm::vec2 min{0.0f};
    glm::vec2 max{0.0f};

    glm::vec2 Center() const {
        return (min + max) / 2.0f;
    }

    float Area() const {
        if (!IsValid()) return 0.0f;
        return (max.x - min.x) * (max.y - min.y);
    }

    bool IsValid() const {
        return std::isfinite(min.x) && std::isfinite(min.y)
            && std::isfinite(max.x) && std::isfinite(max.y)
            && min.x <= max.x && min.y <= max.y;
    }

    bool Overlaps(const AABB2& other) const
    {
        if (!IsValid() || !other.IsValid()) return false;
        return min.x <= other.max.x && max.x >= other.min.x &&
               min.y <= other.max.y && max.y >= other.min.y;
    }

    bool Contains(const AABB2& other) const
    {
        if (!IsValid() || !other.IsValid()) return false;
        return min.x <= other.min.x && max.x >= other.max.x &&
               min.y <= other.min.y && max.y >= other.max.y;
    }

    AABB2 Merge(const AABB2& other) const {
        return Merge(*this, other);
    }

    static AABB2 Merge(const AABB2& a, const AABB2& b) {
        if (!a.IsValid()) return b;
        if (!b.IsValid()) return a;
        return AABB2{
            {std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y)},
            {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y)}
        };
    }
};
