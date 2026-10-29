#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <limits>
#include <cmath>

inline constexpr uint32_t InvalidIndex =
    std::numeric_limits<uint32_t>::max();

using BodyId = uint32_t;

enum class MotionType {
    Static, Kinematic, Dynamic
};

struct Transform2D {
    glm::vec2 position{0.0f};
    // counterclockwise, expressed in radians
    float rotation = 0.0f;
};

inline glm::vec2 WorldPoint(const Transform2D& transform,
    const glm::vec2& localPoint) {
    const float angle = transform.rotation;
    float c = std::cos(angle);
    float s = std::sin(angle);

    glm::vec2 rotated{
        c * localPoint.x - s * localPoint.y,
        s * localPoint.x + c * localPoint.y
    };

    return transform.position + rotated;
}

inline glm::vec2 LocalPoint(const Transform2D& transform,
    const glm::vec2& worldPoint) {
    const float angle = transform.rotation;
    float c = std::cos(angle);
    float s = std::sin(angle);

    glm::vec2 local = worldPoint - transform.position;
    local = {
        local.x * c + local.y * s,
        -local.x * s + local.y * c
    };
    return local;
}
