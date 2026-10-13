#pragma once

#include <glm/glm.hpp>
#include <cstdint>
#include <limits>

inline constexpr uint32_t InvalidIndex =
    std::numeric_limits<uint32_t>::max();

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
    float c = glm::cos(angle);
    float s = glm::sin(angle);

    glm::vec2 rotated{
        c * localPoint.x - s * localPoint.y,
        s * localPoint.x + c * localPoint.y
    };

    return transform.position + rotated;
}
