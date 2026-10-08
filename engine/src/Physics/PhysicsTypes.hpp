#pragma once

#include <glm/glm.hpp>

struct Transform2D {
    glm::vec2 position{0.0f};
    // counterclockwise, expressed in degrees
    float rotation = 0.0f;
};

inline glm::vec2 WorldPoint(const Transform2D& transform, 
    const glm::vec2& localPoint) {
    float angle = glm::radians(transform.rotation);
    float c = glm::cos(angle);
    float s = glm::sin(angle);

    glm::vec2 rotated{
        c * localPoint.x - s * localPoint.y,
        s * localPoint.x + c * localPoint.y
    };

    return transform.position + rotated;
}