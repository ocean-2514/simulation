#pragma once

#include <glm/glm.hpp>
#include <vector>

struct Collider;
struct CircleCollider;
struct PolygonCollider;

class MathHelper {
public:
    static float CalculateTriangleArea(const glm::vec2& p1, 
        const glm::vec2& p2, const glm::vec2& p3);

    static float CalculatePolygonArea(const std::vector<glm::vec2>& points);

    static glm::vec2 CalculatePolygonCentroid(const std::vector<glm::vec2>& points);

    // vertices counterclockwise
    static std::vector<glm::vec2> CalculateNormals(
        const std::vector<glm::vec2>& vertices);

    // rotation expressed in radians
    static glm::vec2 Rotate(const glm::vec2& vec, float rotation);
    static float Cross(const glm::vec2& a, const glm::vec2& b);

    struct InertiaCalculator {
        float operator()(const CircleCollider& circle);
        float operator()(const PolygonCollider& polygon);
    };

    static float CalculateInertia(const Collider& collider, float mass);
};