#include "Utils/MathHelper.hpp"
#include "Physics/Collider.hpp"

#include <algorithm>
#include <cmath>

float MathHelper::CalculateTriangleArea(const glm::vec2& p1, 
    const glm::vec2& p2, const glm::vec2& p3) {
    return 0.5f * std::abs((p2.x - p1.x) * (p3.y - p1.y) - 
        (p3.x - p1.x) * (p2.y - p1.y));
}

float MathHelper::CalculatePolygonArea(const std::vector<glm::vec2>& points) {
    float area = 0.0f;
    std::size_t n = points.size();
    if (n < 3) {
        return 0.0f; // Not a polygon
    }
    const glm::vec2& firstPoint = points[0];
    for (std::size_t i = 1; i < n - 1; ++i) {
        area += (points[i].x - firstPoint.x) * (points[i + 1].y - firstPoint.y) - 
                (points[i + 1].x - firstPoint.x) * (points[i].y - firstPoint.y);
    }
    return std::abs(area) * 0.5f;
}

glm::vec2 MathHelper::CalculatePolygonCentroid(const std::vector<glm::vec2>& points) {
    std::size_t n = points.size();
    if (n < 3) return glm::vec2(0.0f);

    const glm::vec2& ref = points[0];

    float area = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;

    for (std::size_t i = 0; i < n; ++i) {
        std::size_t j = (i + 1) % n;
        float x0 = points[i].x - ref.x;
        float y0 = points[i].y - ref.y;
        float x1 = points[j].x - ref.x;
        float y1 = points[j].y - ref.y;

        float cross = x0 * y1 - x1 * y0;
        area += cross;
        cx += (x0 + x1) * cross;
        cy += (y0 + y1) * cross;
    }

    area *= 0.5f;
    if (std::abs(area) < 1e-8f) return glm::vec2(0.0f);

    float inv = 1.0f / (6.0f * area);
    return glm::vec2(cx * inv + ref.x, cy * inv + ref.y);
}

std::vector<glm::vec2> MathHelper::CalculateNormals(
    const std::vector<glm::vec2>& vertices) {
    std::size_t n = vertices.size();
    if (n < 3) return {};

    std::vector<glm::vec2> normals;
    normals.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        glm::vec2 edge = vertices[(i + 1) % n] - vertices[i];
        glm::vec2 normal = {edge.y, -edge.x};
        normals.push_back(glm::normalize(normal));
    }

    return normals;
}

glm::vec2 MathHelper::Rotate(const glm::vec2& vec, float rotation) {
    const float angle = rotation;
    float c = std::cos(angle);
    float s = std::sin(angle);
    return {
        vec.x * c - vec.y * s,
        vec.x * s + vec.y * c
    };
}

float MathHelper::InertiaCalculator::operator()(
    const CircleCollider& circle) {
    if (circle.radius <= 0.0f || !std::isfinite(circle.radius)) return 0.0f;
    return 0.5f * circle.radius * circle.radius;
} 

float MathHelper::InertiaCalculator::operator()(
    const PolygonCollider& polygon) {
    std::size_t n = polygon.vertices.size();
    if (n < 3) {
        return 0.0f;
    }

    const auto& points = polygon.vertices;
    const glm::vec2& ref = points[0];

    float twiceArea = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    float inertia = 0.0f;

    for (std::size_t i = 0; i < n; ++i) {
        std::size_t j = (i + 1) % n;
        float x0 = points[i].x - ref.x;
        float y0 = points[i].y - ref.y;
        float x1 = points[j].x - ref.x;
        float y1 = points[j].y - ref.y;

        float cross = x0 * y1 - x1 * y0;
        twiceArea += cross;
        cx += (x0 + x1) * cross;
        cy += (y0 + y1) * cross;
        inertia += (x0 * x0 + y0 * y0 + x1 * x1 + y1 * y1   
            + x0 * x1 + y0 * y1) * cross;
    }

    if (std::abs(twiceArea) < 1e-6f) return 0.0f;

    const float centroidScale = 1.0f / (3.0f * twiceArea);
    const float centroidX = cx * centroidScale;
    const float centroidY = cy * centroidScale;
    const float inertiaPerMassAtReference = inertia / (6.0f * twiceArea);
    const float inertiaPerMassAtCentroid = inertiaPerMassAtReference
        - centroidX * centroidX - centroidY * centroidY;
    return std::max(0.0f, inertiaPerMassAtCentroid);
} 

float MathHelper::CalculateInertia(
    const Collider& collider, float mass) {
    if (mass <= 0.0f || !std::isfinite(mass)) return 0.0f;

    static InertiaCalculator calculator;
    return std::visit(calculator, collider.data) * mass;
}
