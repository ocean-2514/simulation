#pragma once

#include "Physics/PhysicsTypes.hpp"
#include "Utils/MathHelper.hpp"

#include <vector>
#include <variant>

struct CircleCollider {
    float radius = 0.5f;
};

struct PolygonCollider {
    // vertices counterclockwise
    std::vector<glm::vec2> vertices;
    std::vector<glm::vec2> normals;

    void PopulateNormals() {
        normals = MathHelper::CalculateNormals(vertices);
    }
};

enum class ColliderType {
    Circle, Polygon
};

using ColliderData = std::variant<
    CircleCollider, PolygonCollider>;

struct Collider {
    ColliderType type = ColliderType::Circle;
    ColliderData data = CircleCollider{};
    // Transform2D localTransform{};
    float restitution = 0.2f;
    float friction = 0.5f;
};