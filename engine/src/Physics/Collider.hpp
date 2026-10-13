#pragma once

#include "Physics/PhysicsTypes.hpp"
#include "Utils/MathHelper.hpp"

#include <vector>
#include <variant>

struct CircleCollider {
    float radius = 0.5f;
};

// currently convex polygon
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
    ColliderData data = CircleCollider{};

    ColliderType Type() const {
        return std::holds_alternative<CircleCollider>(data)
            ? ColliderType::Circle
            : ColliderType::Polygon;
    }
};
