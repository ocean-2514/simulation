#include "Physics/Collision/CollisionDetector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace
{

// axis should be normalized
void ProjectPolygon(const std::vector<glm::vec2>& vertices, 
    const glm::vec2& axis, 
    float& outMin, float& outMax) {
    outMin = std::numeric_limits<float>::max();
    outMax = -std::numeric_limits<float>::max();

    for (const auto& vertex : vertices) {
        float proj = glm::dot(axis, vertex);
        outMin = std::min(outMin, proj);
        outMax = std::max(outMax, proj);
    }
}

glm::vec2 ComputeContactPoint(
    const std::vector<glm::vec2>& verticesA, 
    const std::vector<glm::vec2>& verticesB, 
    const glm::vec2& axis) {
    if (verticesA.empty() || verticesB.empty()) return glm::vec2{0.0f};

    const glm::vec2 normal = glm::normalize(axis);
    const glm::vec2 tangent{-normal.y, normal.x};

    // The manifold normal points from A to B. The contacting support of A is
    // therefore its maximum projection along the normal; B's is its minimum.
    float supportA = -std::numeric_limits<float>::max();
    for (const glm::vec2& vertex : verticesA) {
        supportA = std::max(supportA, glm::dot(vertex, normal));
    }

    float supportB = std::numeric_limits<float>::max();
    for (const glm::vec2& vertex : verticesB) {
        supportB = std::min(supportB, glm::dot(vertex, normal));
    }

    // Keep the whole support edge when several vertices share the extreme
    // projection. Picking an arbitrary endpoint creates an artificial torque
    // for face-to-face contacts such as a polygon resting on the ground.
    const float supportTolerance = 1e-5f
        * std::max({1.0f, std::abs(supportA), std::abs(supportB)});
    float tangentMinA = std::numeric_limits<float>::max();
    float tangentMaxA = -std::numeric_limits<float>::max();
    for (const glm::vec2& vertex : verticesA) {
        if (supportA - glm::dot(vertex, normal) <= supportTolerance) {
            const float projection = glm::dot(vertex, tangent);
            tangentMinA = std::min(tangentMinA, projection);
            tangentMaxA = std::max(tangentMaxA, projection);
        }
    }

    float tangentMinB = std::numeric_limits<float>::max();
    float tangentMaxB = -std::numeric_limits<float>::max();
    for (const glm::vec2& vertex : verticesB) {
        if (glm::dot(vertex, normal) - supportB <= supportTolerance) {
            const float projection = glm::dot(vertex, tangent);
            tangentMinB = std::min(tangentMinB, projection);
            tangentMaxB = std::max(tangentMaxB, projection);
        }
    }

    // Pick the middle of the overlapping support intervals. If the features
    // only touch at a corner, use the midpoint of their nearest endpoints.
    float tangentCoordinate;
    if (tangentMaxA < tangentMinB) {
        tangentCoordinate = 0.5f * (tangentMaxA + tangentMinB);
    } else if (tangentMaxB < tangentMinA) {
        tangentCoordinate = 0.5f * (tangentMaxB + tangentMinA);
    } else {
        const float overlapMin = std::max(tangentMinA, tangentMinB);
        const float overlapMax = std::min(tangentMaxA, tangentMaxB);
        tangentCoordinate = 0.5f * (overlapMin + overlapMax);
    }

    const float normalCoordinate = 0.5f * (supportA + supportB);
    return normal * normalCoordinate + tangent * tangentCoordinate;
}

// <point, dist>
std::pair<glm::vec2, float> ClosestPointOnSegment(
    const glm::vec2& center, const glm::vec2& vertexA,
    const glm::vec2& vertexB) {
    const glm::vec2 ab = vertexB - vertexA;
    const float lengthSquared = glm::dot(ab, ab);
    if (lengthSquared <= 1e-12f) {
        return {vertexA, glm::length(center - vertexA)};
    }

    const float t = std::clamp(
        glm::dot(center - vertexA, ab) / lengthSquared, 0.0f, 1.0f);
    const glm::vec2 point = vertexA + t * ab;
    return {point, glm::length(center - point)};
}

} // namespace


CollisionManifold CollisionDetector::Detect(
    const Collider& first,
    const Transform2D& firstTransform,
    const Collider& second,
    const Transform2D& secondTransform) {
    if (first.Type() == ColliderType::Circle &&
        second.Type() == ColliderType::Circle) {
        return CircleVsCircle(
            std::get<CircleCollider>(first.data),
            firstTransform,
            std::get<CircleCollider>(second.data), 
            secondTransform
        );
    }
    if (first.Type() == ColliderType::Circle
        && second.Type() == ColliderType::Polygon) {
        return CircleVsPolygon(
            std::get<CircleCollider>(first.data),
            firstTransform,
            std::get<PolygonCollider>(second.data),
            secondTransform
        );
    }
    if (first.Type() == ColliderType::Polygon &&
        second.Type() == ColliderType::Circle) {
        auto manifold = CircleVsPolygon(
            std::get<CircleCollider>(second.data),
            secondTransform,
            std::get<PolygonCollider>(first.data), 
            firstTransform
        );
        manifold.normal = -manifold.normal;
        return manifold;
    }
    if (first.Type() == ColliderType::Polygon &&
        second.Type() == ColliderType::Polygon) {
        return PolygonVsPolygon(
            std::get<PolygonCollider>(first.data),
            firstTransform,
            std::get<PolygonCollider>(second.data),
            secondTransform
        );
    }
    return CollisionManifold{};
}

CollisionManifold CollisionDetector::CircleVsCircle(
    const CircleCollider& first,
    const Transform2D& firstTransform,
    const CircleCollider& second,
    const Transform2D& secondTransform) {
    glm::vec2 distance = secondTransform.position - firstTransform.position;
    float length2 = glm::dot(distance, distance);
    if (length2 >= (first.radius + second.radius) * 
        (first.radius + second.radius)) {
        return CollisionManifold{};
    }

    CollisionManifold manifold{};
    manifold.colliding = true;
    glm::vec2 normal;
    float length = std::sqrt(length2);
    if (length < 1e-6f) {
        normal = glm::vec2{1.0f, 0.0f};
    } else {
        normal = distance / length;
    }
    manifold.normal = normal;
    manifold.contactPoint = firstTransform.position + 
        normal * first.radius;
    manifold.penetration = first.radius + second.radius - length;

    return manifold;
}

CollisionManifold CollisionDetector::CircleVsPolygon(
    const CircleCollider& circle,
    const Transform2D& circleTransform,
    const PolygonCollider& polygon,
    const Transform2D& polygonTransform) {
    const glm::vec2& center = circleTransform.position;
    const float radius = circle.radius;
    const std::size_t n = polygon.vertices.size();
    if (n < 3 || radius <= 0.0f) return {};

    std::vector<glm::vec2> vertices;
    vertices.reserve(n);
    for (const auto& vertex : polygon.vertices) {
        vertices.push_back(WorldPoint(polygonTransform, vertex));
    }

    bool centerInsidePolygon = true;
    glm::vec2 closestPoint{0.0f};
    glm::vec2 closestOutwardNormal{0.0f};
    float minDistance = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& vertexA = vertices[i];
        const auto& vertexB = vertices[(i + 1) % n];
        const glm::vec2 edge = vertexB - vertexA;
        const float edgeLengthSquared = glm::dot(edge, edge);
        if (edgeLengthSquared <= 1e-12f) continue;

        // Polygon vertices are counterclockwise, so this is the outward normal.
        const glm::vec2 outwardNormal = glm::normalize(
            glm::vec2{edge.y, -edge.x});
        const float signedDistance = glm::dot(center - vertexA, outwardNormal);
        if (signedDistance > 0.0f) centerInsidePolygon = false;
        if (signedDistance > radius) return {};

        auto [point, dist] = ClosestPointOnSegment(center, vertexA, vertexB);
        if (dist < minDistance) {
            minDistance = dist;
            closestPoint = point;
            closestOutwardNormal = outwardNormal;
        }
    }
    if (minDistance == std::numeric_limits<float>::max()) return {};

    if (centerInsidePolygon) {
        const glm::vec2 normal = minDistance > 1e-6f
            ? glm::normalize(center - closestPoint)
            : -closestOutwardNormal;
        const glm::vec2 contactPoint = minDistance > 1e-6f
            ? center - normal * radius
            : closestPoint;
        return {
            true,
            normal,
            contactPoint,
            minDistance + radius
        };
    }

    if (minDistance >= radius) return {};

    const glm::vec2 normal = minDistance > 1e-6f
        ? glm::normalize(closestPoint - center)
        : -closestOutwardNormal;
    const glm::vec2 contactPoint = minDistance > 1e-6f
        ? center + normal * radius
        : closestPoint;
    return {
        true,
        normal,
        contactPoint,
        radius - minDistance
    };
}

CollisionManifold CollisionDetector::PolygonVsPolygon(
    const PolygonCollider& first,
    const Transform2D& firstTransform,
    const PolygonCollider& second,
    const Transform2D& secondTransform) {
    std::vector<glm::vec2> axes;
    axes.reserve(first.normals.size() + second.normals.size());
    for (const auto& normal : first.normals) {
        axes.push_back(glm::normalize(MathHelper::Rotate(normal, firstTransform.rotation)));
    }
    for (const auto& normal : second.normals) {
        axes.push_back(glm::normalize(MathHelper::Rotate(normal, secondTransform.rotation)));
    }

    std::vector<glm::vec2> verticesA, verticesB;
    verticesA.reserve(first.vertices.size());
    verticesB.reserve(second.vertices.size());
    for (const auto& vertex : first.vertices) {
        verticesA.push_back(WorldPoint(firstTransform, vertex));
    }
    for (const auto& vertex : second.vertices) {
        verticesB.push_back(WorldPoint(secondTransform, vertex));
    }

    glm::vec2 bestAxis{0.0f};
    float minPenetration = std::numeric_limits<float>::max();
    for (const auto& axis : axes) {
        float min1, max1, min2, max2;
        ProjectPolygon(verticesA, axis, min1, max1);
        ProjectPolygon(verticesB, axis, min2, max2);

        const float overlap = std::min(max1, max2) - std::max(min1, min2);
        if (overlap <= 0.0f) {
            return CollisionManifold{false};
        }

        if (overlap < minPenetration) {
            minPenetration = overlap;
            bestAxis = axis;
        }
    }

    if (glm::length(bestAxis) < 1e-6f) {
        return CollisionManifold{false};
    }

    glm::vec2 centroidA = MathHelper::CalculatePolygonCentroid(verticesA);
    glm::vec2 centroidB = MathHelper::CalculatePolygonCentroid(verticesB);
    glm::vec2 distance = centroidB - centroidA;
    if (glm::dot(distance, bestAxis) < 0.0f) {
        bestAxis *= -1.0f;
    }

    CollisionManifold manifold;
    manifold.colliding = true;
    manifold.normal = glm::normalize(bestAxis);
    manifold.penetration = minPenetration;
    manifold.contactPoint = ComputeContactPoint(verticesA, verticesB, manifold.normal);

    return manifold;
}
