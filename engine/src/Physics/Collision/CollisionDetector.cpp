#include "Physics/Collision/CollisionDetector.hpp"

#include <limits>
#include <glm/gtc/matrix_transform.hpp>

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
    float minProj = std::numeric_limits<float>::max();
    glm::vec2 contactA = verticesA[0];
    for (const glm::vec2& v : verticesA) {
        float proj = glm::dot(v, axis);
        if (proj < minProj) {
            minProj = proj;
            contactA = v;
        }
    }
    
    float maxProj = -std::numeric_limits<float>::max();
    glm::vec2 contactB = verticesB[0];
    for (const glm::vec2& v : verticesB) {
        float proj = glm::dot(v, axis);
        if (proj > maxProj) {
            maxProj = proj;
            contactB = v;
        }
    }
    
    return (contactA + contactB) * 0.5f;
}

} // namespace


CollisionManifold CollisionDetector::Detect(
    const Collider& first,
    const Transform2D& firstTransform,
    const Collider& second,
    const Transform2D& secondTransform) {
    if (first.type == ColliderType::Circle && 
        second.type == ColliderType::Circle) {
        return CircleVsCircle(
            std::get<CircleCollider>(first.data),
            firstTransform,
            std::get<CircleCollider>(second.data), 
            secondTransform
        );
    }
    if (first.type == ColliderType::Circle 
        && second.type == ColliderType::Polygon) {
        return CircleVsPolygon(
            std::get<CircleCollider>(first.data),
            firstTransform,
            std::get<PolygonCollider>(second.data),
            secondTransform
        );
    }
    if (first.type == ColliderType::Polygon && 
        second.type == ColliderType::Circle) {
        return CircleVsPolygon(
            std::get<CircleCollider>(second.data),
            secondTransform,
            std::get<PolygonCollider>(first.data), 
            firstTransform
        );
    }
    if (first.type == ColliderType::Polygon && 
        second.type == ColliderType::Polygon) {
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
    // TODO: implement circle-vs-convex-polygon detection with nearest-point projection.
    // This stub keeps the API safe until the narrow-phase algorithm is added.
    return CollisionManifold{};
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