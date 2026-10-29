#include "Physics/Collision/CollisionDetector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace
{

struct FaceQuery {
    float separation = -std::numeric_limits<float>::max();
    int polygon = -1;   // 0: A, 1: B
    int face = -1;      // 该多边形的边索引
    glm::vec2 normal{0.0f};
};

struct ClipVertex {
    glm::vec2 point{0.0f};
    // 0/1 are original incident-edge vertices; 2 is a clipped intersection.
    uint8_t incidentVertex = 0;
    // 0 is original; 1/2 identify the reference side that created the point.
    uint8_t clipSide = 0;
};

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

void SwapManifoldBodies(CollisionManifold& manifold) {
    manifold.normal = -manifold.normal;
    manifold.referenceIsA ^= 1;
    for (std::size_t i = 0; i < manifold.pointCount; ++i) {
        auto& point = manifold.points[i];
        std::swap(point.localPointA, point.localPointB);
        point.feature.referenceIsA ^= 1;
    }
}

bool QueryFace(const std::vector<glm::vec2>& selfVertices,
    const std::vector<glm::vec2>& selfNormals,
    const std::vector<glm::vec2>& otherVertices,
    int polygonIndex,
    FaceQuery& best) {
    bool found = false;
    for (std::size_t i = 0; i < selfNormals.size(); ++i) {
        const auto& normal = selfNormals[i];
        if (!std::isfinite(normal.x)
            || !std::isfinite(normal.y)) continue;

        float projectionA = -std::numeric_limits<float>::max();
        float projectionB = +std::numeric_limits<float>::max();
        for (const auto& vertex : selfVertices) {
            if (glm::dot(normal, vertex) > projectionA) {
                projectionA = glm::dot(normal, vertex);
            }
        }
        for (const auto& vertex : otherVertices) {
            if (glm::dot(normal, vertex) < projectionB) {
                projectionB = glm::dot(normal, vertex);
            }
        }
        if (projectionA < projectionB) continue;

        found = true;
        float separation = projectionB - projectionA;
        if (separation > best.separation) {
            best.separation = separation;
            best.normal = normal;
            best.polygon = polygonIndex;
            best.face = i;
        }
    }
    return found;
}

int FindIncidentEdge(const std::vector<glm::vec2>& normals,
                     const glm::vec2& referenceNormal) {
    if (normals.empty()) return -1;

    int best = -1;
    float minDot = std::numeric_limits<float>::max();
    for (std::size_t i = 0; i < normals.size(); ++i) {
        const float dot = glm::dot(referenceNormal, normals[i]);
        if (dot < minDot) {
            minDot = dot;
            best = static_cast<int>(i);
        }
    }
    return best;
}

// 保留 dot(normal, p) <= offset 的部分。
// 输入 2 个点，输出最多 2 个点。
int ClipSegmentToLine(ClipVertex out[2],
                      const ClipVertex in[2],
                      const glm::vec2& normal,
                      float offset,
                      uint8_t clipSide) {
    int count = 0;
    const float d0 = glm::dot(normal, in[0].point) - offset;
    const float d1 = glm::dot(normal, in[1].point) - offset;

    if (d0 <= 0.0f) out[count++] = in[0];
    if (d1 <= 0.0f) out[count++] = in[1];

    // 一内一外，产生一个交点。
    if (d0 * d1 < 0.0f) {
        const float t = d0 / (d0 - d1);
        ClipVertex clipped;
        clipped.point = in[0].point + t * (in[1].point - in[0].point);
        clipped.incidentVertex = 2;
        clipped.clipSide = clipSide;
        out[count++] = clipped;
    }

    return count;
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
        SwapManifoldBodies(manifold);
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
    float length = std::sqrt(length2);
    glm::vec2 normal = length < 1e-6 ? glm::vec2{1.0f, 0.0f}
        : distance / length;
    const float penetration = first.radius + second.radius - length;
    manifold.colliding = true;
    manifold.normal = normal;
    manifold.pointCount = 1;

    auto& point = manifold.points[0];
    const glm::vec2 pA = firstTransform.position + normal * first.radius;
    const glm::vec2 pB = secondTransform.position - normal * second.radius;
    glm::vec2 worldPosition = (pA + pB) * 0.5f;
    point.position = worldPosition;
    point.localPointA = LocalPoint(firstTransform, worldPosition);
    point.localPointB = LocalPoint(secondTransform, worldPosition);
    point.separation = -penetration;

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
    std::size_t closestEdge = 0;
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
            closestEdge = i;
        }
    }
    if (minDistance == std::numeric_limits<float>::max()) return {};

    glm::vec2 normal{0.0f};
    if (centerInsidePolygon) {
        normal = minDistance > 1e-6f
            ? glm::normalize(center - closestPoint)
            : -closestOutwardNormal;
    } else {
        if (minDistance >= radius) return {};
        normal = minDistance > 1e-6f
            ? glm::normalize(closestPoint - center)
            : -closestOutwardNormal;
    }

    CollisionManifold manifold;
    manifold.colliding = true;
    manifold.normal = normal;
    manifold.pointCount = 1;

    auto& point = manifold.points[0];
    glm::vec2 pA = center + normal * radius;
    glm::vec2 pB = closestPoint;
    point.position = (pA + pB) * 0.5f;
    point.localPointA = LocalPoint(circleTransform, pA);
    point.localPointB = LocalPoint(polygonTransform, pB);
    point.separation = -glm::dot(normal, pA - pB);

    point.feature.referenceIsA = 0;
    point.feature.referenceFace = static_cast<uint16_t>(closestEdge);
    point.feature.incidentFace = 0;
    point.feature.incidentVertex = 0;
    point.feature.clipSide = 0;
    manifold.referenceIsA = 0;

    return manifold;
}

CollisionManifold CollisionDetector::PolygonVsPolygon(
    const PolygonCollider& first,
    const Transform2D& firstTransform,
    const PolygonCollider& second,
    const Transform2D& secondTransform) {
    if (first.vertices.size() < 3 || second.vertices.size() < 3) {
        return CollisionManifold{};
    }
    if (first.normals.size() != first.vertices.size()
        || second.normals.size() != second.vertices.size()) {
        return CollisionManifold{};
    }

    std::vector<glm::vec2> verticesA, verticesB;
    std::vector<glm::vec2> normalsA, normalsB;
    verticesA.reserve(first.vertices.size());
    verticesB.reserve(second.vertices.size());
    normalsA.reserve(first.normals.size());
    normalsB.reserve(second.normals.size());

    for (std::size_t i = 0; i < first.vertices.size(); ++i) {
        verticesA.push_back(WorldPoint(firstTransform, first.vertices[i]));
        normalsA.push_back(glm::normalize(
            MathHelper::Rotate(first.normals[i], firstTransform.rotation)));
    }
    for (std::size_t i = 0; i < second.vertices.size(); ++i) {
        verticesB.push_back(WorldPoint(secondTransform, second.vertices[i]));
        normalsB.push_back(glm::normalize(
            MathHelper::Rotate(second.normals[i], secondTransform.rotation)));
    }

    FaceQuery best;
    if (!QueryFace(verticesA, normalsA, verticesB, 0, best)) {
        return CollisionManifold{};
    }
    if (!QueryFace(verticesB, normalsB, verticesA, 1, best)) {
        return CollisionManifold{};
    }
    if (best.polygon < 0 || best.face < 0) {
        return CollisionManifold{};
    }

    const bool referenceIsA = best.polygon == 0;

    const float referenceNormalLengthSquared =
        glm::dot(best.normal, best.normal);
    if (!(referenceNormalLengthSquared > 1e-12f)
        || !std::isfinite(referenceNormalLengthSquared)) {
        return CollisionManifold{};
    }
    const glm::vec2 referenceNormal =
        best.normal * glm::inversesqrt(referenceNormalLengthSquared);
    const glm::vec2 manifoldNormal = referenceIsA
        ? referenceNormal
        : -referenceNormal; // 统一成 A -> B

    const auto& referenceVertices = referenceIsA ? verticesA : verticesB;
    const auto& incidentVertices  = referenceIsA ? verticesB : verticesA;
    const auto& incidentNormals   = referenceIsA ? normalsB : normalsA;

    const int referenceFace = best.face;
    const int incidentFace = FindIncidentEdge(
        incidentNormals, referenceNormal);
    if (referenceFace < 0 || incidentFace < 0) {
        return CollisionManifold{};
    }

    const std::size_t referenceCount = referenceVertices.size();
    const glm::vec2 v1 = referenceVertices[referenceFace];
    const glm::vec2 v2 = referenceVertices[(referenceFace + 1) % referenceCount];
    const glm::vec2 referenceEdge = v2 - v1;
    if (glm::dot(referenceEdge, referenceEdge) < 1e-12f) {
        return CollisionManifold{};
    }
    const glm::vec2 tangent = glm::normalize(referenceEdge);

    const std::size_t incidentCount = incidentVertices.size();
    const glm::vec2 q0 = incidentVertices[incidentFace];
    const glm::vec2 q1 = incidentVertices[(incidentFace + 1) % incidentCount];

    ClipVertex incident[2] = {
        {q0, 0}, {q1, 1}
    };

    ClipVertex clipped1[2];
    const int count1 = ClipSegmentToLine(
        clipped1, incident, -tangent, -glm::dot(tangent, v1), 1);
    if (count1 == 0) return CollisionManifold{};

    ClipVertex clipped2[2];
    const int count2 = ClipSegmentToLine(
        clipped2, clipped1, tangent, glm::dot(tangent, v2), 2);
    if (count2 == 0) return CollisionManifold{};

    CollisionManifold manifold;
    manifold.normal = manifoldNormal;
    manifold.referenceIsA = referenceIsA ? 1 : 0;

    constexpr float contactTolerance = 0.01f;

    for (int i = 0; i < count2; ++i) {
        const glm::vec2& q = clipped2[i].point;
        const float separation = glm::dot(q - v1, referenceNormal);
        if (separation > contactTolerance) {
            continue; // 这个点已经离开参考面，不是有效接触
        }

        ManifoldPoint point;
        point.separation = separation;
        point.position = q - 0.5f * separation * referenceNormal;

        // q 在入射体上，pRef 在参考面上。
        const glm::vec2 pRef = q - separation * referenceNormal;
        if (referenceIsA) {
            // 参考体是 A，入射体是 B
            point.localPointA = LocalPoint(firstTransform, pRef);
            point.localPointB = LocalPoint(secondTransform, q);
        } else {
            // 参考体是 B，入射体是 A
            point.localPointA = LocalPoint(firstTransform, q);
            point.localPointB = LocalPoint(secondTransform, pRef);
        }

        point.feature.referenceIsA = referenceIsA ? 1 : 0;
        point.feature.referenceFace = static_cast<uint16_t>(referenceFace);
        point.feature.incidentFace = static_cast<uint16_t>(incidentFace);
        point.feature.incidentVertex = clipped2[i].incidentVertex;
        point.feature.clipSide = clipped2[i].clipSide;

        // 去重：两个裁剪点可能因为数值误差几乎重合。
        bool duplicate = false;
        for (uint8_t j = 0; j < manifold.pointCount; ++j) {
            const glm::vec2 delta = point.position - manifold.points[j].position;
            if (glm::dot(delta, delta) < 1e-8f) {
                duplicate = true;
                break;
            }
        }
        if (duplicate) continue;

        if (manifold.pointCount >= MaxManifoldPoints) break;
        manifold.points[manifold.pointCount++] = point;
    }

    manifold.colliding = manifold.pointCount > 0;
    return manifold;
}
