#pragma once

#include "Physics/Collider.hpp"
#include "Physics/Collision/CollisionManifold.hpp"

class CollisionDetector
{
public:
    static CollisionManifold Detect(
        const Collider& first,
        const Transform2D& firstTransform,
        const Collider& second,
        const Transform2D& secondTransform);

private:
    static CollisionManifold CircleVsCircle(
        const CircleCollider& first,
        const Transform2D& firstTransform,
        const CircleCollider& second,
        const Transform2D& secondTransform);

    static CollisionManifold CircleVsPolygon(
        const CircleCollider& circle,
        const Transform2D& circleTransform,
        const PolygonCollider& polygon,
        const Transform2D& polygonTransform);

    static CollisionManifold PolygonVsPolygon(
        const PolygonCollider& first,
        const Transform2D& firstTransform,
        const PolygonCollider& second,
        const Transform2D& secondTransform);
};