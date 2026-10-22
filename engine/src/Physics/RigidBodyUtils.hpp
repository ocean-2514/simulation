#pragma once

#include "Physics/RigidBody.hpp"
#include "Utils/MathHelper.hpp"

inline float NonNegativeFinite(float value) {
    return std::isfinite(value) ? std::max(value, 0.0f) : 0.0f;
}

inline float EffectiveInverseMass(const RigidBody& body) {
    return body.IsDynamic() ? NonNegativeFinite(body.inverseMass) : 0.0f;
}

inline float EffectiveInverseInertia(const RigidBody& body) {
    return body.IsDynamic() ? NonNegativeFinite(body.inverseInertia) : 0.0f;
}

inline float Cross(const glm::vec2& a, const glm::vec2& b) {
    return a.x * b.y - a.y * b.x;
}

inline void ApplyVelocityImpulse(
    RigidBody& bodyA,
    RigidBody& bodyB,
    const glm::vec2& rA,
    const glm::vec2& rB,
    const glm::vec2& impulseOnB) {
    if (bodyA.IsDynamic()) {
        bodyA.AddImpulse(-impulseOnB);
        bodyA.AddAngularImpulse(-Cross(rA, impulseOnB));
    }
    if (bodyB.IsDynamic()) {
        bodyB.AddImpulse(impulseOnB);
        bodyB.AddAngularImpulse(Cross(rB, impulseOnB));
    }
}

inline void ApplyPositionImpulse(
    RigidBody& body,
    const glm::vec2& localCentroid,
    const glm::vec2& impulse,
    const glm::vec2& centerOffset,
    float inverseMass,
    float inverseInertia) {
    if (inverseMass <= 0.0f && inverseInertia <= 0.0f) return;

    const glm::vec2 center = WorldPoint(body.transform, localCentroid);
    const glm::vec2 newCenter = center + inverseMass * impulse;
    const float angleDelta = inverseInertia * Cross(centerOffset, impulse);
    body.transform.rotation += angleDelta;
    body.transform.position = newCenter
        - MathHelper::Rotate(localCentroid, body.transform.rotation);
}