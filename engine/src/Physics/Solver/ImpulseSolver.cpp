#include "Physics/Solver/ImpulseSolver.hpp"

#include <algorithm>
#include <cmath>

namespace
{

float Cross(const glm::vec2& a, const glm::vec2& b) {
    return a.x * b.y - a.y * b.x;
}

float EffectiveInverseMass(const RigidBody& body) {
    return body.IsDynamic() ? std::max(body.inverseMass, 0.0f) : 0.0f;
}

float EffectiveInverseInertia(const RigidBody& body) {
    return body.IsDynamic() ? std::max(body.inverseInertia, 0.0f) : 0.0f;
}

glm::vec2 VelocityAtPoint(
    const RigidBody& body, const glm::vec2& offset) {
    if (body.IsStatic()) return glm::vec2{0.0f};
    return body.linearVelocity + glm::vec2{
        -body.angularVelocity * offset.y,
         body.angularVelocity * offset.x
    };
}

} // namespace


void ImpulseSolver::SolvePosition(
    PhysicsBody& bodyA, PhysicsBody& bodyB,
    const CollisionManifold& manifold) {
    if (!manifold.colliding) return;
    auto& first = bodyA.rigidBody;
    auto& second = bodyB.rigidBody;
    const float firstInverseMass = first.IsDynamic()
        ? std::max(first.inverseMass, 0.0f)
        : 0.0f;
    const float secondInverseMass = second.IsDynamic()
        ? std::max(second.inverseMass, 0.0f)
        : 0.0f;
    const float inverseMassSum = firstInverseMass + secondInverseMass;
    if (inverseMassSum <= 0.0f) return;

    const glm::vec2 correction = manifold.normal * manifold.penetration;
    first.Translate(-correction * (firstInverseMass / inverseMassSum));
    second.Translate(correction * (secondInverseMass / inverseMassSum));
}

void ImpulseSolver::SolveVelocity(
    PhysicsBody& first, PhysicsBody& second,
    const CollisionManifold& manifold) {
    if (!manifold.colliding) return;

    auto& bodyA = first.rigidBody;
    auto& bodyB = second.rigidBody;
    const float normalLengthSquared = glm::dot(manifold.normal, manifold.normal);
    if (normalLengthSquared <= 1e-12f) return;
    const glm::vec2 normal = manifold.normal * glm::inversesqrt(normalLengthSquared);

    const glm::vec2 centroidA = WorldPoint(bodyA.transform, first.collider.centroid);
    const glm::vec2 centroidB = WorldPoint(bodyB.transform, second.collider.centroid);
    const glm::vec2 rA = manifold.contactPoint - centroidA;
    const glm::vec2 rB = manifold.contactPoint - centroidB;

    const float inverseMassA = EffectiveInverseMass(bodyA);
    const float inverseMassB = EffectiveInverseMass(bodyB);
    const float inverseInertiaA = EffectiveInverseInertia(bodyA);
    const float inverseInertiaB = EffectiveInverseInertia(bodyB);
    const float inverseMassSum = inverseMassA + inverseMassB;

    glm::vec2 relativeVelocity = VelocityAtPoint(bodyB, rB)
        - VelocityAtPoint(bodyA, rA);
    const float velocityAlongNormal = glm::dot(relativeVelocity, normal);
    if (velocityAlongNormal >= 0.0f) return;

    const float rACrossNormal = Cross(rA, normal);
    const float rBCrossNormal = Cross(rB, normal);
    const float normalDenominator = inverseMassSum
        + rACrossNormal * rACrossNormal * inverseInertiaA
        + rBCrossNormal * rBCrossNormal * inverseInertiaB;
    if (normalDenominator <= 1e-12f) return;

    const float restitution = std::min(
        std::max(bodyA.restitution, 0.0f),
        std::max(bodyB.restitution, 0.0f));
    const float normalImpulseMagnitude =
        -(1.0f + restitution) * velocityAlongNormal / normalDenominator;
    const glm::vec2 normalImpulse = normalImpulseMagnitude * normal;

    if (bodyA.IsDynamic()) {
        bodyA.AddImpulse(-normalImpulse);
        bodyA.AddAngularImpulse(-Cross(rA, normalImpulse));
    }
    if (bodyB.IsDynamic()) {
        bodyB.AddImpulse(normalImpulse);
        bodyB.AddAngularImpulse(Cross(rB, normalImpulse));
    }

    // Recompute contact velocity after the normal impulse before solving friction.
    relativeVelocity = VelocityAtPoint(bodyB, rB) - VelocityAtPoint(bodyA, rA);
    const glm::vec2 relativeVelocityTangent = relativeVelocity
        - normal * glm::dot(relativeVelocity, normal);
    const float tangentSpeed = glm::length(relativeVelocityTangent);
    if (tangentSpeed <= 1e-6f) return;

    const glm::vec2 tangent = relativeVelocityTangent / tangentSpeed;
    const float rACrossTangent = Cross(rA, tangent);
    const float rBCrossTangent = Cross(rB, tangent);
    const float tangentDenominator = inverseMassSum
        + rACrossTangent * rACrossTangent * inverseInertiaA
        + rBCrossTangent * rBCrossTangent * inverseInertiaB;
    if (tangentDenominator <= 1e-12f) return;

    const float friction = std::sqrt(
        std::max(bodyA.friction, 0.0f) * std::max(bodyB.friction, 0.0f));
    const float maxFrictionImpulse = friction * normalImpulseMagnitude;
    const float tangentImpulseMagnitude = glm::clamp(
        -tangentSpeed / tangentDenominator,
        -maxFrictionImpulse, maxFrictionImpulse);
    const glm::vec2 tangentImpulse = tangentImpulseMagnitude * tangent;

    if (bodyA.IsDynamic()) {
        bodyA.AddImpulse(-tangentImpulse);
        bodyA.AddAngularImpulse(-Cross(rA, tangentImpulse));
    }
    if (bodyB.IsDynamic()) {
        bodyB.AddImpulse(tangentImpulse);
        bodyB.AddAngularImpulse(Cross(rB, tangentImpulse));
    }
}
