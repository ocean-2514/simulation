#include "Physics/Collision/ContactConstraint.hpp"

#include "Physics/PhysicsWorld.hpp"

#include <algorithm>
#include <cmath>

namespace
{

bool IsFinite(const glm::vec2& value) {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

} // namespace

void ContactConstraint::Prepare(SolverContext& context) {
    m_impulseState = {};
    if (!m_manifold.colliding || m_bodyA == m_bodyB
        || !IsFinite(m_manifold.contactPoint)
        || !std::isfinite(m_manifold.penetration)) {
        return;
    }

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    const float normalLengthSquared = glm::dot(
        m_manifold.normal, m_manifold.normal);
    if (!(normalLengthSquared > 1e-12f)
        || !std::isfinite(normalLengthSquared)) {
        return;
    }

    ContactImpulseState& state = m_impulseState;
    state.normal = m_manifold.normal * glm::inversesqrt(normalLengthSquared);
    state.tangent = {-state.normal.y, state.normal.x};

    const glm::vec2 centroidA = WorldPoint(
        rigidA.transform, bodyA->collider.centroid);
    const glm::vec2 centroidB = WorldPoint(
        rigidB.transform, bodyB->collider.centroid);
    state.rA = m_manifold.contactPoint - centroidA;
    state.rB = m_manifold.contactPoint - centroidB;

    state.inverseMassA = EffectiveInverseMass(rigidA);
    state.inverseMassB = EffectiveInverseMass(rigidB);
    state.inverseInertiaA = EffectiveInverseInertia(rigidA);
    state.inverseInertiaB = EffectiveInverseInertia(rigidB);
    state.inverseMassSum = state.inverseMassA + state.inverseMassB;

    const float rACrossNormal = Cross(state.rA, state.normal);
    const float rBCrossNormal = Cross(state.rB, state.normal);
    const float normalDenominator = state.inverseMassSum
        + rACrossNormal * rACrossNormal * state.inverseInertiaA
        + rBCrossNormal * rBCrossNormal * state.inverseInertiaB;
    if (!(normalDenominator > 1e-12f)
        || !std::isfinite(normalDenominator)) {
        m_impulseState = {};
        return;
    }
    state.normalEffectiveMass = 1.0f / normalDenominator;

    const float rACrossTangent = Cross(state.rA, state.tangent);
    const float rBCrossTangent = Cross(state.rB, state.tangent);
    const float tangentDenominator = state.inverseMassSum
        + rACrossTangent * rACrossTangent * state.inverseInertiaA
        + rBCrossTangent * rBCrossTangent * state.inverseInertiaB;
    if (tangentDenominator > 1e-12f && std::isfinite(tangentDenominator)) {
        state.tangentEffectiveMass = 1.0f / tangentDenominator;
    }

    const float frictionA = NonNegativeFinite(rigidA.friction);
    const float frictionB = NonNegativeFinite(rigidB.friction);
    state.friction = std::sqrt(frictionA * frictionB);

    const glm::vec2 relativeVelocity = rigidB.VelocityAtPoint(state.rB)
        - rigidA.VelocityAtPoint(state.rA);
    const float velocityAlongNormal = glm::dot(relativeVelocity, state.normal);
    const float restitutionA = NonNegativeFinite(rigidA.restitution);
    const float restitutionB = NonNegativeFinite(rigidB.restitution);
    constexpr float restitutionThreshold = 1.0f;
    if (velocityAlongNormal < -restitutionThreshold) {
        state.restitutionTarget = std::min(restitutionA, restitutionB)
            * -velocityAlongNormal;
    }
    state.active = true;
}

void ContactConstraint::WarmStart(SolverContext& context) {
    ContactImpulseState& state = m_impulseState;
    if (!state.active || (state.accumulatedNormalImpulse == 0.0f
        && state.accumulatedTangentImpulse == 0.0f)) {
        return;
    }

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    const glm::vec2 impulse = state.accumulatedNormalImpulse * state.normal
        + state.accumulatedTangentImpulse * state.tangent;
    ApplyVelocityImpulse(bodyA->rigidBody, bodyB->rigidBody,
        state.rA, state.rB, impulse);
}

void ContactConstraint::SolveVelocity(SolverContext& context) {
    ContactImpulseState& state = m_impulseState;
    if (!state.active) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    glm::vec2 relativeVelocity = rigidB.VelocityAtPoint(state.rB)
        - rigidA.VelocityAtPoint(state.rA);
    const float velocityAlongNormal = glm::dot(relativeVelocity, state.normal);
    const float oldNormalImpulse = state.accumulatedNormalImpulse;
    state.accumulatedNormalImpulse = std::max(
        oldNormalImpulse + state.normalEffectiveMass
            * (state.restitutionTarget - velocityAlongNormal),
        0.0f);
    const glm::vec2 normalImpulse =
        (state.accumulatedNormalImpulse - oldNormalImpulse) * state.normal;
    ApplyVelocityImpulse(rigidA, rigidB, state.rA, state.rB, normalImpulse);

    if (state.tangentEffectiveMass <= 0.0f) return;
    relativeVelocity = rigidB.VelocityAtPoint(state.rB)
        - rigidA.VelocityAtPoint(state.rA);
    const float velocityAlongTangent = glm::dot(relativeVelocity, state.tangent);
    const float oldTangentImpulse = state.accumulatedTangentImpulse;
    const float maxFrictionImpulse =
        state.friction * state.accumulatedNormalImpulse;
    state.accumulatedTangentImpulse = glm::clamp(
        oldTangentImpulse - state.tangentEffectiveMass * velocityAlongTangent,
        -maxFrictionImpulse, maxFrictionImpulse);
    const glm::vec2 tangentImpulse =
        (state.accumulatedTangentImpulse - oldTangentImpulse) * state.tangent;
    ApplyVelocityImpulse(rigidA, rigidB, state.rA, state.rB, tangentImpulse);
}

void ContactConstraint::SolvePosition(SolverContext& context) {
    const ContactImpulseState& state = m_impulseState;
    if (!state.active) return;

    const float slop = NonNegativeFinite(context.settings.slop);
    const float correctionFactor = NonNegativeFinite(context.settings.baumgarte);
    const float penetration = std::max(m_manifold.penetration - slop, 0.0f);
    if (penetration <= 0.0f || correctionFactor <= 0.0f) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    const glm::vec2 centroidA = WorldPoint(
        rigidA.transform, bodyA->collider.centroid);
    const glm::vec2 centroidB = WorldPoint(
        rigidB.transform, bodyB->collider.centroid);
    const glm::vec2 rA = m_manifold.contactPoint - centroidA;
    const glm::vec2 rB = m_manifold.contactPoint - centroidB;
    const float rACrossNormal = Cross(rA, state.normal);
    const float rBCrossNormal = Cross(rB, state.normal);
    const float inverseMassSum = state.inverseMassSum
        + rACrossNormal * rACrossNormal * state.inverseInertiaA
        + rBCrossNormal * rBCrossNormal * state.inverseInertiaB;
    if (!(inverseMassSum > 1e-12f) || !std::isfinite(inverseMassSum)) return;

    const glm::vec2 positionImpulse =
        (correctionFactor * penetration / inverseMassSum) * state.normal;
    ApplyPositionImpulse(rigidA, bodyA->collider.centroid,
        -positionImpulse, rA, state.inverseMassA, state.inverseInertiaA);
    ApplyPositionImpulse(rigidB, bodyB->collider.centroid,
        positionImpulse, rB, state.inverseMassB, state.inverseInertiaB);
}
