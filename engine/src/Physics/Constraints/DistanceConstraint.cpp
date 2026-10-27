#include "Physics/Constraints/DistanceConstraint.hpp"

#include <algorithm>
#include <cmath>

void DistanceConstraint::Prepare(SolverContext& context) {
    m_isActive = false;
    m_effectiveMass = 0.0f;
    m_bias = 0.0f;

    if (!(context.deltaTime > 0.0f) || !std::isfinite(context.deltaTime)
        || m_bodyA == m_bodyB || m_targetLength < 0.0f
        || !std::isfinite(m_targetLength)) {
        m_accumulatedImpulse = 0.0f;
        m_previousDeltaTime = 0.0f;
        return;
    }

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) {
        m_accumulatedImpulse = 0.0f;
        m_previousDeltaTime = 0.0f;
        return;
    }

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    const glm::vec2 worldAnchorA = WorldPoint(
        rigidA.transform, m_localAnchorA);
    const glm::vec2 worldAnchorB = WorldPoint(
        rigidB.transform, m_localAnchorB);
    const glm::vec2 worldCentroidA = WorldPoint(
        rigidA.transform, bodyA->collider.centroid);
    const glm::vec2 worldCentroidB = WorldPoint(
        rigidB.transform, bodyB->collider.centroid);
    m_rA = worldAnchorA - worldCentroidA;
    m_rB = worldAnchorB - worldCentroidB;

    const glm::vec2 separation = worldAnchorB - worldAnchorA;
    const float length = glm::length(separation);
    if (length > 1e-6f) {
        m_normal = separation / length;
    }

    const float crossA = MathHelper::Cross(m_rA, m_normal);
    const float crossB = MathHelper::Cross(m_rB, m_normal);
    const float inverseMassSum = EffectiveInverseMass(rigidA)
        + EffectiveInverseMass(rigidB)
        + EffectiveInverseInertia(rigidA) * crossA * crossA
        + EffectiveInverseInertia(rigidB) * crossB * crossB;
    if (inverseMassSum <= 1e-6f) {
        m_accumulatedImpulse = 0.0f;
        m_previousDeltaTime = 0.0f;
        return;
    }

    m_effectiveMass = 1.0f / inverseMassSum;
    m_isActive = true;

    const float error = length - m_targetLength;
    const float slop = std::max(context.settings.slop, 0.0f);
    const float biasFactor = std::max(context.settings.baumgarte, 0.0f);
    if (error > slop) {
        m_bias = biasFactor * (error - slop) / context.deltaTime;
    } else if (error < -slop) {
        m_bias = biasFactor * (error + slop) / context.deltaTime;
    }

    if (m_previousDeltaTime > 0.0f) {
        m_accumulatedImpulse *= context.deltaTime / m_previousDeltaTime;
    } else {
        m_accumulatedImpulse = 0.0f;
    }
    m_previousDeltaTime = context.deltaTime;
}

void DistanceConstraint::WarmStart(SolverContext& context) {
    if (!m_isActive || m_accumulatedImpulse == 0.0f) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    const glm::vec2 impulse = m_accumulatedImpulse * m_normal;
    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    if (rigidA.IsDynamic()) {
        rigidA.AddImpulse(impulse);
        rigidA.AddAngularImpulse(MathHelper::Cross(m_rA, impulse));
    }
    if (rigidB.IsDynamic()) {
        rigidB.AddImpulse(-impulse);
        rigidB.AddAngularImpulse(-MathHelper::Cross(m_rB, impulse));
    }
}

void DistanceConstraint::SolveVelocity(SolverContext& context) {
    if (!m_isActive) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    const glm::vec2 velocityA = rigidA.VelocityAtPoint(m_rA);
    const glm::vec2 velocityB = rigidB.VelocityAtPoint(m_rB);
    const float relativeVelocity = glm::dot(
        velocityB - velocityA, m_normal);

    // This is a bilateral distance constraint, so its accumulated impulse is
    // not clamped. Apply only this iteration's delta to the velocities.
    const float impulseDelta = m_effectiveMass
        * (relativeVelocity + m_bias);
    m_accumulatedImpulse += impulseDelta;
    const glm::vec2 impulse = impulseDelta * m_normal;

    if (rigidA.IsDynamic()) {
        rigidA.AddImpulse(impulse);
        rigidA.AddAngularImpulse(MathHelper::Cross(m_rA, impulse));
    }
    if (rigidB.IsDynamic()) {
        rigidB.AddImpulse(-impulse);
        rigidB.AddAngularImpulse(-MathHelper::Cross(m_rB, impulse));
    }
}

void DistanceConstraint::SolvePosition(SolverContext& context) {
    if (!m_isActive) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    const glm::vec2 worldAnchorA = WorldPoint(
        rigidA.transform, m_localAnchorA);
    const glm::vec2 worldAnchorB = WorldPoint(
        rigidB.transform, m_localAnchorB);
    const glm::vec2 separation = worldAnchorB - worldAnchorA;
    const float length = glm::length(separation);

    glm::vec2 normal;
    if (length <= 1e-6f) normal = glm::vec2{1.0f, 0.0f};
    else normal = separation / length;

    const float error = length - m_targetLength;
    const float slop = std::max(context.settings.slop, 0.0f);
    float correctedError = 0.0f;
    if (error > slop) correctedError = error - slop;
    else if (error < -slop) correctedError = error + slop;
    if (correctedError == 0.0f) return;

    const glm::vec2 worldCentroidA = WorldPoint(
        rigidA.transform, bodyA->collider.centroid);
    const glm::vec2 worldCentroidB = WorldPoint(
        rigidB.transform, bodyB->collider.centroid);
    const glm::vec2 rA = worldAnchorA - worldCentroidA;
    const glm::vec2 rB = worldAnchorB - worldCentroidB;
    const float crossA = MathHelper::Cross(rA, normal);
    const float crossB = MathHelper::Cross(rB, normal);
    const float inverseMassA = EffectiveInverseMass(rigidA);
    const float inverseMassB = EffectiveInverseMass(rigidB);
    const float inverseInertiaA = EffectiveInverseInertia(rigidA);
    const float inverseInertiaB = EffectiveInverseInertia(rigidB);
    const float inverseMassSum = inverseMassA + inverseMassB
        + inverseInertiaA * crossA * crossA
        + inverseInertiaB * crossB * crossB;
    if (inverseMassSum <= 1e-6f) return;

    const float correctionFactor = std::max(context.settings.baumgarte, 0.0f);
    const glm::vec2 positionImpulse =
        (correctionFactor * correctedError / inverseMassSum) * normal;

    ApplyPositionImpulse(
        rigidA, bodyA->collider.centroid, positionImpulse, rA,
        inverseMassA, inverseInertiaA);
    ApplyPositionImpulse(
        rigidB, bodyB->collider.centroid, -positionImpulse, rB,
        inverseMassB, inverseInertiaB);
}
