#include "Physics/Constraints/HingeConstraint.hpp"

#include <cmath>

namespace
{

bool IsFinite(const glm::vec2& value) {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

bool ComputeEffectiveMass(
    const glm::vec2& rA,
    const glm::vec2& rB,
    float inverseMassA,
    float inverseMassB,
    float inverseInertiaA,
    float inverseInertiaB,
    glm::mat2& effectiveMass) {
    const float inverseMassSum = inverseMassA + inverseMassB;
    const float kXX = inverseMassSum
        + rA.y * rA.y * inverseInertiaA
        + rB.y * rB.y * inverseInertiaB;
    const float kXY = -rA.x * rA.y * inverseInertiaA
        - rB.x * rB.y * inverseInertiaB;
    const float kYY = inverseMassSum
        + rA.x * rA.x * inverseInertiaA
        + rB.x * rB.x * inverseInertiaB;
    const float determinant = kXX * kYY - kXY * kXY;
    if (!(determinant > 1e-12f) || !std::isfinite(determinant)) {
        return false;
    }

    effectiveMass[0][0] = kYY / determinant;
    effectiveMass[1][1] = kXX / determinant;
    effectiveMass[1][0] = -kXY / determinant;
    effectiveMass[0][1] = -kXY / determinant;
    return std::isfinite(effectiveMass[0][0])
        && std::isfinite(effectiveMass[1][1])
        && std::isfinite(effectiveMass[1][0])
        && std::isfinite(effectiveMass[0][1]);
}

void ResetState(
    glm::vec2& accumulatedImpulse,
    float& previousDeltaTime) {
    accumulatedImpulse = glm::vec2{0.0f};
    previousDeltaTime = 0.0f;
}

} // namespace

void HingeConstraint::Prepare(SolverContext& context) {
    m_isActive = false;
    m_effectiveMass = glm::mat2{0.0f};
    m_bias = glm::vec2{0.0f};

    if (!(context.deltaTime > 0.0f) || !std::isfinite(context.deltaTime)
        || m_bodyA == m_bodyB || !IsFinite(m_localAnchorA)
        || !IsFinite(m_localAnchorB)) {
        ResetState(m_accumulatedImpulse, m_previousDeltaTime);
        return;
    }

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) {
        ResetState(m_accumulatedImpulse, m_previousDeltaTime);
        return;
    }

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    const glm::vec2 worldAnchorA = WorldPoint(
        rigidA.transform, m_localAnchorA);
    const glm::vec2 worldAnchorB = WorldPoint(
        rigidB.transform, m_localAnchorB);
    const glm::vec2 centroidA = WorldPoint(
        rigidA.transform, bodyA->collider.centroid);
    const glm::vec2 centroidB = WorldPoint(
        rigidB.transform, bodyB->collider.centroid);
    m_rA = worldAnchorA - centroidA;
    m_rB = worldAnchorB - centroidB;
    if (!IsFinite(m_rA) || !IsFinite(m_rB)) {
        ResetState(m_accumulatedImpulse, m_previousDeltaTime);
        return;
    }

    m_isActive = ComputeEffectiveMass(
        m_rA, m_rB,
        EffectiveInverseMass(rigidA), EffectiveInverseMass(rigidB),
        EffectiveInverseInertia(rigidA), EffectiveInverseInertia(rigidB),
        m_effectiveMass);
    if (!m_isActive) {
        ResetState(m_accumulatedImpulse, m_previousDeltaTime);
        return;
    }

    const glm::vec2 separation = worldAnchorB - worldAnchorA;
    const float anchorError = glm::length(separation);
    const float slop = NonNegativeFinite(context.settings.slop);
    const float correctionFactor =
        NonNegativeFinite(context.settings.baumgarte);
    if (std::isfinite(anchorError) && anchorError > slop) {
        m_bias = (correctionFactor / context.deltaTime)
            * ((anchorError - slop) / anchorError) * separation;
    }

    if (m_previousDeltaTime > 0.0f
        && std::isfinite(m_previousDeltaTime)) {
        m_accumulatedImpulse *= context.deltaTime / m_previousDeltaTime;
    } else {
        m_accumulatedImpulse = glm::vec2{0.0f};
    }
    m_previousDeltaTime = context.deltaTime;
}

void HingeConstraint::WarmStart(SolverContext& context) {
    if (!m_isActive
        || (m_accumulatedImpulse.x == 0.0f
            && m_accumulatedImpulse.y == 0.0f)) {
        return;
    }

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    if (rigidA.IsDynamic()) {
        rigidA.AddImpulse(m_accumulatedImpulse);
        rigidA.AddAngularImpulse(Cross(m_rA, m_accumulatedImpulse));
    }
    if (rigidB.IsDynamic()) {
        rigidB.AddImpulse(-m_accumulatedImpulse);
        rigidB.AddAngularImpulse(-Cross(m_rB, m_accumulatedImpulse));
    }
}

void HingeConstraint::SolveVelocity(SolverContext& context) {
    if (!m_isActive) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    const glm::vec2 velocityA = rigidA.VelocityAtPoint(m_rA);
    const glm::vec2 velocityB = rigidB.VelocityAtPoint(m_rB);
    const glm::vec2 impulseDelta =
        m_effectiveMass * (velocityB - velocityA + m_bias);
    m_accumulatedImpulse += impulseDelta;

    if (rigidA.IsDynamic()) {
        rigidA.AddImpulse(impulseDelta);
        rigidA.AddAngularImpulse(Cross(m_rA, impulseDelta));
    }
    if (rigidB.IsDynamic()) {
        rigidB.AddImpulse(-impulseDelta);
        rigidB.AddAngularImpulse(-Cross(m_rB, impulseDelta));
    }
}

void HingeConstraint::SolvePosition(SolverContext& context) {
    if (!m_isActive) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    const float slop = NonNegativeFinite(context.settings.slop);
    const float correctionFactor =
        NonNegativeFinite(context.settings.baumgarte);
    if (correctionFactor <= 0.0f) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    const glm::vec2 worldAnchorA = WorldPoint(
        rigidA.transform, m_localAnchorA);
    const glm::vec2 worldAnchorB = WorldPoint(
        rigidB.transform, m_localAnchorB);
    const glm::vec2 separation = worldAnchorB - worldAnchorA;
    const float anchorError = glm::length(separation);
    if (!std::isfinite(anchorError) || anchorError <= slop) return;

    const glm::vec2 centroidA = WorldPoint(
        rigidA.transform, bodyA->collider.centroid);
    const glm::vec2 centroidB = WorldPoint(
        rigidB.transform, bodyB->collider.centroid);
    const glm::vec2 rA = worldAnchorA - centroidA;
    const glm::vec2 rB = worldAnchorB - centroidB;
    const float inverseMassA = EffectiveInverseMass(rigidA);
    const float inverseMassB = EffectiveInverseMass(rigidB);
    const float inverseInertiaA = EffectiveInverseInertia(rigidA);
    const float inverseInertiaB = EffectiveInverseInertia(rigidB);

    glm::mat2 effectiveMass{0.0f};
    if (!ComputeEffectiveMass(rA, rB, inverseMassA, inverseMassB,
        inverseInertiaA, inverseInertiaB, effectiveMass)) {
        return;
    }

    const glm::vec2 correctedError =
        ((anchorError - slop) / anchorError) * separation;
    const glm::vec2 positionImpulse =
        correctionFactor * (effectiveMass * correctedError);
    ApplyPositionImpulse(rigidA, bodyA->collider.centroid,
        positionImpulse, rA, inverseMassA, inverseInertiaA);
    ApplyPositionImpulse(rigidB, bodyB->collider.centroid,
        -positionImpulse, rB, inverseMassB, inverseInertiaB);
}
