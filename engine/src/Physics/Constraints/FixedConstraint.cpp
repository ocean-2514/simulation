#include "Physics/Constraints/FixedConstraint.hpp"

#include <algorithm>
#include <cmath>

namespace
{

constexpr float Pi = 3.14159265358979323846f;

bool IsFinite(const glm::vec2& value) {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

float WrapAngle(float angle) {
    return std::remainder(angle, 2.0f * Pi);
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

void ApplyAngularPositionImpulse(
    RigidBody& body,
    const glm::vec2& localCentroid,
    float angleDelta) {
    if (angleDelta == 0.0f) return;

    const glm::vec2 center = WorldPoint(body.transform, localCentroid);
    body.transform.rotation += angleDelta;
    body.transform.position = center
        - MathHelper::Rotate(localCentroid, body.transform.rotation);
}

void ResetImpulses(
    glm::vec2& linearImpulse,
    float& angularImpulse,
    float& previousDeltaTime) {
    linearImpulse = glm::vec2{0.0f};
    angularImpulse = 0.0f;
    previousDeltaTime = 0.0f;
}

} // namespace

void FixedConstraint::Prepare(SolverContext& context) {
    m_isActive = false;
    m_isLinearActive = false;
    m_isAngularActive = false;
    m_effectiveMass = glm::mat2{0.0f};
    m_effectiveAngularMass = 0.0f;
    m_bias = glm::vec2{0.0f};
    m_angularBias = 0.0f;

    if (!(context.deltaTime > 0.0f) || !std::isfinite(context.deltaTime)
        || m_bodyA == m_bodyB || !IsFinite(m_localAnchorA)
        || !IsFinite(m_localAnchorB) || !std::isfinite(m_referenceAngle)) {
        ResetImpulses(m_accumulatedImpulse,
            m_accumulatedAngularImpulse, m_previousDeltaTime);
        return;
    }

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) {
        ResetImpulses(m_accumulatedImpulse,
            m_accumulatedAngularImpulse, m_previousDeltaTime);
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
    if (!IsFinite(m_rA) || !IsFinite(m_rB)) {
        ResetImpulses(m_accumulatedImpulse,
            m_accumulatedAngularImpulse, m_previousDeltaTime);
        return;
    }

    const float inverseMassA = EffectiveInverseMass(rigidA);
    const float inverseMassB = EffectiveInverseMass(rigidB);
    const float inverseInertiaA = EffectiveInverseInertia(rigidA);
    const float inverseInertiaB = EffectiveInverseInertia(rigidB);
    m_isLinearActive = ComputeEffectiveMass(
        m_rA, m_rB, inverseMassA, inverseMassB,
        inverseInertiaA, inverseInertiaB, m_effectiveMass);
    if (!m_isLinearActive) {
        m_accumulatedImpulse = glm::vec2{0.0f};
    }

    const float inverseInertiaSum = inverseInertiaA + inverseInertiaB;
    if (inverseInertiaSum > 1e-12f && std::isfinite(inverseInertiaSum)) {
        m_effectiveAngularMass = 1.0f / inverseInertiaSum;
        m_isAngularActive = std::isfinite(m_effectiveAngularMass);
    } else {
        m_accumulatedAngularImpulse = 0.0f;
    }

    m_isActive = m_isLinearActive || m_isAngularActive;
    if (!m_isActive) {
        ResetImpulses(m_accumulatedImpulse,
            m_accumulatedAngularImpulse, m_previousDeltaTime);
        return;
    }

    const float slop = NonNegativeFinite(context.settings.slop);
    const float biasFactor = NonNegativeFinite(context.settings.baumgarte);
    const glm::vec2 separation = worldAnchorB - worldAnchorA;
    const float anchorError = glm::length(separation);
    if (m_isLinearActive && std::isfinite(anchorError) && anchorError > slop) {
        m_bias = (biasFactor / context.deltaTime)
            * ((anchorError - slop) / anchorError) * separation;
    }

    const float relativeAngle = rigidB.transform.rotation
        - rigidA.transform.rotation - m_referenceAngle;
    const float angleError = WrapAngle(relativeAngle);
    if (m_isAngularActive && std::isfinite(angleError)) {
        m_angularBias = (biasFactor / context.deltaTime) * angleError;
    }

    if (m_previousDeltaTime > 0.0f
        && std::isfinite(m_previousDeltaTime)) {
        const float timestepRatio = context.deltaTime / m_previousDeltaTime;
        m_accumulatedImpulse *= timestepRatio;
        m_accumulatedAngularImpulse *= timestepRatio;
    } else {
        m_accumulatedImpulse = glm::vec2{0.0f};
        m_accumulatedAngularImpulse = 0.0f;
    }
    m_previousDeltaTime = context.deltaTime;
}

void FixedConstraint::WarmStart(SolverContext& context) {
    if (!m_isActive) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    if (m_isLinearActive) {
        if (rigidA.IsDynamic()) {
            rigidA.AddImpulse(m_accumulatedImpulse);
            rigidA.AddAngularImpulse(Cross(m_rA, m_accumulatedImpulse));
        }
        if (rigidB.IsDynamic()) {
            rigidB.AddImpulse(-m_accumulatedImpulse);
            rigidB.AddAngularImpulse(-Cross(m_rB, m_accumulatedImpulse));
        }
    }

    if (m_isAngularActive) {
        if (rigidA.IsDynamic()) {
            rigidA.AddAngularImpulse(m_accumulatedAngularImpulse);
        }
        if (rigidB.IsDynamic()) {
            rigidB.AddAngularImpulse(-m_accumulatedAngularImpulse);
        }
    }
}

void FixedConstraint::SolveVelocity(SolverContext& context) {
    if (!m_isActive) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    if (m_isLinearActive) {
        const glm::vec2 velocityA = rigidA.VelocityAtPoint(m_rA);
        const glm::vec2 velocityB = rigidB.VelocityAtPoint(m_rB);
        const glm::vec2 relativeVelocity = velocityB - velocityA;
        const glm::vec2 impulseDelta =
            m_effectiveMass * (relativeVelocity + m_bias);
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

    if (m_isAngularActive) {
        const float relativeAngularVelocity =
            rigidB.angularVelocity - rigidA.angularVelocity;
        const float angularImpulseDelta = m_effectiveAngularMass
            * (relativeAngularVelocity + m_angularBias);
        m_accumulatedAngularImpulse += angularImpulseDelta;

        if (rigidA.IsDynamic()) {
            rigidA.AddAngularImpulse(angularImpulseDelta);
        }
        if (rigidB.IsDynamic()) {
            rigidB.AddAngularImpulse(-angularImpulseDelta);
        }
    }
}

void FixedConstraint::SolvePosition(SolverContext& context) {
    if (!m_isActive) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;
    const float correctionFactor = NonNegativeFinite(context.settings.baumgarte);
    const float slop = NonNegativeFinite(context.settings.slop);
    if (correctionFactor <= 0.0f) return;

    const float inverseMassA = EffectiveInverseMass(rigidA);
    const float inverseMassB = EffectiveInverseMass(rigidB);
    const float inverseInertiaA = EffectiveInverseInertia(rigidA);
    const float inverseInertiaB = EffectiveInverseInertia(rigidB);

    if (m_isLinearActive) {
        const glm::vec2 worldAnchorA = WorldPoint(
            rigidA.transform, m_localAnchorA);
        const glm::vec2 worldAnchorB = WorldPoint(
            rigidB.transform, m_localAnchorB);
        const glm::vec2 separation = worldAnchorB - worldAnchorA;
        const float anchorError = glm::length(separation);
        if (anchorError > slop && std::isfinite(anchorError)) {
            const glm::vec2 centroidA = WorldPoint(
                rigidA.transform, bodyA->collider.centroid);
            const glm::vec2 centroidB = WorldPoint(
                rigidB.transform, bodyB->collider.centroid);
            const glm::vec2 rA = worldAnchorA - centroidA;
            const glm::vec2 rB = worldAnchorB - centroidB;
            glm::mat2 effectiveMass{0.0f};
            if (ComputeEffectiveMass(rA, rB, inverseMassA, inverseMassB,
                inverseInertiaA, inverseInertiaB, effectiveMass)) {
                const glm::vec2 correctedError =
                    ((anchorError - slop) / anchorError) * separation;
                const glm::vec2 positionImpulse =
                    correctionFactor * (effectiveMass * correctedError);
                ApplyPositionImpulse(rigidA, bodyA->collider.centroid,
                    positionImpulse, rA, inverseMassA, inverseInertiaA);
                ApplyPositionImpulse(rigidB, bodyB->collider.centroid,
                    -positionImpulse, rB, inverseMassB, inverseInertiaB);
            }
        }
    }

    if (m_isAngularActive) {
        const float angleError = WrapAngle(
            rigidB.transform.rotation - rigidA.transform.rotation
            - m_referenceAngle);
        const float inverseInertiaSum = inverseInertiaA + inverseInertiaB;
        if (std::isfinite(angleError)
            && inverseInertiaSum > 1e-12f
            && std::isfinite(inverseInertiaSum)) {
            const float angularImpulse = correctionFactor
                * angleError / inverseInertiaSum;
            ApplyAngularPositionImpulse(rigidA, bodyA->collider.centroid,
                inverseInertiaA * angularImpulse);
            ApplyAngularPositionImpulse(rigidB, bodyB->collider.centroid,
                -inverseInertiaB * angularImpulse);
        }
    }
}
