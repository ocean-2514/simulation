#include "Physics/Collision/ContactConstraint.hpp"

#include "Physics/PhysicsWorld.hpp"
#include "Physics/Collision/ContactCache.hpp"

#include <algorithm>
#include <cmath>

namespace
{

bool IsFinite(const glm::vec2& value) {
    return std::isfinite(value.x) && std::isfinite(value.y);
}

} // namespace

void ContactConstraint::Prepare(SolverContext& context) {
    m_active = false;
    m_pointStates = {};
    m_normal = glm::vec2{0.0f};
    m_tangent = glm::vec2{0.0f};
    m_localReferenceNormal = glm::vec2{0.0f};
    m_hasReferenceNormal = false;
    m_friction = 0.0f;
    m_inverseMassSum = 0.0f;

    if (!m_manifold.colliding || m_bodyA == m_bodyB
        || m_manifold.pointCount == 0
        || m_manifold.pointCount > MaxManifoldPoints) {
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
    m_normal = m_manifold.normal * glm::inversesqrt(normalLengthSquared);
    m_tangent = {-m_normal.y, m_normal.x};

    const bool referenceIsA = m_manifold.referenceIsA != 0;
    const PhysicsBody& referenceBody = referenceIsA ? *bodyA : *bodyB;
    if (referenceBody.collider.Type() == ColliderType::Polygon) {
        const auto& polygon = std::get<PolygonCollider>(
            referenceBody.collider.data);
        const std::size_t referenceFace =
            m_manifold.points[0].feature.referenceFace;
        if (referenceFace < polygon.normals.size()) {
            const glm::vec2 localNormal = polygon.normals[referenceFace];
            const float localNormalLengthSquared = glm::dot(
                localNormal, localNormal);
            if (localNormalLengthSquared > 1e-12f
                && std::isfinite(localNormalLengthSquared)) {
                m_localReferenceNormal = localNormal
                    * glm::inversesqrt(localNormalLengthSquared);
                m_hasReferenceNormal = true;
            }
        }
    }

    m_friction = NonNegativeFinite(std::sqrt(
        NonNegativeFinite(rigidA.friction)
        * NonNegativeFinite(rigidB.friction)));

    const glm::vec2 centroidA = WorldPoint(
        rigidA.transform, bodyA->collider.centroid);
    const glm::vec2 centroidB = WorldPoint(
        rigidB.transform, bodyB->collider.centroid);

    m_inverseMassA = EffectiveInverseMass(rigidA);
    m_inverseMassB = EffectiveInverseMass(rigidB);
    m_inverseInertiaA = EffectiveInverseInertia(rigidA);
    m_inverseInertiaB = EffectiveInverseInertia(rigidB);
    m_inverseMassSum = m_inverseMassA + m_inverseMassB;

    for (uint8_t i = 0; i < m_manifold.pointCount; ++i) {
        const auto& point = m_manifold.points[i];
        auto& state = m_pointStates[i];

        state.rA = point.position - centroidA;
        state.rB = point.position - centroidB;

        const float rACrossNormal = Cross(state.rA, m_normal);
        const float rBCrossNormal = Cross(state.rB, m_normal);
        const float normalDenominator = m_inverseMassSum
            + rACrossNormal * rACrossNormal * m_inverseInertiaA
            + rBCrossNormal * rBCrossNormal * m_inverseInertiaB;
        if (!(normalDenominator > 1e-12f)
            || !std::isfinite(normalDenominator)) {
            state = {};
            continue;
        }
        state.normalEffectiveMass = 1.0f / normalDenominator;

        const float rACrossTangent = Cross(state.rA, m_tangent);
        const float rBCrossTangent = Cross(state.rB, m_tangent);
        const float tangentDenominator = m_inverseMassSum
            + rACrossTangent * rACrossTangent * m_inverseInertiaA
            + rBCrossTangent * rBCrossTangent * m_inverseInertiaB;
        if (tangentDenominator > 1e-12f && std::isfinite(tangentDenominator)) {
            state.tangentEffectiveMass = 1.0f / tangentDenominator;
        }

        const glm::vec2 relativeVelocity = rigidB.VelocityAtPoint(state.rB)
            - rigidA.VelocityAtPoint(state.rA);
        const float velocityAlongNormal = glm::dot(relativeVelocity, m_normal);
        const float restitutionA = NonNegativeFinite(rigidA.restitution);
        const float restitutionB = NonNegativeFinite(rigidB.restitution);
        constexpr float restitutionThreshold = 1.0f;
        if (velocityAlongNormal < -restitutionThreshold) {
            state.restitutionTarget = std::min(restitutionA, restitutionB)
                * -velocityAlongNormal;
        }

        if (context.contactCache) {
            context.contactCache->Restore(
                {m_bodyA, m_bodyB, point.feature},
                m_normal, context.deltaTime,
                state.accumulatedNormalImpulse,
                state.accumulatedTangentImpulse);
        }
        state.accumulatedNormalImpulse = std::isfinite(
            state.accumulatedNormalImpulse)
            ? std::max(state.accumulatedNormalImpulse, 0.0f)
            : 0.0f;
        const float maxFrictionImpulse = m_friction
            * state.accumulatedNormalImpulse;
        state.accumulatedTangentImpulse = state.tangentEffectiveMass > 0.0f
            && std::isfinite(state.accumulatedTangentImpulse)
            ? glm::clamp(state.accumulatedTangentImpulse,
                -maxFrictionImpulse, maxFrictionImpulse)
            : 0.0f;
        state.active = true;
        m_active = true;
    }
}

void ContactConstraint::WarmStart(SolverContext& context) {
    if (!m_active) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    for (uint8_t i = 0; i < m_manifold.pointCount; ++i) {
        ContactPointState& state = m_pointStates[i];
        if (!state.active) continue;

        const glm::vec2 impulse =
            state.accumulatedNormalImpulse * m_normal
            + state.accumulatedTangentImpulse * m_tangent;
        if (state.accumulatedNormalImpulse == 0.0f
            && state.accumulatedTangentImpulse == 0.0f) {
            continue;
        }

        ApplyVelocityImpulse(bodyA->rigidBody, bodyB->rigidBody,
            state.rA, state.rB, impulse);
    }
}

void ContactConstraint::SolveVelocity(SolverContext& context) {
    if (!m_active) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;

    for (uint8_t i = 0; i < m_manifold.pointCount; ++i) {
        const auto& point = m_manifold.points[i];
        auto& state = m_pointStates[i];
        if (!state.active || state.normalEffectiveMass <= 0.0f) {
            continue;
        }

        glm::vec2 relativeVelocity = rigidB.VelocityAtPoint(state.rB)
            - rigidA.VelocityAtPoint(state.rA);
        const float velocityAlongNormal = glm::dot(relativeVelocity, m_normal);
        const float oldNormalImpulse = state.accumulatedNormalImpulse;
        state.accumulatedNormalImpulse = std::max(
            oldNormalImpulse + state.normalEffectiveMass
                * (state.restitutionTarget - velocityAlongNormal),
            0.0f);
        const glm::vec2 normalImpulse =
            (state.accumulatedNormalImpulse - oldNormalImpulse) * m_normal;
        ApplyVelocityImpulse(rigidA, rigidB, state.rA, state.rB, normalImpulse);

        if (state.tangentEffectiveMass <= 0.0f) {
            state.accumulatedTangentImpulse = 0.0f;
        } else {
            relativeVelocity = rigidB.VelocityAtPoint(state.rB)
                - rigidA.VelocityAtPoint(state.rA);
            const float velocityAlongTangent = glm::dot(
                relativeVelocity, m_tangent);
            const float oldTangentImpulse = state.accumulatedTangentImpulse;
            const float maxFrictionImpulse =
                m_friction * state.accumulatedNormalImpulse;
            state.accumulatedTangentImpulse = glm::clamp(
                oldTangentImpulse - state.tangentEffectiveMass
                    * velocityAlongTangent,
                -maxFrictionImpulse, maxFrictionImpulse);
            const glm::vec2 tangentImpulse =
                (state.accumulatedTangentImpulse - oldTangentImpulse)
                * m_tangent;
            ApplyVelocityImpulse(
                rigidA, rigidB, state.rA, state.rB, tangentImpulse);
        }

        if (context.contactCache) {
            context.contactCache->Store(
                {m_bodyA, m_bodyB, point.feature},
                m_normal,
                state.accumulatedNormalImpulse,
                state.accumulatedTangentImpulse,
                context.deltaTime);
        }
    }
}

void ContactConstraint::SolvePosition(SolverContext& context) {
    if (!m_active) return;

    const float slop = NonNegativeFinite(context.settings.slop);
    const float correctionFactor = NonNegativeFinite(context.settings.baumgarte);
    if (correctionFactor <= 0.0f) return;

    PhysicsBody* bodyA = context.world.GetBody(m_bodyA);
    PhysicsBody* bodyB = context.world.GetBody(m_bodyB);
    if (!bodyA || !bodyB) return;

    RigidBody& rigidA = bodyA->rigidBody;
    RigidBody& rigidB = bodyB->rigidBody;

    for (uint8_t i = 0; i < m_manifold.pointCount; ++i) {
        const auto& point = m_manifold.points[i];
        auto& state = m_pointStates[i];
        if (!state.active) continue;

        // Recompute the reference normal after each positional correction.
        glm::vec2 positionNormal = m_normal;
        if (m_hasReferenceNormal) {
            const RigidBody& referenceBody = m_manifold.referenceIsA != 0
                ? rigidA : rigidB;
            const glm::vec2 rotatedNormal = MathHelper::Rotate(
                m_localReferenceNormal, referenceBody.transform.rotation);
            const float rotatedLengthSquared = glm::dot(
                rotatedNormal, rotatedNormal);
            if (rotatedLengthSquared > 1e-12f
                && std::isfinite(rotatedLengthSquared)) {
                positionNormal = rotatedNormal
                    * glm::inversesqrt(rotatedLengthSquared);
                if (m_manifold.referenceIsA == 0) {
                    positionNormal = -positionNormal;
                }
            }
        }
        const glm::vec2 pA = WorldPoint(rigidA.transform, point.localPointA);
        const glm::vec2 pB = WorldPoint(rigidB.transform, point.localPointB);
        if (!IsFinite(pA) || !IsFinite(pB)) continue;
        const float separation = glm::dot(pB - pA, positionNormal);
        if (!std::isfinite(separation)) continue;
        const float penetration = std::max(-separation - slop, 0.0f);
        if (penetration <= 0.0f) continue;

        const glm::vec2 contactPoint = 0.5f * (pA + pB);
        const glm::vec2 centroidA = WorldPoint(
            rigidA.transform, bodyA->collider.centroid);
        const glm::vec2 centroidB = WorldPoint(
            rigidB.transform, bodyB->collider.centroid);
        const glm::vec2 rA = contactPoint - centroidA;
        const glm::vec2 rB = contactPoint - centroidB;

        const float rACrossNormal = Cross(rA, positionNormal);
        const float rBCrossNormal = Cross(rB, positionNormal);
        const float inverseMassSum = m_inverseMassSum
            + rACrossNormal * rACrossNormal * m_inverseInertiaA
            + rBCrossNormal * rBCrossNormal * m_inverseInertiaB;
        if (!(inverseMassSum > 1e-12f) || !std::isfinite(inverseMassSum)) continue;

        const glm::vec2 positionImpulse =
            (correctionFactor * penetration / inverseMassSum) * positionNormal;
        ApplyPositionImpulse(rigidA, bodyA->collider.centroid,
            -positionImpulse, rA, m_inverseMassA, m_inverseInertiaA);
        ApplyPositionImpulse(rigidB, bodyB->collider.centroid,
            positionImpulse, rB, m_inverseMassB, m_inverseInertiaB);
    }
}
