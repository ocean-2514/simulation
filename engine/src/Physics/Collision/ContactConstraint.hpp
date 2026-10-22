#pragma once

#include "Physics/Collision/CollisionManifold.hpp"
#include "Physics/Constraints/Constraint.hpp"
#include "Physics/PhysicsTypes.hpp"

struct ContactImpulseState {
    glm::vec2 normal{0.0f};
    glm::vec2 tangent{0.0f};
    glm::vec2 rA{0.0f};
    glm::vec2 rB{0.0f};

    float inverseMassA = 0.0f;
    float inverseMassB = 0.0f;
    float inverseInertiaA = 0.0f;
    float inverseInertiaB = 0.0f;
    float inverseMassSum = 0.0f;
    float normalEffectiveMass = 0.0f;
    float tangentEffectiveMass = 0.0f;
    float friction = 0.0f;
    float restitutionTarget = 0.0f;
    float accumulatedNormalImpulse = 0.0f;
    float accumulatedTangentImpulse = 0.0f;
    bool active = false;
};

// A contact is a constraint created by collision detection for one substep.
// Its velocity-solver geometry and effective masses are prepared once, then
// reused across all velocity iterations.
class ContactConstraint final : public Constraint {
public:
    ContactConstraint(
        BodyId bodyA,
        BodyId bodyB,
        const CollisionManifold& manifold)
        : m_bodyA(bodyA), m_bodyB(bodyB), m_manifold(manifold) {}

    void Prepare(SolverContext& context) override;
    void WarmStart(SolverContext& context) override;
    void SolveVelocity(SolverContext& context) override;
    void SolvePosition(SolverContext& context) override;

private:
    BodyId m_bodyA = InvalidIndex;
    BodyId m_bodyB = InvalidIndex;
    CollisionManifold m_manifold{};
    ContactImpulseState m_impulseState{};
};
