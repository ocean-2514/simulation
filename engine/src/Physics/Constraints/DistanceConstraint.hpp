#pragma once

#include "Physics/Constraints/Constraint.hpp"
#include "Physics/PhysicsWorld.hpp"

struct DistanceConstraintDesc {
    BodyId bodyA = InvalidIndex;
    BodyId bodyB = InvalidIndex;
    glm::vec2 localAnchorA{0.f};
    glm::vec2 localAnchorB{0.f};
    float targetLength = 1.0f;
};

class DistanceConstraint : public Constraint
{
public:
    DistanceConstraint(const DistanceConstraintDesc& desc)
        : m_bodyA(desc.bodyA), m_bodyB(desc.bodyB),
        m_localAnchorA(desc.localAnchorA), m_localAnchorB(desc.localAnchorB),
        m_targetLength(desc.targetLength) {}

    void Prepare(SolverContext& context) override;
    void WarmStart(SolverContext& context) override;
    void SolveVelocity(SolverContext& context) override;
    void SolvePosition(SolverContext& context) override;

private:
    BodyId m_bodyA = InvalidIndex;
    BodyId m_bodyB = InvalidIndex;
    glm::vec2 m_localAnchorA{0.f};
    glm::vec2 m_localAnchorB{0.f};
    float m_targetLength = 1.0f;

    glm::vec2 m_rA;
    glm::vec2 m_rB;
    glm::vec2 m_normal{1.0f, 0.0f};
    float m_effectiveMass = 0.0f;
    float m_bias = 0.0f;

    float m_accumulatedImpulse = 0.f;
    float m_previousDeltaTime = 0.0f;
    bool m_isActive = false;
};
