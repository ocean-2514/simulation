#pragma once

#include "Physics/Constraints/Constraint.hpp"
#include "Physics/PhysicsWorld.hpp"

struct FixedConstraintDesc {
    BodyId bodyA = InvalidIndex;
    BodyId bodyB = InvalidIndex;
    glm::vec2 localAnchorA{0.f};
    glm::vec2 localAnchorB{0.f};
    float referenceAngle = 0.f;
};

class FixedConstraint : public Constraint {
public:
    FixedConstraint(const FixedConstraintDesc& desc)
        : m_bodyA(desc.bodyA), m_bodyB(desc.bodyB),
        m_localAnchorA(desc.localAnchorA),
        m_localAnchorB(desc.localAnchorB),
        m_referenceAngle{desc.referenceAngle} {}

    void Prepare(SolverContext& context) override;
    void WarmStart(SolverContext& context) override;
    void SolveVelocity(SolverContext& context) override;
    void SolvePosition(SolverContext& context) override;

private:
    BodyId m_bodyA;
    BodyId m_bodyB;
    glm::vec2 m_localAnchorA{0.f};
    glm::vec2 m_localAnchorB{0.f};
    float m_referenceAngle = 0.f;

    glm::vec2 m_rA;
    glm::vec2 m_rB;
    glm::mat2 m_effectiveMass{0.0f};
    glm::vec2 m_bias{0.f};
    float m_effectiveAngularMass = 0.0f;
    float m_angularBias = 0.0f;

    glm::vec2 m_accumulatedImpulse{0.0f};
    float m_accumulatedAngularImpulse = 0.0f;
    float m_previousDeltaTime = 0.0f;
    bool m_isActive = false;
    bool m_isLinearActive = false;
    bool m_isAngularActive = false;
};
