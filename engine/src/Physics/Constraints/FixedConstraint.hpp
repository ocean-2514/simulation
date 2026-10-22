#pragma once

#include "Physics/Constraints/Constraint.hpp"
#include "Physics/PhysicsWorld.hpp"

struct FixedConstraintDesc {
    BodyId bodyA;
    BodyId bodyB;
    glm::vec2 localAnchorA{0.f};
    glm::vec2 localAnchorB{0.f};
    float referenceAngle = 0.f;
};

class FixedConstraint : Constraint {
public:
    FixedConstraint(const FixedConstraintDesc& desc)
        : m_bodyA(desc.bodyA), m_bodyB(desc.bodyB),
        m_localAnchorA(desc.localAnchorA), 
        m_localAnchorB(desc.localAnchorB) {}

    void Prepare(SolverContext& context) override;
    void WarmStart(SolverContext& context) override;
    void SolveVelocity(SolverContext& context) override;
    void SolvePosition(SolverContext& context) override;

private:
    BodyId m_bodyA;
    BodyId m_bodyB;
    glm::vec2 m_localAnchorA{0.f};
    glm::vec2 m_localAnchorB{0.f};

};