#pragma once

#include "Physics/Constraints/Constraint.hpp"
#include "Physics/PhysicsWorld.hpp"

struct HingeConstraintDesc {
    BodyId bodyA = InvalidIndex;
    BodyId bodyB = InvalidIndex;
    glm::vec2 localAnchorA{0.0f};
    glm::vec2 localAnchorB{0.0f};
};

// A 2D revolute joint: the two anchors stay together while relative rotation
// around the out-of-plane axis remains free.
class HingeConstraint : public Constraint {
public:
    explicit HingeConstraint(const HingeConstraintDesc& desc)
        : m_bodyA(desc.bodyA),
          m_bodyB(desc.bodyB),
          m_localAnchorA(desc.localAnchorA),
          m_localAnchorB(desc.localAnchorB) {}

    void Prepare(SolverContext& context) override;
    void WarmStart(SolverContext& context) override;
    void SolveVelocity(SolverContext& context) override;
    void SolvePosition(SolverContext& context) override;

private:
    BodyId m_bodyA = InvalidIndex;
    BodyId m_bodyB = InvalidIndex;
    glm::vec2 m_localAnchorA{0.0f};
    glm::vec2 m_localAnchorB{0.0f};

    glm::vec2 m_rA{0.0f};
    glm::vec2 m_rB{0.0f};
    glm::mat2 m_effectiveMass{0.0f};
    glm::vec2 m_bias{0.0f};
    glm::vec2 m_accumulatedImpulse{0.0f};
    float m_previousDeltaTime = 0.0f;
    bool m_isActive = false;
};
