#pragma once

#include "Physics/Forces/ForceGenetator.hpp"
#include "Physics/PhysicsWorld.hpp"

#include <cmath>

struct SpringForceDesc {
    BodyId bodyA = InvalidIndex;
    BodyId bodyB = InvalidIndex;
    glm::vec2 localAnchorA{0.f};
    glm::vec2 localAnchorB{0.f};
    float restLength = 1.0f;
    float stiffness = 20.0f;
    float damping = 1.0f;
};

class SpringForceGenerator : public ForceGenerator {
public:
    SpringForceGenerator(const SpringForceDesc& desc)
        : m_bodyA(desc.bodyA), m_bodyB(desc.bodyB),
        m_localAnchorA(desc.localAnchorA), m_localAnchorB(desc.localAnchorB),
        m_restLength(desc.restLength), m_stiffness(desc.stiffness),
        m_damping(desc.damping) {}

    void Apply(ForceContext& context, float) override {
        if (m_bodyA == m_bodyB
            || !std::isfinite(m_restLength) || m_restLength < 0.0f
            || !std::isfinite(m_stiffness) || m_stiffness < 0.0f
            || !std::isfinite(m_damping) || m_damping < 0.0f) {
            return;
        }

        auto& world = context.world;

        auto* bodyA = world.GetBody(m_bodyA);
        auto* bodyB = world.GetBody(m_bodyB);
        if (!bodyA || !bodyB) return;

        auto& rigiA = bodyA->rigidBody;
        auto& rigiB = bodyB->rigidBody;
        glm::vec2 worldAnchorA = WorldPoint(rigiA.transform,
            m_localAnchorA);
        glm::vec2 worldAnchorB = WorldPoint(rigiB.transform,
            m_localAnchorB);
        glm::vec2 worldCentroidA = WorldPoint(rigiA.transform,
            bodyA->collider.centroid);
        glm::vec2 worldCentroidB = WorldPoint(rigiB.transform,
            bodyB->collider.centroid);
        glm::vec2 rA = worldAnchorA - worldCentroidA;
        glm::vec2 rB = worldAnchorB - worldCentroidB;
        glm::vec2 dist = worldAnchorB - worldAnchorA;
        float length = glm::length(dist);
        if (!std::isfinite(length) || length <= 1e-6f) return;
        glm::vec2 distNorm = dist / length;

        glm::vec2 velAnchorA = rigiA.VelocityAtPoint(rA);
        glm::vec2 velAnchorB = rigiB.VelocityAtPoint(rB);
        glm::vec2 relativeVel = velAnchorB - velAnchorA;
        float velAlongNorm = glm::dot(relativeVel, distNorm);

        float springForceMag = m_stiffness * (length - m_restLength);
        float dampingForceMag = m_damping * velAlongNorm;
        glm::vec2 force = (springForceMag + dampingForceMag) * distNorm;

        rigiA.ApplyForce(+force);
        rigiB.ApplyForce(-force);
        rigiA.ApplyTorque(MathHelper::Cross(rA, +force));
        rigiB.ApplyTorque(MathHelper::Cross(rB, -force));
    }
    
private:
    BodyId m_bodyA = InvalidIndex;
    BodyId m_bodyB = InvalidIndex;
    glm::vec2 m_localAnchorA{0.f};
    glm::vec2 m_localAnchorB{0.f};
    float m_restLength = 1.0f;
    float m_stiffness = 20.0f;
    float m_damping = 1.0f;
};
