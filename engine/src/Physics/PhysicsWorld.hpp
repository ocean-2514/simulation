#pragma once

#include "Physics/RigidBody.hpp"
#include "Physics/Collider.hpp"
#include "Physics/BroadPhase/AABB.hpp"
#include "Physics/BroadPhase/Spatial/BVHTree2D.hpp"
#include "Utils/MathHelper.hpp"

#include <vector>

struct PhysicsBody {
    RigidBody rigidBody;
    Collider collider;
    AABB2 bounds{};
};

class PhysicsWorld
{
public:
    uint32_t CreateBody(const RigidBody& body, 
        const Collider& collider, 
        const glm::vec2& pos = glm::vec2{0.f});

    PhysicsBody* GetBody(uint32_t id) {
        if (id == InvalidIndex || id >= static_cast<uint32_t>(m_bodies.size())) return nullptr;
        return &m_bodies[id];
    }

    const PhysicsBody* GetBody(uint32_t id) const {
        if (id == InvalidIndex || id >= static_cast<uint32_t>(m_bodies.size())) return nullptr;
        return &m_bodies[id];
    }

    void CalculateInertia(uint32_t i) {
        if (auto* body = GetBody(i)) {
            body->rigidBody.SetInertia(MathHelper::CalculateInertia(
                body->collider, body->rigidBody.GetMass()));
        }
    }

    void PopulateInertia() {
        for (auto& [body, collider, _] : m_bodies) {
            body.SetInertia(MathHelper::CalculateInertia(collider, body.GetMass()));
        }
    }

    void Step(float deltaTime, int iteration = 10);

    glm::vec2 gravity{0.0f, -1.0f};

private:
    static AABB2 ComputeBounds(const PhysicsBody& body);
    void StepOneIteration(float deltaTime);

    std::vector<PhysicsBody> m_bodies;
    std::vector<BVHTree2D::ProxyId> m_proxyIds;
    BVHTree2D m_broadPhase;
};
