#pragma once

#include "Physics/RigidBody.hpp"
#include "Physics/Collider.hpp"
#include "BroadPhase/AABB.hpp"
#include <vector>

struct PhysicsBody {
    RigidBody rigidBody;
    Collider collider;
};

class PhysicsWorld
{
public:
    uint32_t CreateBody(const RigidBody& body, const Collider& collider) {
        m_bodies.push_back({body, collider});
        return static_cast<uint32_t>(m_bodies.size()) - 1;
    }

    PhysicsBody* GetBody(uint32_t id) {
        if (id == InvalidIndex || id >= static_cast<uint32_t>(m_bodies.size())) return nullptr;
        return &m_bodies[id];
    }

    const PhysicsBody* GetBody(uint32_t id) const {
        if (id == InvalidIndex || id >= static_cast<uint32_t>(m_bodies.size())) return nullptr;
        return &m_bodies[id];
    }

    void Step(float deltaTime);

    glm::vec2 gravity{0.0f, -1.0f};

private:
    std::vector<PhysicsBody> m_bodies;
    std::vector<AABB> m_aabbs;
};
