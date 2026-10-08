#pragma once

#include "Physics/RigidBody.hpp"
#include "Physics/Collider.hpp"
#include "BroadPhase/AABB.hpp"
#include "Physics/Collision/CollisionDetector.hpp"
#include <vector>

struct PhysicsBody {
    RigidBody rigidBody;
    Collider collider;
};

class PhysicsWorld
{
public:
    int CreateBody(const RigidBody& body, const Collider& collider) {
        m_bodies.push_back({body, collider});
        return static_cast<int>(m_bodies.size()) - 1;
    }

    PhysicsBody* GetBody(int id) {
        if (id < 0 || id >= static_cast<int>(m_bodies.size())) return nullptr;
        return &m_bodies[id];
    }

    const PhysicsBody* GetBody(int id) const {
        if (id < 0 || id >= static_cast<int>(m_bodies.size())) return nullptr;
        return &m_bodies[id];
    }

    void Step(float fixedDeltaTime);

    glm::vec2 gravity{0.0f, -9.8f};

private:
    std::vector<PhysicsBody> m_bodies;
    std::vector<AABB> m_aabbs;
};