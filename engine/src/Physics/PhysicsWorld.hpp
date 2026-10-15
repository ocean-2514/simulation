#pragma once

#include "Physics/RigidBody.hpp"
#include "Physics/Collider.hpp"
#include "BroadPhase/AABB.hpp"
#include "Utils/MathHelper.hpp"
#include <vector>
#include <iostream>

struct PhysicsBody {
    RigidBody rigidBody;
    Collider collider;
};

class PhysicsWorld
{
public:
    uint32_t CreateBody(const RigidBody& body, 
        const Collider& collider, 
        const glm::vec2& pos = glm::vec2{0.f}) {
        m_bodies.push_back({body, collider});
        auto& back = m_bodies.back();
        back.rigidBody.SetPosition(pos);
        back.rigidBody.SetInertia(MathHelper::CalculateInertia(
            back.collider, back.rigidBody.GetMass()));
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

    void CalculateInertia(uint32_t i) {
        if (auto* body = GetBody(i)) {
            body->rigidBody.SetInertia(MathHelper::CalculateInertia(
                body->collider, body->rigidBody.GetMass()));
        }
    }

    void PopulateInertia() {
        for (auto& [body, collider] : m_bodies) {
            body.SetInertia(MathHelper::CalculateInertia(collider, body.GetMass()));
        }
    }

    void Step(float deltaTime);

    glm::vec2 gravity{0.0f, -1.0f};

private:
    std::vector<PhysicsBody> m_bodies;
    std::vector<AABB> m_aabbs;
};
