#pragma once

#include "Physics/RigidBody.hpp"
#include "Physics/Collider.hpp"
#include "Physics/BroadPhase/AABB.hpp"
#include "Physics/BroadPhase/Spatial/BVHTree2D.hpp"
#include "Utils/MathHelper.hpp"
#include "Physics/Forces/ForceGenetator.hpp"
#include "Physics/Constraints/Constraint.hpp"
#include "Physics/Collision/ContactCache.hpp"

#include <vector>
#include <memory>
#include <utility>

struct PhysicsBody {
    RigidBody rigidBody;
    Collider collider;
    AABB2 bounds{};

    void ApplyForceAtPoint(const glm::vec2& localPoint,
        const glm::vec2& force) {
        rigidBody.ApplyForce(force);
        glm::vec2 distLocal = localPoint - collider.centroid;
        glm::vec2 distWorld = MathHelper::Rotate(
            distLocal, rigidBody.transform.rotation);
        rigidBody.ApplyTorque(MathHelper::Cross(distWorld, force));
    }
};

class PhysicsWorld
{
public:
    BodyId CreateBody(const RigidBody& body,
        const Collider& collider, 
        const glm::vec2& pos = glm::vec2{0.f});

    PhysicsBody* GetBody(BodyId id) {
        if (id == InvalidIndex || id >= static_cast<BodyId>(m_bodies.size())) return nullptr;
        return &m_bodies[id];
    }

    const PhysicsBody* GetBody(BodyId id) const {
        if (id == InvalidIndex || id >= static_cast<BodyId>(m_bodies.size())) return nullptr;
        return &m_bodies[id];
    }

    void CalculateInertia(BodyId i) {
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

    void ApplyForceAtPoint(BodyId id, const glm::vec2& localPoint,
        const glm::vec2& force) {
        auto* body = GetBody(id);
        if (!body) return;
        body->ApplyForceAtPoint(localPoint, force);
    }

    void RegisterForceGenerator(std::shared_ptr<ForceGenerator> gen) {
        if (gen) m_forceGenerators.push_back(std::move(gen));
    }

    void RegisterConstraint(std::shared_ptr<Constraint> constraint) {
        if (constraint) m_constraints.push_back(std::move(constraint));
    }

    void Step(float deltaTime, int subStepCount = 5);

    glm::vec2 gravity{0.0f, -1.0f};

private:
    static AABB2 ComputeBounds(const PhysicsBody& body);
    void SubStep(float deltaTime);

    float m_currentTime = 0.f;
    std::vector<PhysicsBody> m_bodies;
    std::vector<BVHTree2D::ProxyId> m_proxyIds;
    std::vector<std::shared_ptr<ForceGenerator>> m_forceGenerators;
    std::vector<std::shared_ptr<Constraint>> m_constraints;
    BVHTree2D m_broadPhase;

    ContactCache m_contactCache;
    uint32_t m_stepIndex = 0;
};
