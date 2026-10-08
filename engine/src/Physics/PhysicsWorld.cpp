#include "Physics/PhysicsWorld.hpp"

void PhysicsWorld::Step(float deltaTime) {
    /*
    1. IntegrateForces
    2. ComputeAABBs
    3. BroadPhase
    4. NarrowPhase
    5. SolveVelocity
    6. SolvePosition
    7. IntegrateVelocity
    */

    for (auto& physicsBody : m_bodies) {
        physicsBody.rigidBody.IntegrateForces(gravity, deltaTime);
    }

    for (auto& physicsBody : m_bodies) {
        physicsBody.rigidBody.IntegrateVelocity(gravity, deltaTime);
    }

    m_aabbs.clear();
    m_aabbs.reserve(m_bodies.size());
    for (const auto& physicsBody : m_bodies) {
        // TODO: compute AABB from collider and transform once collider-specific logic exists.
        // This is a placeholder so the world step is still structurally coherent.
        m_aabbs.push_back(AABB{});
    }
}