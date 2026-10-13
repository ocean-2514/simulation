#include "Physics/PhysicsWorld.hpp"
#include "Physics/Collision/CollisionDetector.hpp"
#include "Physics/Solver/ImpulseSolver.hpp"

void PhysicsWorld::Step(float deltaTime) {
    /*
    1. Integrate forces for dynamic bodies
    2. Integrate velocity for dynamic and kinematic bodies
    3. Check all body pairs with narrow phase
    4. Correct penetration for dynamic bodies
    */

    for (auto& physicsBody : m_bodies) {
        physicsBody.rigidBody.IntegrateForces(gravity, deltaTime);
    }

    for (auto& physicsBody : m_bodies) {
        physicsBody.rigidBody.IntegrateVelocity(deltaTime);
    }

    // m_aabbs.clear();
    // m_aabbs.reserve(m_bodies.size());
    // for (const auto& physicsBody : m_bodies) {
    //     // TODO: compute AABB from collider and transform once collider-specific logic exists.
    //     // This is a placeholder so the world step is still structurally coherent.
    //     m_aabbs.push_back(AABB{});
    // }

    for (std::size_t i = 0; i < m_bodies.size(); ++i) {
        for (std::size_t j = i + 1; j < m_bodies.size(); ++j) {
            auto& first = m_bodies[i];
            auto& second = m_bodies[j];
            const auto& manifold = CollisionDetector::Detect(
                first.collider, first.rigidBody.transform,
                second.collider, second.rigidBody.transform
            );
            if (!manifold.colliding) continue;
            ImpulseSolver::SolvePosition(
                first.rigidBody, second.rigidBody, manifold);
        }
    }
}
