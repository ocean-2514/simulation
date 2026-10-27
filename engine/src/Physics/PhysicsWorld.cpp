#include "Physics/PhysicsWorld.hpp"
#include "Physics/Collision/CollisionDetector.hpp"
#include "Physics/Collision/ContactConstraint.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace
{

bool CanCollide(const RigidBody& bodyA, const RigidBody& bodyB) {
    return (bodyA.categoryBits & bodyB.maskBits) != 0
        && (bodyA.maskBits & bodyB.categoryBits) != 0;
}

} // namespace

AABB2 PhysicsWorld::ComputeBounds(const PhysicsBody& body) {
    const AABB2 invalidBounds{{1.0f, 1.0f}, {-1.0f, -1.0f}};
    const Transform2D& transform = body.rigidBody.transform;

    if (body.collider.Type() == ColliderType::Circle) {
        const float radius = std::get<CircleCollider>(body.collider.data).radius;
        if (!std::isfinite(radius) || radius < 0.0f
            || !std::isfinite(transform.position.x)
            || !std::isfinite(transform.position.y)) {
            return invalidBounds;
        }
        const glm::vec2 extent{radius};
        return {transform.position - extent, transform.position + extent};
    }

    const auto& polygon = std::get<PolygonCollider>(body.collider.data);
    if (polygon.vertices.empty()) return invalidBounds;

    AABB2 bounds{
        {std::numeric_limits<float>::max(), std::numeric_limits<float>::max()},
        {-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()}
    };
    for (const glm::vec2& vertex : polygon.vertices) {
        const glm::vec2 worldVertex = WorldPoint(transform, vertex);
        if (!std::isfinite(worldVertex.x) || !std::isfinite(worldVertex.y)) {
            return invalidBounds;
        }
        bounds.min.x = std::min(bounds.min.x, worldVertex.x);
        bounds.min.y = std::min(bounds.min.y, worldVertex.y);
        bounds.max.x = std::max(bounds.max.x, worldVertex.x);
        bounds.max.y = std::max(bounds.max.y, worldVertex.y);
    }
    return bounds;
}

BodyId PhysicsWorld::CreateBody(
    const RigidBody& body,
    const Collider& collider,
    const glm::vec2& pos) {
    if (m_bodies.size() >= static_cast<std::size_t>(InvalidIndex)) {
        return InvalidIndex;
    }

    PhysicsBody physicsBody{body, collider};
    physicsBody.rigidBody.SetPosition(pos);
    physicsBody.rigidBody.SetInertia(MathHelper::CalculateInertia(
        physicsBody.collider, physicsBody.rigidBody.GetMass()));
    physicsBody.bounds = ComputeBounds(physicsBody);

    const BodyId bodyId = static_cast<BodyId>(m_bodies.size());
    m_bodies.push_back(std::move(physicsBody));
    m_proxyIds.push_back(m_broadPhase.CreateProxy(bodyId, m_bodies.back().bounds));
    return bodyId;
}

void PhysicsWorld::Step(float deltaTime, int subStepCount) {
    if (!(deltaTime > 0.0f) || !std::isfinite(deltaTime)
        || subStepCount <= 0) return;
    float delta = deltaTime / subStepCount;
    for (int i = 0; i < subStepCount; ++i) {
        SubStep(delta);
    }   
}

void PhysicsWorld::SubStep(float deltaTime) {
    m_currentTime += deltaTime;
    ForceContext forceContext{*this, m_currentTime};
    for (auto& gen : m_forceGenerators) {
        if (gen) gen->Apply(forceContext, deltaTime);
    }
    for (auto& physicsBody : m_bodies) {
        physicsBody.rigidBody.Update(gravity, deltaTime);
    }

    // Update tight bounds every frame. The BVH only needs reinsertion when an
    // object's new tight bounds leave its fat leaf bounds.
    for (std::size_t i = 0; i < m_bodies.size(); ++i) {
        PhysicsBody& body = m_bodies[i];
        body.bounds = ComputeBounds(body);
        BVHTree2D::ProxyId& proxy = m_proxyIds[i];

        if (!body.bounds.IsValid()) {
            if (proxy != BVHTree2D::InvalidIndex) {
                m_broadPhase.DestroyProxy(proxy);
                proxy = BVHTree2D::InvalidIndex;
            }
        } else if (proxy == BVHTree2D::InvalidIndex) {
            proxy = m_broadPhase.CreateProxy(
                static_cast<uint32_t>(i), body.bounds);
        } else {
            m_broadPhase.MoveProxy(proxy, body.bounds);
        }
    }

    std::vector<ContactConstraint> contacts;
    std::vector<BVHTree2D::ObjectHandle> candidates;
    for (std::size_t i = 0; i < m_bodies.size(); ++i) {
        candidates.clear();
        m_broadPhase.Query(m_bodies[i].bounds, candidates);

        for (const BVHTree2D::ObjectHandle candidate : candidates) {
            const std::size_t j = candidate;
            if (j <= i || j >= m_bodies.size()) continue;

            auto& first = m_bodies[i];
            auto& second = m_bodies[j];
            if (!CanCollide(first.rigidBody, second.rigidBody)) continue;
            if (!first.rigidBody.IsDynamic()
                && !second.rigidBody.IsDynamic()) {
                continue;
            }

            const auto manifold = CollisionDetector::Detect(
                first.collider, first.rigidBody.transform,
                second.collider, second.rigidBody.transform
            );
            if (!manifold.colliding) continue;
            contacts.emplace_back(
                static_cast<BodyId>(i), static_cast<BodyId>(j), manifold);
        }
    }

    // prepare
    SolverContext solverContext{
        deltaTime, 1.0f / deltaTime, gravity, {}, *this
    };
    for (auto& contact : contacts) {
        contact.Prepare(solverContext);
    }
    for (auto& constraint : m_constraints) {
        constraint->Prepare(solverContext);
    }
    for (auto& contact : contacts) {
        contact.WarmStart(solverContext);
    }
    for (auto& constraint : m_constraints) {
        constraint->WarmStart(solverContext);
    }


    int velocityIterations = 5;
    for (int i = 0; i < velocityIterations; ++i) {
        for (auto& contact : contacts) {
            contact.SolveVelocity(solverContext);
        }
        for (auto& constraint : m_constraints) {
            constraint->SolveVelocity(solverContext);
        }
    }
    int positionIterations = 5;
    for (int i = 0; i < positionIterations; ++i) {
        for (auto& contact : contacts) {
            contact.SolvePosition(solverContext);
        }
        for (auto& constraint : m_constraints) {
            constraint->SolvePosition(solverContext);
        }
    }
}
