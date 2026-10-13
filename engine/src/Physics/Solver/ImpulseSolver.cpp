#include "Physics/Solver/ImpulseSolver.hpp"

#include <algorithm>

void ImpulseSolver::SolvePosition(
    RigidBody& first, RigidBody& second,
    const CollisionManifold& manifold) {
    if (!manifold.colliding) return;
    const float firstInverseMass = first.IsDynamic()
        ? std::max(first.inverseMass, 0.0f)
        : 0.0f;
    const float secondInverseMass = second.IsDynamic()
        ? std::max(second.inverseMass, 0.0f)
        : 0.0f;
    const float inverseMassSum = firstInverseMass + secondInverseMass;
    if (inverseMassSum <= 0.0f) return;

    const glm::vec2 correction = manifold.normal * manifold.penetration;
    first.Translate(-correction * (firstInverseMass / inverseMassSum));
    second.Translate(correction * (secondInverseMass / inverseMassSum));
}
