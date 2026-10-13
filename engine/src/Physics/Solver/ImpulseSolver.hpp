#pragma once

#include "Physics/RigidBody.hpp"
#include "Physics/Collision/CollisionManifold.hpp"

class ImpulseSolver
{
public:
    static void SolvePosition(
        RigidBody& first, RigidBody& second,
        const CollisionManifold& manifold);
};
