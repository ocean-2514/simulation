#pragma once

#include "Physics/RigidBody.hpp"
#include "Physics/Collision/CollisionManifold.hpp"
#include "Physics/PhysicsWorld.hpp"

class ImpulseSolver
{
public:
    static void SolvePosition(
        PhysicsBody& first, PhysicsBody& second,
        const CollisionManifold& manifold);
    
    static void SolveVelocity(
        PhysicsBody& first, PhysicsBody& second,
        const CollisionManifold& manifold);
};
