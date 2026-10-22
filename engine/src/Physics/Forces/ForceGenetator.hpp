#pragma once

#include "Physics/PhysicsTypes.hpp"

class PhysicsWorld;

struct ForceContext {
    PhysicsWorld& world;
    float simulationTime;
};  

class ForceGenerator
{
public:
    virtual ~ForceGenerator() = default;
    virtual void Apply(ForceContext& context, float deltaTime) = 0;
};