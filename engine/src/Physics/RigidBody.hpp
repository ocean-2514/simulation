#pragma once

#include "Physics/PhysicsTypes.hpp"

class RigidBody {
public:
    Transform2D transform;
    glm::vec2 linearVelocity{0.0f};
    float angularVelocity = 0.0f;
    glm::vec2 force{0.0f};
    float torque = 0.0f;

    float inverseMass = 1.0f;
    float inverseInertia = 0.0f;
    bool isStatic = false;

    void ApplyForce(const glm::vec2& value) {
        if (!isStatic) {
            force += value;
        }
    }

    void IntegrateForces(const glm::vec2& gravity, float deltaTime) {
        if (isStatic) return;

        linearVelocity += (gravity + force * inverseMass) * deltaTime;
        force = glm::vec2{0.0f};
        angularVelocity += torque * inverseInertia * deltaTime;
        torque = 0.0f;
    }

    void IntegrateVelocity(const glm::vec2& gravity, float deltaTime) {
        if (isStatic) return;

        transform.position += linearVelocity * deltaTime;
        transform.rotation += angularVelocity * deltaTime;
    }
};