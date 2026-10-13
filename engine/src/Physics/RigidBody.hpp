#pragma once

#include "Physics/PhysicsTypes.hpp"

class RigidBody {
public:
    Transform2D transform{};
    glm::vec2 linearVelocity{0.0f};
    float angularVelocity = 0.0f;
    glm::vec2 force{0.0f};
    float torque = 0.0f;

    float inverseMass = 1.0f;
    float inverseInertia = 0.0f;

    float restitution = 0.2f;
    float friction = 0.5f;

    MotionType motionType = MotionType::Dynamic;

    void ApplyForce(const glm::vec2& value) {
        if (motionType == MotionType::Dynamic) {
            force += value;
        }
    }

    void IntegrateForces(const glm::vec2& gravity, float deltaTime) {
        if (motionType != MotionType::Dynamic) return;

        linearVelocity += (gravity + force * inverseMass) * deltaTime;
        force = glm::vec2{0.0f};
        angularVelocity += torque * inverseInertia * deltaTime;
        torque = 0.0f;
    }

    void IntegrateVelocity(float deltaTime) {
        if (motionType == MotionType::Static) return;

        transform.position += linearVelocity * deltaTime;
        transform.rotation += angularVelocity * deltaTime;
    }

    void Translate(const glm::vec2& pos) {
        transform.position += pos;
    }

    void Rotate(float angle) {
        transform.rotation += angle;
    }

    bool IsDynamic() const {
        return motionType == MotionType::Dynamic;
    }

    bool IsKinematic() const {
        return motionType == MotionType::Kinematic;
    }

    bool IsStatic() const {
        return motionType == MotionType::Static;
    }
};
