#pragma once

#include "Physics/PhysicsTypes.hpp"
#include "Physics/Collision/CollisionGroup.hpp"

class RigidBody {
public:
    Transform2D transform{};
    glm::vec2 linearVelocity{0.0f};
    float angularVelocity = 0.0f;
    glm::vec2 force{0.0f};
    float torque = 0.0f;

    float inverseMass = 1.0f;
    float inverseInertia = 0.0f;

    float restitution = 0.5f;
    float friction = 0.5f;

    MotionType motionType = MotionType::Dynamic;

    uint16_t categoryBits = static_cast<uint16_t>(CollisionGroup::GROUP0);
    uint16_t maskBits = DefaultCollisionMask;

    // dont secure that offset is inside the body
    glm::vec2 VelocityAtPoint(const glm::vec2& worldOffset) {
        if (IsStatic()) return glm::vec2{0.0f};
        return linearVelocity + glm::vec2{
            -angularVelocity * worldOffset.y,
            angularVelocity * worldOffset.x
        };
    }

    void AddImpulse(const glm::vec2& impulse) {
        if (motionType == MotionType::Static) return;
        linearVelocity += impulse * inverseMass;
    }

    void AddAngularImpulse(float impulse) {
        if (motionType == MotionType::Static) return;
        angularVelocity += impulse * inverseInertia;
    }

    void ApplyForce(const glm::vec2& value) {
        if (motionType == MotionType::Dynamic) {
            force += value;
        }
    }

    void ApplyTorque(float value) {
        if (motionType == MotionType::Dynamic) {
            torque += value;
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

    void Update(const glm::vec2& gravity, float deltaTime) {
        IntegrateForces(gravity, deltaTime);
        IntegrateVelocity(deltaTime);
    }

    void Translate(const glm::vec2& pos) {
        transform.position += pos;
    }

    void Rotate(float angle) {
        transform.rotation += angle;
    }

    void SetLinearVelocity(const glm::vec2& velocity) {
        linearVelocity = velocity;
    }

    void SetAngularVelocity(float velocity) {
        angularVelocity = velocity;
    }

    void SetMass(float mass) {
        inverseMass = mass <= 1e-6f ? 0.f : 1.f / mass;
    }

    float GetMass() const {
        return inverseMass < 0.f ? 0.f : 1.f / inverseMass;
    }

    void SetInertia(float inertia) {
        inverseInertia = inertia <= 1e-6f ? 0.f : 1.f / inertia;
    }

    void SetPosition(const glm::vec2& pos) {
        transform.position = pos;
    }

    void SetRotation(float rotation) {
        transform.rotation = rotation;
    }

    void SetTransform(const Transform2D& trans) {
        transform = trans;
    }

    void SetCollisionGroup(CollisionGroup group) {
        categoryBits = static_cast<uint16_t>(group);
    }

    void AddCollisionGroup(CollisionGroup group) {
        categoryBits |= static_cast<uint16_t>(group);
    }

    void SetCategoryBits(uint16_t bits) {
        categoryBits = bits;
    }

    void SetCollisionMask(CollisionGroup group) {
        maskBits = static_cast<uint16_t>(group);
    }

    void AddCollisionMask(CollisionGroup group) {
        maskBits |= static_cast<uint16_t>(group);
    }

    void SetMaskBits(uint16_t bits) {
        maskBits = bits;
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
