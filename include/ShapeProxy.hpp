#pragma once

#include "Eng.h"

#include <glm/gtc/matrix_transform.hpp>
#include <memory>

class ShapeProxy {
public:
    std::shared_ptr<Shape> shape;
    uint32_t physicsHandle = InvalidIndex;
    
    static PhysicsWorld& World() {
        static PhysicsWorld& world = Engine::GetInstance().GetPhysicsWorld();
        return world;
    }

    static ShapeProxy CreateProxy(std::shared_ptr<Shape> shape, 
        const RigidBody& body, const Collider& collider, 
        const glm::vec2& pos = glm::vec2{0.f}) {
        ShapeProxy proxy;
        proxy.shape = std::move(shape);
        proxy.physicsHandle = World().CreateBody(body, collider, pos);
        return proxy;
    }

    void Draw() const {
        if (!shape) return;
        glBindVertexArray(shape->VAO);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(shape->indices.size()), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

    glm::mat4 PullModelTransform() const {
        const auto* body = World().GetBody(physicsHandle);
        if (!body) return glm::mat4{1.0f};

        const Transform2D& transform = body->rigidBody.transform;
        glm::mat4 model = glm::translate(
            glm::mat4{1.0f}, glm::vec3{transform.position, 0.0f});
        return glm::rotate(
            model, transform.rotation, glm::vec3{0.0f, 0.0f, 1.0f});
    }

    bool PushModelTransform(const glm::vec2& pos) {
        auto* body = World().GetBody(physicsHandle);
        if (!body) return false;
        if (!body->rigidBody.IsKinematic()) {
            return false;
        }
        body->rigidBody.SetPosition(pos);
        return true;
    }

    bool SetLinearVelocity(const glm::vec2& velocity) {
        auto* body = World().GetBody(physicsHandle);
        if (!body || !body->rigidBody.IsKinematic()) return false;
        body->rigidBody.SetLinearVelocity(velocity);
        return true;
    }

    bool IsDynamic() const {
        const auto* body = World().GetBody(physicsHandle);
        return body && body->rigidBody.IsDynamic();
    }
};

