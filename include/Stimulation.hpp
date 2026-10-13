#include <memory>

#include "Eng.h"
#include "ShapeProxy.hpp"

#include <vector>

class Stimulation : public Application
{
public:
    bool Init() override {
        m_shader = std::make_unique<Shader>(SHADER_DIR "/vs.glsl", SHADER_DIR "/fs.glsl");

        RigidBody groundBody{};
        groundBody.motionType = MotionType::Static;
        RigidBody triangleBody{};
        triangleBody.motionType = MotionType::Kinematic;

        std::vector<glm::vec2> polygonPoints = {
            {-0.5f, -0.5f}, {0.8f, -0.4f}, {0.4f, 0.5f}, {-0.4f, 0.4f}
        };
        std::vector<glm::vec2> trianglePoints = {
            {-0.3f, -0.3f}, {0.3f, -0.3f}, {0.f, 0.3f}
        };
        std::vector<glm::vec2> groundPoints = {
            {-2.0f, -0.9f}, {-2.0f, -1.1f}, {2.0f, -1.1f}, {2.0f, -0.9f}
        };

        PolygonCollider polygonCollider{polygonPoints};
        polygonCollider.PopulateNormals();
        PolygonCollider triangleCollider{trianglePoints};
        triangleCollider.PopulateNormals();
        PolygonCollider groundCollider{groundPoints};
        groundCollider.PopulateNormals();

        m_ground = ShapeProxy::CreateProxy(
            std::make_shared<Polygon>(groundPoints),
            groundBody,
            Collider{ColliderData{groundCollider}}
        );
        m_triangle = ShapeProxy::CreateProxy(
            std::make_shared<Polygon>(trianglePoints),
            triangleBody,
            Collider{ColliderData{triangleCollider}}
        );
        m_triangle.PushModelTransform({0.0f, 1.0f});
        m_polygon = ShapeProxy::CreateProxy(
            std::make_shared<Polygon>(polygonPoints),
            RigidBody{},
            Collider{ColliderData{polygonCollider}}
        );
        m_circle = ShapeProxy::CreateProxy(
            std::make_shared<Circle>(0.5f),
            RigidBody{},
            Collider{CircleCollider{0.5f}}
        );

        return true;
    }

    void Update(float) override {
        ProcessInput();
        if (!m_shader) return;

        m_shader->use();

        glm::mat4 view = glm::mat4(1.0f);

        float aspect = static_cast<float>(Engine::GetInstance().GetWidth()) 
            / static_cast<float>(Engine::GetInstance().GetHeight());
        float halfHeight = 1.0f;
        float halfWidth = halfHeight * aspect;

        glm::mat4 projection = glm::ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, -1.0f, 10.0f);

        m_shader->setMatrix4f("uView", view);
        m_shader->setMatrix4f("uProjection", projection);

        m_shader->setVec3("uColor", 0.76f, 0.24f, 0.34f);

        {
            glm::mat4 model = m_polygon.PullModelTransform();
            m_shader->setMatrix4f("uModel", model);
            m_polygon.Draw();
        }
        {
            glm::mat4 model = m_triangle.PullModelTransform();
            m_shader->setMatrix4f("uModel", model);
            m_triangle.Draw();
        }
        {
            glm::mat4 model = m_circle.PullModelTransform();
            m_shader->setMatrix4f("uModel", model);
            m_circle.Draw();
        }
        m_shader->setMatrix4f("uModel", glm::mat4{1.0f});
        m_ground.Draw();
    }

    void ProcessInput() {
        const auto& inputManager = Engine::GetInstance().GetInputManager();
        if (inputManager.IsKeyPressed(static_cast<int>(Key::Escape))) {
            Engine::GetInstance().SetWindowShouldClose(true);
        }

        glm::vec2 direction{0.0f};
        if (inputManager.IsKeyPressed(static_cast<int>(Key::W))) direction.y += 1.0f;
        if (inputManager.IsKeyPressed(static_cast<int>(Key::S))) direction.y -= 1.0f;
        if (inputManager.IsKeyPressed(static_cast<int>(Key::A))) direction.x -= 1.0f;
        if (inputManager.IsKeyPressed(static_cast<int>(Key::D))) direction.x += 1.0f;

        const float lengthSquared = glm::dot(direction, direction);
        if (lengthSquared > 1.0f) {
            direction *= glm::inversesqrt(lengthSquared);
        }
        m_triangle.SetLinearVelocity(direction * m_triangleMoveSpeed);
    }

private:
    static constexpr float m_triangleMoveSpeed = 1.5f;
    std::unique_ptr<Shader> m_shader;
    ShapeProxy m_ground;
    ShapeProxy m_polygon;
    ShapeProxy m_triangle;
    ShapeProxy m_circle;
};
