#include <memory>

#include "Eng.h"
#include "ShapeProxy.hpp"

#include <vector>

class Stimulation : public Application
{
public:
    bool Init() override {
        m_shader = std::make_unique<Shader>(SHADER_DIR "/vs.glsl", SHADER_DIR "/fs.glsl");

        RigidBody boundsBody{};
        boundsBody.motionType = MotionType::Static;
        boundsBody.inverseMass = 0.f;
        RigidBody triangleBody{};
        triangleBody.motionType = MotionType::Kinematic;
        triangleBody.inverseMass = 0.f;

        std::vector<glm::vec2> polygonPoints = {
            {-0.5f, -0.5f}, {0.8f, -0.5f}, {0.4f, 0.5f}, {-0.4f, 0.4f}
        };
        std::vector<glm::vec2> trianglePoints = {
            {-0.3f, -0.3f}, {0.3f, -0.3f}, {0.f, 0.3f}
        };
        PolygonCollider polygonCollider{polygonPoints};
        polygonCollider.PopulateNormals();
        PolygonCollider triangleCollider{trianglePoints};
        triangleCollider.PopulateNormals();

        constexpr float boundsHalfWidth = 3.1f;
        constexpr float boundsBottom = -1.6f;
        constexpr float boundsTop = 1.6f;
        constexpr float wallThickness = 0.2f;
        auto createWall = [&boundsBody](float width, float height,
            const glm::vec2& position) {
            std::vector<glm::vec2> wallPoints = {
                {-width * 0.5f, -height * 0.5f},
                { width * 0.5f, -height * 0.5f},
                { width * 0.5f,  height * 0.5f},
                {-width * 0.5f,  height * 0.5f}
            };
            PolygonCollider wallCollider{wallPoints};
            wallCollider.PopulateNormals();
            return ShapeProxy::CreateProxy(
                std::make_shared<Polygon>(wallPoints),
                boundsBody,
                Collider{ColliderData{wallCollider}},
                position
            );
        };
        const float boundsHeight = boundsTop - boundsBottom;
        const float boundsCenterY = (boundsTop + boundsBottom) * 0.5f;
        m_bounds.reserve(4);
        m_bounds.push_back(createWall(
            boundsHalfWidth * 2.0f + wallThickness * 2.0f,
            wallThickness,
            {0.0f, boundsBottom - wallThickness * 0.5f}
        ));
        m_bounds.push_back(createWall(
            boundsHalfWidth * 2.0f + wallThickness * 2.0f,
            wallThickness,
            {0.0f, boundsTop + wallThickness * 0.5f}
        ));
        m_bounds.push_back(createWall(
            wallThickness,
            boundsHeight + wallThickness * 2.0f,
            {-boundsHalfWidth - wallThickness * 0.5f, boundsCenterY}
        ));
        m_bounds.push_back(createWall(
            wallThickness,
            boundsHeight + wallThickness * 2.0f,
            {boundsHalfWidth + wallThickness * 0.5f, boundsCenterY}
        ));
        m_triangle = ShapeProxy::CreateProxy(
            std::make_shared<Polygon>(trianglePoints),
            triangleBody,
            Collider{ColliderData{triangleCollider}},
            {0.f, 1.f}
        );
        m_polygon = ShapeProxy::CreateProxy(
            std::make_shared<Polygon>(polygonPoints),
            RigidBody{},
            Collider{polygonCollider},
            {2.f, 1.05f}
        );
        m_circle = ShapeProxy::CreateProxy(
            std::make_shared<Circle>(0.5f),
            RigidBody{},
            Collider{CircleCollider{0.5f}},
            {-1.f, 1.f}
        );

        constexpr float fixedObjectHalfWidth = 0.25f;
        constexpr float fixedObjectHalfHeight = 0.16f;
        constexpr float fixedObjectAnchorY = 0.0f;
        std::vector<glm::vec2> fixedObjectPoints = {
            {-fixedObjectHalfWidth, -fixedObjectHalfHeight},
            { fixedObjectHalfWidth, -fixedObjectHalfHeight},
            { fixedObjectHalfWidth,  fixedObjectHalfHeight},
            {-fixedObjectHalfWidth,  fixedObjectHalfHeight}
        };
        PolygonCollider fixedObjectCollider{fixedObjectPoints};
        fixedObjectCollider.PopulateNormals();
        m_fixedObject = ShapeProxy::CreateProxy(
            std::make_shared<Polygon>(fixedObjectPoints),
            RigidBody{},
            Collider{ColliderData{fixedObjectCollider}},
            {boundsHalfWidth - fixedObjectHalfWidth, fixedObjectAnchorY}
        );

        constexpr float hingedObjectHalfWidth = 0.325f;
        constexpr float hingedObjectHalfHeight = 0.08f;
        constexpr float hingedObjectAnchorY = 0.75f;
        std::vector<glm::vec2> hingedObjectPoints = {
            {-hingedObjectHalfWidth, -hingedObjectHalfHeight},
            { hingedObjectHalfWidth, -hingedObjectHalfHeight},
            { hingedObjectHalfWidth,  hingedObjectHalfHeight},
            {-hingedObjectHalfWidth,  hingedObjectHalfHeight}
        };
        PolygonCollider hingedObjectCollider{hingedObjectPoints};
        hingedObjectCollider.PopulateNormals();
        m_hingedObject = ShapeProxy::CreateProxy(
            std::make_shared<Polygon>(hingedObjectPoints),
            RigidBody{},
            Collider{ColliderData{hingedObjectCollider}},
            {-boundsHalfWidth + hingedObjectHalfWidth, hingedObjectAnchorY}
        );

        constexpr float ballSpacing = 0.19f;
        auto ballGeometry = std::make_shared<Circle>(m_ballRadius, 32);
        RigidBody ballBody{};
        ballBody.restitution = 0.15f;
        ballBody.friction = 0.6f;
        m_balls.reserve(m_ballColumns * m_ballRows);
        for (int row = 0; row < m_ballRows; ++row) {
            const float rowOffset = row % 2 == 0 ? 0.0f : ballSpacing * 0.5f;
            for (int column = 0; column < m_ballColumns; ++column) {
                const float x = -0.5f * (m_ballColumns - 1) * ballSpacing
                    + column * ballSpacing + rowOffset;
                const float y = m_ballStartY + row * ballSpacing;
                m_balls.push_back(ShapeProxy::CreateProxy(
                    ballGeometry,
                    ballBody,
                    Collider{CircleCollider{m_ballRadius}},
                    {x, y}
                ));
            }
        }

        auto& world = Engine::GetInstance().GetPhysicsWorld();
        auto springForce = std::make_shared<SpringForceGenerator>(
            SpringForceDesc{m_triangle.physicsHandle, m_circle.physicsHandle}
        );
        FixedConstraintDesc fixedConstraintDesc{};
        fixedConstraintDesc.bodyA = m_bounds[3].physicsHandle;
        fixedConstraintDesc.bodyB = m_fixedObject.physicsHandle;
        fixedConstraintDesc.localAnchorA = {
            -wallThickness * 0.5f,
            fixedObjectAnchorY - boundsCenterY
        };
        fixedConstraintDesc.localAnchorB = {fixedObjectHalfWidth, 0.0f};
        fixedConstraintDesc.referenceAngle = 0.0f;
        auto fixedConstraint = std::make_shared<FixedConstraint>(
            fixedConstraintDesc);
        HingeConstraintDesc hingeConstraintDesc{};
        hingeConstraintDesc.bodyA = m_bounds[2].physicsHandle;
        hingeConstraintDesc.bodyB = m_hingedObject.physicsHandle;
        hingeConstraintDesc.localAnchorA = {
            wallThickness * 0.5f,
            hingedObjectAnchorY - boundsCenterY
        };
        hingeConstraintDesc.localAnchorB = {-hingedObjectHalfWidth, 0.0f};
        auto hingeConstraint = std::make_shared<HingeConstraint>(
            hingeConstraintDesc);
        world.RegisterForceGenerator(springForce);
        world.RegisterConstraint(fixedConstraint);
        world.RegisterConstraint(hingeConstraint);

        return true;
    }

    void Update(float) override {
        ProcessInput();
        if (!m_shader) return;

        m_shader->use();

        glm::mat4 view = glm::mat4(1.0f);

        float aspect = static_cast<float>(Engine::GetInstance().GetWidth()) 
            / static_cast<float>(Engine::GetInstance().GetHeight());
        float halfHeight = 1.0f * 2;
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
        m_shader->setVec3("uColor", 0.24f, 0.72f, 0.56f);
        {
            glm::mat4 model = m_fixedObject.PullModelTransform();
            m_shader->setMatrix4f("uModel", model);
            m_fixedObject.Draw();
        }
        m_shader->setVec3("uColor", 0.24f, 0.48f, 0.86f);
        {
            glm::mat4 model = m_hingedObject.PullModelTransform();
            m_shader->setMatrix4f("uModel", model);
            m_hingedObject.Draw();
        }
        m_shader->setVec3("uColor", 0.76f, 0.24f, 0.34f);
        for (const auto& ball : m_balls) {
            glm::mat4 model = ball.PullModelTransform();
            m_shader->setMatrix4f("uModel", model);
            ball.Draw();
        }
        m_shader->setMatrix4f("uModel", glm::mat4{1.0f});
        for (const auto& bound : m_bounds) {
            glm::mat4 model = bound.PullModelTransform();
            m_shader->setMatrix4f("uModel", model);
            bound.Draw();
        }
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
    static constexpr int m_ballColumns = 25;
    static constexpr int m_ballRows = 8;
    static constexpr float m_ballRadius = 0.075f;
    static constexpr float m_ballStartY = -1.0f;
    static constexpr float m_triangleMoveSpeed = 1.5f;
    std::unique_ptr<Shader> m_shader;
    std::vector<ShapeProxy> m_bounds;
    std::vector<ShapeProxy> m_balls;
    ShapeProxy m_polygon;
    ShapeProxy m_triangle;
    ShapeProxy m_circle;
    ShapeProxy m_fixedObject;
    ShapeProxy m_hingedObject;
};
