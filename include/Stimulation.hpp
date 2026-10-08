#include <memory>

#include "Eng.h"

class Stimulation : public Application
{
public:
    bool Init() override {
        m_shader = std::make_unique<Shader>(SHADER_DIR "/vs.glsl", SHADER_DIR "/fs.glsl");
        m_circle = std::make_unique<Circle>(0.7f, 64);
        m_polygon = std::make_unique<Polygon>(std::vector<glm::vec2>{
            glm::vec2(-0.5f, -0.5f),
            glm::vec2(0.8f, -0.4f),
            glm::vec2(0.4f, 0.5f),
            glm::vec2(-0.4f, 0.4f)
        });

        return true;
    }

    void Update(float deltaTime) override {
        ProcessInput();
        
        if (!m_shader || !m_circle) {
            return;
        }

        m_shader->use();

        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = glm::mat4(1.0f);

        float aspect = static_cast<float>(Engine::GetInstance().GetWidth()) 
            / static_cast<float>(Engine::GetInstance().GetHeight());
        float halfHeight = 1.0f;
        float halfWidth = halfHeight * aspect;

        glm::mat4 projection = glm::ortho(-halfWidth, halfWidth, -halfHeight, halfHeight, -1.0f, 10.0f);

        m_shader->setMatrix4f("uModel", model);
        m_shader->setMatrix4f("uView", view);
        m_shader->setMatrix4f("uProjection", projection);

        // glBindVertexArray(m_circle->VAO);
        // glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_circle->indices.size()), GL_UNSIGNED_INT, 0);
        m_shader->setVec3("uColor", 0.76f, 0.24f, 0.34f);
        glBindVertexArray(m_polygon->VAO);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_polygon->indices.size()), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        m_shader->setVec3("uColor", 0.0f, 0.0f, 0.0f);
        m_polygon->drawCentroid();
    }

    void ProcessInput() {
        const auto& inputManager = Engine::GetInstance().GetInputManager();
        if (inputManager.IsKeyPressed(static_cast<int>(Key::Escape))) {
            Engine::GetInstance().SetWindowShouldClose(true);
        }
    }

private:
    std::unique_ptr<Shader> m_shader;
    std::unique_ptr<Circle> m_circle;
    std::unique_ptr<Polygon> m_polygon;
};