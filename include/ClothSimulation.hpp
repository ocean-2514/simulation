#pragma once

#include "Eng.h"

class ClothSimulation : public Application
{
public:
    bool Init() override {
        m_shader = std::make_unique<Shader>(SHADER_DIR "/cloth/vs.glsl", SHADER_DIR "/cloth/fs.glsl");
        m_cloth = ClothGrid::Create({.origin = {0.0f, -.5f, 0.0f}});
        m_cloth->PinParticle(0);
        m_cloth->PinParticle(15);
        m_clothRenderer = std::make_shared<ClothMeshRenderer>(m_cloth);
        m_solver = std::make_unique<ClothSolver>(m_cloth);
        return true;
    }

    void Update(float deltaTime) override {
        ProcessInput(deltaTime);

        const auto& engine = Engine::GetInstance();
        m_shader->use();
        m_shader->setVec3("uColor", glm::vec3(1.0f, 0.0f, 0.0f));
        m_shader->setMatrix4f("uModel", glm::mat4{1.0f});
        m_shader->setMatrix4f("uView", m_camera.GetViewMatrix());
        m_shader->setMatrix4f("uProjection", 
            glm::perspective(m_camera.Zoom, static_cast<float>(engine.GetWidth()) / engine.GetHeight(), 0.01f, 100.0f));

        m_solver->Step(deltaTime);
        m_clothRenderer->RenewVertices();
        m_clothRenderer->Draw();
    }

private:
    void ProcessInput(float deltaTime) {
        const auto& input = Engine::GetInstance().GetInputManager();
        if (input.IsKeyPressed(Key::A)) m_camera.ProcessKeyboard(CameraMovement::LEFT, deltaTime);
        else if (input.IsKeyPressed(Key::D)) m_camera.ProcessKeyboard(CameraMovement::RIGHT, deltaTime);
        else if (input.IsKeyPressed(Key::W)) m_camera.ProcessKeyboard(CameraMovement::FORWARD, deltaTime);
        else if (input.IsKeyPressed(Key::S)) m_camera.ProcessKeyboard(CameraMovement::BACKWARD, deltaTime);
        else if (input.IsKeyPressed(Key::Q)) m_camera.ProcessKeyboard(CameraMovement::UP, deltaTime);
        else if (input.IsKeyPressed(Key::E)) m_camera.ProcessKeyboard(CameraMovement::DOWN, deltaTime);
        if (input.IsKeyPressed(Key::Escape)) SetShouldStop(true);

        glm::vec2 mouseOffset = input.GetMousePosOffset();
        m_camera.ProcessMouseMovement(mouseOffset.x, -mouseOffset.y);
    }

    Camera m_camera;
    std::shared_ptr<Shader> m_shader;

    std::unique_ptr<ClothSolver> m_solver;
    std::shared_ptr<ClothGrid> m_cloth;
    std::shared_ptr<ClothMeshRenderer> m_clothRenderer;
};
