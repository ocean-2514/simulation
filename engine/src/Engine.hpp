#pragma once

#include "Application.hpp"
#include "Input/InputManager.hpp"
#include "Physics/PhysicsWorld.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>

class Engine
{
public:
    static Engine& GetInstance() {
        static Engine engine;
        return engine;
    }

    bool Init();
    void Run();

    void SetApplication(Application* app) {
        m_app = app;
    }

    void SetWindowShouldClose(bool value) {
        glfwSetWindowShouldClose(m_window, value);
    }
 
    int GetWidth() const {
        return m_screenWidth;
    }
    
    int GetHeight() const {
        return m_screenHeight;
    }

    InputManager& GetInputManager() {
        return m_inputManager;
    }

    PhysicsWorld& GetPhysicsWorld() {
        return m_world;
    }

private:
    Engine(int screenWidth = 1920, int screenHeight = 1080)
    : m_window(nullptr), m_app(nullptr),
      m_screenWidth(screenWidth), m_screenHeight(screenHeight) {}

    ~Engine() {
	    glfwTerminate();
    }

    static void OnKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
        if (action == GLFW_PRESS) {
            Engine::GetInstance().GetInputManager().SetKeyState(key, true);
        } else if (action == GLFW_RELEASE) {
            Engine::GetInstance().GetInputManager().SetKeyState(key, false);
        }
    }

    static void OnMouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
        if (action == GLFW_PRESS) {
            Engine::GetInstance().GetInputManager().SetMouseButtonState(button, true);
        } else if (action == GLFW_RELEASE) {
            Engine::GetInstance().GetInputManager().SetMouseButtonState(button, false);
        }
    }

private:
    GLFWwindow* m_window;
    Application* m_app;
    InputManager m_inputManager;
    PhysicsWorld m_world;
    int m_screenWidth, m_screenHeight;
};
