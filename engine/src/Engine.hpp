#pragma once

#include "Application.hpp"
#include "Input/InputManager.hpp"

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

    bool Init() {
        glfwInit();
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        m_window = glfwCreateWindow(m_screenWidth, m_screenHeight, "window", NULL, NULL);
        if (m_window == NULL)
        {
            std::cout << "Failed to create window" << std::endl;
            glfwTerminate();
            return false;
        } 
        glfwMakeContextCurrent(m_window);

        //初始化GLAD块
        if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
        {
            std::cout << "Failed to initialize GLAD" << std::endl;
            return false;
        }

        // glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        glViewport(0, 0, m_screenWidth, m_screenHeight);
        
        glEnable(GL_PROGRAM_POINT_SIZE);  //允许点的大小变化
        glEnable(GL_DEPTH_TEST);   //深度
        glDepthFunc(GL_LESS);

        // callbacks
        glfwSetKeyCallback(m_window, OnKeyCallback);
        glfwSetMouseButtonCallback(m_window, OnMouseButtonCallback);

        // app
        if (m_app) {
            if (!m_app->Init()) {
                return false;
            }
        }

        return true;
    }

    void Run() {
        float lastTime = 0.0f;
        float deltaTime = 0.0f;
        while(!glfwWindowShouldClose(m_window))
        {
            float time = glfwGetTime();
            deltaTime = time - lastTime;
            lastTime = time;

            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


            if (m_app) {
                m_app->Update(deltaTime);
            }
            
            glfwSwapBuffers(m_window);
            glfwPollEvents(); 
        }
    }

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

private:
    Engine(int screenWidth = 1920, int screenHeight = 1080)
    : m_screenWidth(screenWidth), m_screenHeight(screenHeight) {}

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
    int m_screenWidth, m_screenHeight;
};