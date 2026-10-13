#include "Engine.hpp"

bool Engine::Init() {
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

void Engine::Run() {
    float lastTime = 0.0f;
    float deltaTime = 0.0f;
    while(!glfwWindowShouldClose(m_window))
    {
        glfwPollEvents();

        float time = glfwGetTime();
        deltaTime = time - lastTime;
        lastTime = time;

        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        m_world.Step(deltaTime);
        if (m_app) {
            m_app->Update(deltaTime);
        }
        
        glfwSwapBuffers(m_window);
    }
}
