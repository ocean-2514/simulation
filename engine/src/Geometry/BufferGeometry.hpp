#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

inline constexpr float PI = glm::pi<float>();

struct Vertex
{
    glm::vec3 Position;  // 顶点位置
    glm::vec3 Normal;    // 法线
    glm::vec2 TexCoords; // 纹理坐标
};

class BufferGeometry
{
public:
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    unsigned int VAO = 0;
    unsigned int VBO = 0, EBO = 0;

    BufferGeometry() {}
    BufferGeometry(const BufferGeometry& other) = delete;
    BufferGeometry(BufferGeometry&& other) noexcept {
        VAO = other.VAO;
        VBO = other.VBO;
        EBO = other.EBO;
        vertices = std::move(other.vertices);
        indices = std::move(other.indices);
        
        other.VAO = 0;
        other.VBO = 0;
        other.EBO = 0;
    }

    BufferGeometry& operator=(const BufferGeometry& other) = delete;
    BufferGeometry& operator=(BufferGeometry&& other) noexcept {
        if (this == &other) return *this;
        
        if (VAO) glDeleteVertexArrays(1, &VAO);
        if (VBO) glDeleteBuffers(1, &VBO);
        if (EBO) glDeleteBuffers(1, &EBO);

        VAO = other.VAO;
        VBO = other.VBO;
        EBO = other.EBO;
        vertices = std::move(other.vertices);
        indices = std::move(other.indices);
        
        other.VAO = 0;
        other.VBO = 0;
        other.EBO = 0;

        return *this;
    }

    void dispose()
    {
        if (glfwGetCurrentContext() == nullptr) {
        return;
        }

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        if (VAO != 0) glDeleteVertexArrays(1, &VAO);
        if (VBO != 0) glDeleteBuffers(1, &VBO);
        if (EBO != 0) glDeleteBuffers(1, &EBO);

        VAO = 0;
        VBO = 0;
        EBO = 0;
    }

    virtual ~BufferGeometry() {
        dispose();
    }

private:
    glm::mat4 matrix = glm::mat4(1.0f);

protected:

  void setupBuffers()
  {
    if (glfwGetCurrentContext() == nullptr) {
      throw std::runtime_error("OpenGL context is not ready. Create the GLFW window and call gladLoadGLLoader before creating BufferGeometry.");
    }

    if (vertices.empty()) {
      return;
    }

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    if (!indices.empty()) glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    // vertex attribute
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.empty() ? nullptr : &vertices[0], GL_STATIC_DRAW);

    // indixes
    if (EBO != 0) {
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.empty() ? nullptr : &indices[0], GL_STATIC_DRAW);
    }

    // 设置顶点属性指针
    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);

    // Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, Normal));

    // TexCoords
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)offsetof(Vertex, TexCoords));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
  }

  void renewVBO() {
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(Vertex), &vertices[0]);
  }
};