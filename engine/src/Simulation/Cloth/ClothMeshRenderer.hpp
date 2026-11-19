#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <stdexcept>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <memory>

#include "Geometry/GeometryCommon.h"
#include "Simulation/Cloth/ClothGrid.hpp"

class ClothMeshRenderer final
{
public:
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    unsigned int VAO = 0;
    unsigned int VBO = 0, EBO = 0;

    ClothMeshRenderer(std::shared_ptr<ClothGrid> cloth);
    ClothMeshRenderer(const ClothMeshRenderer& other) = delete;
    ClothMeshRenderer(ClothMeshRenderer&& other) = delete;
    ClothMeshRenderer& operator=(const ClothMeshRenderer& other) = delete;
    ClothMeshRenderer& operator=(ClothMeshRenderer&& other) = delete;

    ~ClothMeshRenderer() {
        Dispose();
    }

    void Draw() {
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }

    void RenewVertices();

private:
    void Dispose()
    {
        if (glfwGetCurrentContext() == nullptr) return;

        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        if (VAO != 0) glDeleteVertexArrays(1, &VAO);
        if (VBO != 0) glDeleteBuffers(1, &VBO);
        if (EBO != 0) glDeleteBuffers(1, &EBO);

        VAO = VBO = EBO = 0;
    }

    void SetUpBuffers()
    {
        if (glfwGetCurrentContext() == nullptr) 
            throw std::runtime_error("OpenGL context is not ready. Create the GLFW window and call gladLoadGLLoader before creating BufferGeometry.");

        if (vertices.empty()) return;

        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        if (!indices.empty()) glGenBuffers(1, &EBO);

        glBindVertexArray(VAO);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.empty() ? nullptr : &vertices[0], GL_DYNAMIC_DRAW);
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

    void RenewVBO() {
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.size() * sizeof(Vertex), &vertices[0]);
    }

    std::shared_ptr<ClothGrid> m_cloth;
};