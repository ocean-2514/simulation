#pragma once

#include "Geometry/BufferGeometry.hpp"

class Shape : public BufferGeometry
{
public:
    glm::vec2 centroid{0.0f};
    unsigned int centroidVAO = 0;
    unsigned int centroidVBO = 0;

    Shape() = default;
    Shape(const Shape& other) = delete;
    Shape(Shape&& other) noexcept
        : BufferGeometry(std::move(other)),
          centroid(other.centroid),
          centroidVAO(other.centroidVAO),
          centroidVBO(other.centroidVBO)
    {
        other.centroidVAO = 0;
        other.centroidVBO = 0;
    }
    const Shape& operator=(const Shape& other) = delete;
    Shape& operator=(Shape&& other) noexcept {
        if (this == &other) return *this;

        disposeCentroidBuffers();
        BufferGeometry::operator=(std::move(other));
        centroid = other.centroid;
        centroidVAO = other.centroidVAO;
        centroidVBO = other.centroidVBO;

        other.centroidVAO = 0;
        other.centroidVBO = 0;
        return *this;
    }

    ~Shape() override {
        disposeCentroidBuffers();
    }

    void drawCentroid() const
    {
        if (centroidVAO == 0) {
            return;
        }

        glDisable(GL_DEPTH_TEST);
        glPointSize(10.0f);
        glBindVertexArray(centroidVAO);
        glDrawArrays(GL_POINTS, 0, 1);
        glBindVertexArray(0);
        glEnable(GL_DEPTH_TEST);
    }

protected:
    void setupBuffers()
    {
        BufferGeometry::setupBuffers();

        if (glfwGetCurrentContext() == nullptr) {
            throw std::runtime_error("OpenGL context is not ready. Create the GLFW window before creating Shape.");
        }

        glGenVertexArrays(1, &centroidVAO);
        glGenBuffers(1, &centroidVBO);

        glm::vec3 centroidPosition(centroid.x, centroid.y, 0.0f);

        glBindVertexArray(centroidVAO);
        glBindBuffer(GL_ARRAY_BUFFER, centroidVBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3), &centroidPosition, GL_STATIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), nullptr);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }

private:
    void disposeCentroidBuffers()
    {
        if (glfwGetCurrentContext() == nullptr) {
            return;
        }

        if (centroidVAO != 0) {
            glDeleteVertexArrays(1, &centroidVAO);
        }
        if (centroidVBO != 0) {
            glDeleteBuffers(1, &centroidVBO);
        }

        centroidVAO = 0;
        centroidVBO = 0;
    }
};