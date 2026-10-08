#pragma once

#include "Geometry/2D/Shape.hpp"

class Circle : public Shape
{
public:
    Circle(float radius = 0.5f, int segments = 64)
    {
        if (segments < 3) {
            segments = 3;
        }

        Vertex center;
        center.Position = glm::vec3(0.0f, 0.0f, 0.0f);
        center.Normal = glm::vec3(0.0f, 0.0f, 1.0f);
        center.TexCoords = glm::vec2(0.5f, 0.5f);
        this->vertices.push_back(center);

        for (int i = 0; i <= segments; ++i)
        {
            float angle = (float)i / segments * glm::two_pi<float>();
            float x = std::cos(angle) * radius;
            float y = std::sin(angle) * radius;

            Vertex vertex;
            vertex.Position = glm::vec3(x, y, 0.0f);
            vertex.Normal = glm::vec3(0.0f, 0.0f, 1.0f);
            vertex.TexCoords = glm::vec2(0.5f + x / (2.0f * radius), 0.5f + y / (2.0f * radius));
            this->vertices.push_back(vertex);
        }

        for (int i = 1; i <= segments; ++i)
        {
            this->indices.push_back(0);
            this->indices.push_back(i);
            this->indices.push_back(i + 1);
        }

        this->setupBuffers();
    }
};

