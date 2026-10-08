#pragma once

#include "Geometry/2D/Shape.hpp"
#include "Utils/MathHelper.hpp"

class Polygon : public Shape
{
public:
    Polygon(const std::vector<glm::vec2>& points)
    {
        if (points.size() < 3) {
            throw std::invalid_argument("A polygon must have at least 3 points.");
        }

        centroid = MathHelper::CalculatePolygonCentroid(points);

        this->vertices.reserve(points.size());
        this->indices.reserve((points.size() - 2) * 3);
    
        Vertex vertex;
        for (const auto& point : points)
        {
            vertex.Position = glm::vec3(point.x, point.y, 0.0f);
            vertex.Normal = glm::vec3(0.0f, 0.0f, 1.0f);
            vertex.TexCoords = glm::vec2(0.0f, 0.0f); // Texture coordinates can be set as needed
            this->vertices.push_back(vertex);
        }
    
        // Triangulate the polygon using a simple fan method
        for (size_t i = 1; i < points.size() - 1; ++i)
        {
            this->indices.push_back(0);
            this->indices.push_back(i);
            this->indices.push_back(i + 1);
        }
    
        this->setupBuffers();
    }
};