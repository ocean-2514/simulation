#pragma once

#include <glm/glm.hpp>

inline constexpr float PI = glm::pi<float>();

struct Vertex
{
    glm::vec3 Position{0.0f};  // 顶点位置
    glm::vec3 Normal{0.0f};    // 法线
    glm::vec2 TexCoords{0.0f}; // 纹理坐标
};