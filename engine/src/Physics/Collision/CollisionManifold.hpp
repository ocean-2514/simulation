#pragma once

#include <glm/glm.hpp>

#include <array>
#include <cstddef>
#include <cstdint>

inline constexpr std::size_t MaxManifoldPoints = 2;

struct ContactFeatureId {
    uint8_t referenceIsA = 0;
    uint16_t referenceFace = 0;
    uint16_t incidentFace = 0;
    uint8_t incidentVertex = 0;
    uint8_t clipSide = 0;           // 0：原始端点；1/2：由第 1/2 条侧边界产生
};

struct ManifoldPoint {
    glm::vec2 position{0.0f};
    float separation = 0.0f;
    glm::vec2 localPointA{0.0f};
    glm::vec2 localPointB{0.0f};
    ContactFeatureId feature{};
};

struct CollisionManifold {
    bool colliding = false;
    glm::vec2 normal{0.0f};   // A -> B
    std::array<ManifoldPoint, MaxManifoldPoints> points{};
    uint8_t pointCount = 0;

    // 记录参考面来自哪个多边形，位置求解若要跟随参考面旋转会用到。
    uint8_t referenceIsA = 0;
};
